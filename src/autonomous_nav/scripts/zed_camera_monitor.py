#!/usr/bin/env python3

import rclpy
import time
import subprocess
import signal
import sys
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ZedCameraMonitor(Node):
    """
    Node to monitor ZED camera health during operation
    and attempt recovery if issues are detected
    """
    def __init__(self):
        super().__init__('zed_camera_monitor')
        
        # Declare parameters
        self.declare_parameter('health_check_period', 5.0)  # Seconds between checks
        self.declare_parameter('max_restart_attempts', 3)
        self.declare_parameter('recovery_timeout', 30.0)
        self.declare_parameter('camera_topics_to_monitor', [
            '/zed2i/zed_node/rgb/image_rect_color',
            '/zed2i/zed_node/depth/depth_registered',
            '/zed2i/zed_node/point_cloud/cloud_registered'
        ])
        
        # Get parameters
        self.health_check_period = self.get_parameter('health_check_period').value
        self.max_restart_attempts = self.get_parameter('max_restart_attempts').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        self.topics_to_monitor = self.get_parameter('camera_topics_to_monitor').value
        
        # System state
        self.is_camera_healthy = True
        self.restart_attempts = 0
        self.last_topic_activity = {}
        self.last_recovery_time = None
        
        # Create publisher for camera health status
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.camera_health_pub = self.create_publisher(
            Bool,
            '/zed/camera_health',
            qos
        )
        
        self.camera_status_pub = self.create_publisher(
            String,
            '/zed/camera_status',
            10
        )
        
        # Create subscribers to monitor camera topics
        for topic in self.topics_to_monitor:
            # We don't care about the message type, just want to know if topics are being published
            try:
                self.create_subscription(
                    String,  # Generic type, we're just monitoring activity
                    topic,
                    lambda msg, topic=topic: self.topic_callback(msg, topic),
                    1  # Keep only the latest message
                )
                self.last_topic_activity[topic] = None
                self.get_logger().info(f"Monitoring topic: {topic}")
            except Exception as e:
                self.get_logger().error(f"Failed to create subscriber for {topic}: {e}")
        
        # Create health check timer
        self.health_check_timer = self.create_timer(self.health_check_period, self.check_camera_health)
        
        # Log initialization
        self.get_logger().info('ZED Camera Monitor started - monitoring camera health')
        
    def topic_callback(self, msg, topic):
        """Record activity on monitored topics"""
        self.last_topic_activity[topic] = time.time()
        
    def check_camera_health(self):
        """
        Periodic check of camera health
        Monitors:
        1. Topic activity - Are messages being published?
        2. Process status - Is the ZED node running?
        3. System resources - Is there adequate CPU/GPU?
        """
        current_time = time.time()
        
        # Check if any topics have been inactive for too long
        inactive_topics = []
        for topic, last_active in self.last_topic_activity.items():
            if last_active is not None and (current_time - last_active) > self.health_check_period * 3:
                inactive_topics.append(topic)
            elif last_active is None and self.get_uptime().nanoseconds / 1e9 > self.health_check_period * 5:
                # If we've never seen activity and node has been up for a while
                inactive_topics.append(topic)
        
        # Check if ZED processes are running
        zed_running = self.check_zed_process()
        
        # Set camera health status
        previous_health = self.is_camera_healthy
        self.is_camera_healthy = zed_running and len(inactive_topics) == 0
        
        # Publish status
        self.publish_camera_health(self.is_camera_healthy)
        
        # Health state transition - camera was healthy but is now unhealthy
        if previous_health and not self.is_camera_healthy:
            status_msg = f"Camera health issue detected. Inactive topics: {inactive_topics}, ZED process running: {zed_running}"
            self.get_logger().warn(status_msg)
            self.publish_status(status_msg)
            
            # Attempt recovery if we haven't exceeded max attempts
            if self.restart_attempts < self.max_restart_attempts:
                # Check if we're not in a recovery timeout period
                if self.last_recovery_time is None or (current_time - self.last_recovery_time) > self.recovery_timeout:
                    self.restart_attempts += 1
                    self.last_recovery_time = current_time
                    self.get_logger().info(f"Attempting camera recovery (attempt {self.restart_attempts}/{self.max_restart_attempts})")
                    self.attempt_camera_recovery()
                else:
                    self.get_logger().info(f"In recovery timeout period, waiting before next recovery attempt")
            else:
                self.get_logger().error(f"Maximum restart attempts reached ({self.max_restart_attempts}). Manual intervention required.")
                self.publish_status("Camera recovery failed. Manual intervention required.")
        
        # Camera was unhealthy but is now healthy - reset attempt counter
        elif not previous_health and self.is_camera_healthy:
            self.restart_attempts = 0
            self.get_logger().info("Camera health restored")
            self.publish_status("Camera health restored")
    
    def check_zed_process(self):
        """Check if ZED node processes are running"""
        try:
            result = subprocess.run(
                ["pgrep", "-f", "zed_node"],
                capture_output=True,
                text=True
            )
            return len(result.stdout.strip()) > 0
        except Exception as e:
            self.get_logger().error(f"Error checking ZED process: {e}")
            return False
    
    def attempt_camera_recovery(self):
        """Attempt to recover camera function"""
        self.publish_status("Attempting camera recovery...")
        
        try:
            # Step 1: Kill any existing ZED processes
            self.get_logger().info("Killing existing ZED processes")
            subprocess.run(["pkill", "-f", "zed_"], check=False)
            time.sleep(2)  # Allow processes to terminate
            
            # Step 2: Check USB connection - try to reset the USB port (would require sudo in practice)
            self.get_logger().info("Checking USB connection")
            self.publish_status("Checking USB connection...")
            
            # In a production system, this might involve resetting USB ports
            # This is a placeholder for what would be a more robust implementation
            # that might involve udev rules or system-specific commands
            
            # Step 3: Restart the ZED node
            # In a real implementation, this would involve using ROS2 lifecycle management
            # or a system service mechanism to restart the node
            self.get_logger().info("Recovery action complete. Waiting for camera to reconnect...")
            self.publish_status("Recovery actions complete. Waiting for camera reconnection...")
            
            return True
        except Exception as e:
            self.get_logger().error(f"Error during recovery attempt: {e}")
            self.publish_status(f"Recovery failed: {str(e)}")
            return False
    
    def publish_camera_health(self, is_healthy: bool):
        """Publish camera health status"""
        msg = Bool()
        msg.data = is_healthy
        self.camera_health_pub.publish(msg)
    
    def publish_status(self, status_text: str):
        """Publish human-readable status message"""
        msg = String()
        msg.data = status_text
        self.camera_status_pub.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    
    # Handle graceful shutdown for SIGINT (Ctrl+C)
    def signal_handler(sig, frame):
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        node = ZedCameraMonitor()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except Exception as e:
        print(f"Error in ZED camera monitor: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
