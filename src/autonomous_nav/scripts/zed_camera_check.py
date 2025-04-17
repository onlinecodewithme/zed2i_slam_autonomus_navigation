#!/usr/bin/env python3

import rclpy
import time
import sys
import subprocess
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor

class ZedCameraCheck(Node):
    """
    Node to check ZED camera availability and readiness
    """
    def __init__(self):
        super().__init__('zed_camera_check')
        
        # Declare parameters
        self.declare_parameter('max_retries', 5)
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('verbose', True)
        self.declare_parameter('check_timeout', 10.0)
        
        # Get parameters
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_delay = self.get_parameter('retry_delay').value
        self.verbose = self.get_parameter('verbose').value
        self.check_timeout = self.get_parameter('check_timeout').value
        
        # Create publisher to signal camera readiness
        self.camera_ready_pub = self.create_publisher(
            Bool,
            '/zed/camera_ready',
            10
        )
        
        # Create timer for camera checking
        self.check_timer = self.create_timer(0.5, self.check_camera)
        self.retries = 0
        self.camera_ready = False
        
        # Keep track of when we started checking
        self.start_time = time.time()
        
        # Log initialization
        self.get_logger().info('ZED Camera Check Node started')
        
    def log(self, msg, level='info'):
        """Log messages only if verbose is enabled"""
        if self.verbose:
            if level == 'info':
                self.get_logger().info(msg)
            elif level == 'warn':
                self.get_logger().warn(msg)
            elif level == 'error':
                self.get_logger().error(msg)
    
    def run_command(self, cmd):
        """Run a shell command and return the output"""
        try:
            result = subprocess.run(cmd, shell=True, check=True, 
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, 
                                    text=True, timeout=5.0)
            return True, result.stdout
        except subprocess.CalledProcessError as e:
            return False, e.stderr
        except subprocess.TimeoutExpired:
            return False, "Command timed out"
        except Exception as e:
            return False, str(e)
            
    def check_zed_device(self):
        """Check if ZED camera is connected as a device"""
        success, output = self.run_command("lsusb | grep 'Stereo Labs'")
        if success and output:
            self.log(f"ZED camera found: {output.strip()}")
            return True
        else:
            self.log("ZED camera not found in USB devices", level='warn')
            return False
    
    def check_video_device(self):
        """Check if ZED camera is available as a video device"""
        success, output = self.run_command("ls /dev/video*")
        if success and output:
            self.log(f"Video devices available: {output.strip()}")
            return True
        else:
            self.log("No video devices found", level='warn')
            return False
    
    def check_camera_permissions(self):
        """Check if user has permission to access camera devices"""
        success, output = self.run_command("id | grep -c 'video'")
        if success and int(output.strip()) > 0:
            self.log("User has video group permissions")
            return True
        else:
            self.log("User does not have video group permissions. Run: sudo usermod -a -G video $USER", level='error')
            return False
    
    def check_zed_running_processes(self):
        """Check if any ZED processes are already running"""
        success, output = self.run_command("ps aux | grep -E 'zed_node|ZED' | grep -v grep | wc -l")
        if success:
            count = int(output.strip())
            if count > 1:  # More than this checking process
                self.log(f"Found {count} ZED processes already running, which may cause conflicts", level='warn')
                return False
        return True
    
    def check_camera(self):
        """Check if the ZED camera is ready"""
        # Check if we've exceeded the timeout
        if time.time() - self.start_time > self.check_timeout:
            self.log("Camera check timed out", level='error')
            self.publish_camera_status(False)
            self.get_logger().error("ZED camera not ready after timeout. Please check hardware connection.")
            sys.exit(1)
        
        # Skip if camera is already confirmed ready
        if self.camera_ready:
            return
            
        self.log(f"Checking ZED camera availability (attempt {self.retries + 1}/{self.max_retries})")
        
        # Perform checks
        device_check = self.check_zed_device()
        video_check = self.check_video_device()
        permission_check = self.check_camera_permissions()
        process_check = self.check_zed_running_processes()
        
        if device_check and video_check and permission_check and process_check:
            self.log("ZED camera is available and ready")
            self.camera_ready = True
            self.publish_camera_status(True)
            return
            
        # Handle retry logic
        self.retries += 1
        if self.retries >= self.max_retries:
            self.log("Maximum retries reached. Camera not ready.", level='error')
            self.publish_camera_status(False)
            
            # Provide helpful error message based on which check failed
            if not device_check:
                self.get_logger().error("ZED camera not detected as USB device. Check physical connection.")
            elif not video_check:
                self.get_logger().error("ZED camera not available as video device. Check drivers.")
            elif not permission_check:
                self.get_logger().error("User lacks permissions to access camera.")
            elif not process_check:
                self.get_logger().error("Conflicting ZED processes detected. Try killing them first.")
                
            sys.exit(1)
        else:
            self.log(f"Camera not ready. Retrying in {self.retry_delay} seconds...", level='warn')
            time.sleep(self.retry_delay)
    
    def publish_camera_status(self, ready: bool):
        """Publish camera ready status"""
        msg = Bool()
        msg.data = ready
        self.camera_ready_pub.publish(msg)
        
        # Log the final status
        if ready:
            self.get_logger().info("ZED camera is READY. Proceeding with launch sequence.")
        else:
            self.get_logger().error("ZED camera is NOT READY. Launch sequence may fail.")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ZedCameraCheck()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in ZED camera check: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
