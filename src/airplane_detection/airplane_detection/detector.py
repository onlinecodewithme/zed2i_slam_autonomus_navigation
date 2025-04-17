#!/usr/bin/env python3

import cv2
import numpy as np
import torch
import os
import sys
from pathlib import Path
import time


class AirplaneDetector:
    """
    YOLOv5-based airplane detector for the ZED 2i camera
    """
    
    def __init__(self, model_path=None, conf_threshold=0.5, nms_threshold=0.45):
        """
        Initialize the airplane detector
        
        Args:
            model_path: Path to the YOLOv5 model
            conf_threshold: Confidence threshold for detections
            nms_threshold: Non-maximum suppression threshold
        """
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        # Default model path if none provided
        if model_path is None:
            # Get the path to this package
            package_path = Path(__file__).parent.parent
            model_path = str(package_path / 'config' / 'yolov5s.pt')
        
        # Load YOLOv5 model
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
            
            # Set confidence threshold
            self.model.conf = self.conf_threshold
            
            # Set IOU threshold for NMS
            self.model.iou = self.nms_threshold
            
            # Reduce multiple detections with agnostic NMS
            self.model.agnostic = True
            
            # Set device (GPU if available, otherwise CPU)
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model.to(self.device)
            
            print(f"Loaded YOLOv5 model from {model_path} on {self.device}")
            
            # Get the classes that the model can detect
            self.class_names = self.model.names
            print(f"Model can detect these classes: {self.class_names}")
            
            # Class IDs for airplanes/aircraft (modify based on your model)
            self.airplane_classes = ['airplane', 'aircraft', 'plane', 'airliner', 'jet']
            self.airplane_ids = [
                i for i, name in enumerate(self.class_names) 
                if any(ac in name.lower() for ac in self.airplane_classes)
            ]
            
            if not self.airplane_ids:
                print("Warning: No airplane classes found in the model. Using class 0 as fallback.")
                self.airplane_ids = [0]  # Use the first class as fallback
                
            print(f"Using these class IDs for airplanes: {self.airplane_ids}")
            
        except Exception as e:
            print(f"Error loading YOLOv5 model: {e}")
            print("Using fallback detection method...")
            self.model = None
    
    def detect(self, image):
        """
        Detect airplanes in the image
        
        Args:
            image: RGB image as numpy array
            
        Returns:
            List of detections [x, y, w, h, confidence, class_id]
        """
        if self.model is None:
            # Fallback method - use simple color detection
            return self._fallback_detect(image)
        
        # Perform inference with YOLOv5
        results = self.model(image)
        
        # Convert results to numpy array
        detections = results.xyxy[0].cpu().numpy()
        
        # Filter detections to only include airplanes
        airplane_detections = []
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            if int(class_id) in self.airplane_ids:
                # Convert to [x, y, w, h, conf, class_id] format
                x, y = int(x1), int(y1)
                w, h = int(x2 - x1), int(y2 - y1)
                airplane_detections.append([x, y, w, h, conf, int(class_id)])
        
        return airplane_detections
    
    def _fallback_detect(self, image):
        """
        Simple fallback airplane detection based on color and shape
        Use this for testing when YOLO model is not available
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blur, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size and shape
        airplane_detections = []
        for contour in contours:
            # Calculate area and discard small contours
            area = cv2.contourArea(contour)
            if area < 1000:  # Minimum area threshold
                continue
                
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate aspect ratio
            aspect_ratio = float(w) / h
            
            # Airplanes typically have aspect ratios > 1 (wider than tall)
            if 1.5 < aspect_ratio < 6.0:
                # This is a simple heuristic for airplane-like shapes
                airplane_detections.append([x, y, w, h, 0.5, 0])  # 0.5 confidence, class_id 0
        
        return airplane_detections
        
    def estimate_distance(self, detection, depth_image):
        """
        Estimate the distance to the detected airplane using depth information
        
        Args:
            detection: [x, y, w, h, conf, class_id]
            depth_image: Depth image from ZED camera
            
        Returns:
            Estimated distance in meters
        """
        x, y, w, h = detection[:4]
        
        # Calculate center of bounding box
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Get depth at center point
        if center_y < depth_image.shape[0] and center_x < depth_image.shape[1]:
            center_depth = depth_image[center_y, center_x]
        else:
            center_depth = 0
            
        # If depth is not available at center, sample multiple points
        if center_depth == 0:
            # Sample multiple points within the bounding box
            sample_points = []
            num_samples = 9  # 3x3 grid
            for i in range(3):
                for j in range(3):
                    sample_x = x + (w * (i + 1)) // 4
                    sample_y = y + (h * (j + 1)) // 4
                    if sample_y < depth_image.shape[0] and sample_x < depth_image.shape[1]:
                        sample_depth = depth_image[sample_y, sample_x]
                        if sample_depth > 0:
                            sample_points.append(sample_depth)
            
            if sample_points:
                # Calculate median depth to remove outliers
                center_depth = np.median(sample_points)
        
        # Convert depth to meters (assuming depth image is in millimeters)
        distance_meters = center_depth / 1000.0
        
        return distance_meters
        
    def draw_detections(self, image, detections, distance_info=None):
        """
        Draw bounding boxes and labels on the image
        
        Args:
            image: RGB image to draw on
            detections: List of detections [x, y, w, h, confidence, class_id]
            distance_info: Optional list of distance values
            
        Returns:
            Image with drawn detections
        """
        result_img = image.copy()
        
        for i, det in enumerate(detections):
            x, y, w, h, conf, class_id = det
            
            # Draw bounding box
            cv2.rectangle(result_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Prepare label text
            if class_id < len(self.class_names) if hasattr(self, 'class_names') else 0:
                class_name = self.class_names[int(class_id)]
            else:
                class_name = "airplane"
                
            label = f"{class_name}: {conf:.2f}"
            
            # Add distance if available
            if distance_info is not None and i < len(distance_info):
                label += f" {distance_info[i]:.2f}m"
                
            # Draw label background
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(result_img, (x, y - 25), (x + label_size[0], y), (0, 255, 0), -1)
            
            # Draw label text
            cv2.putText(result_img, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
        return result_img

# Test function to run the detector on a single image
def test_detector(image_path):
    detector = AirplaneDetector()
    image = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    start_time = time.time()
    detections = detector.detect(image_rgb)
    elapsed = time.time() - start_time
    
    print(f"Detection time: {elapsed:.2f} seconds")
    print(f"Found {len(detections)} airplanes")
    
    result = detector.draw_detections(image, detections)
    cv2.imshow("Airplane Detections", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Example usage
    if len(sys.argv) > 1:
        test_detector(sys.argv[1])
    else:
        print("Please provide an image path")
