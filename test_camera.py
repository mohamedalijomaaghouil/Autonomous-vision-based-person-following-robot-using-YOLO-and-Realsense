import cv2

from ultralytics import YOLO


class DistanceMeasurement:
    def __init__(self, model_path='yolo11n.pt', known_height=1.7, focal_length=None):
        """
        Initialize the distance measurement class

        Parameters:
            model_path: Path to YOLOv11 model
            known_height: Average human height in meters (1.7m default)
            focal_length: Camera focal length (will be calibrated if None)
        """
        # Load YOLOv11 model
        self.model = YOLO(model_path)

        # Known parameters
        self.known_height = known_height  # Average human height in meters
        self.focal_length = focal_length  # Will be calibrated if None

        # For calibration
        self.calibrated = False if focal_length is None else True
        self.calibration_distance = 3.0  # Default calibration distance in meters

        def get_wheel_speeds(self, distance):
            """
        Determine wheel speeds based on the distance to the person.
        
        Returns a tuple (left_speed, right_speed)
        """
        distance=0;
        if distance is None:
            return (0, 0)

        if distance < 1.0:
            return (0, 0)  # Too close, stop
        elif 1.0 <= distance < 2.0:
            return (50, 50)  # Medium speed
        else:
            return (100, 100)  # Max speed to catch up


    def calibrate(self, frame, bbox, distance=None):
        """
        Calibrate focal length using a person at a known distance

        Parameters:
            frame: Current video frame
            bbox: Bounding box of the person [x1, y1, x2, y2]
            distance: Known distance of the person from camera (meters)
        """
        if distance is not None:
            self.calibration_distance = distance

        # Calculate person height in pixels
        height_pixels = bbox[3] - bbox[1]

        # Calculate focal length: F = (P * D) / W
        # Where F is focal length, P is height in pixels, D is known distance, W is real height
        self.focal_length = (height_pixels * self.calibration_distance) / self.known_height
        self.calibrated = True

        return self.focal_length

    def calculate_distance(self, height_pixels):
        """
        Calculate distance using the formula: D = (W * F) / P
        Where D is distance, W is known height, F is focal length, P is height in pixels
        """
        if not self.calibrated:
            return None

        if height_pixels <= 0:
            return None

        distance = (self.known_height * self.focal_length) / height_pixels
        return distance

    def process_frame(self, frame, calibration_mode=False):
        """
        Process a frame to detect people and measure distance

        Parameters:
            frame: Current video frame
            calibration_mode: Whether to use this frame for calibration

        Returns:
            Processed frame with distance measurements
            List of detected people with distances
        """
        # Detect objects in the frame
        results = self.model(frame)

        # Get detections
        detections = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0].item())
                conf = box.conf[0].item()

                # Class 0 is person in COCO dataset
                if cls == 0 and conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    height_pixels = y2 - y1

                    # If in calibration mode, use this detection for calibration
                    if calibration_mode and not self.calibrated:
                        self.calibrate(frame, [x1, y1, x2, y2])
                        cv2.putText(frame, "Calibrated!", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    # Calculate distance if calibrated
                    distance = None
                    if self.calibrated:
                        distance = self.calculate_distance(height_pixels)

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Add distance information
                    if distance:
                        distance_text = f"Distance: {distance:.2f}m"
                        cv2.putText(frame, distance_text, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                        detections.append({
                            "bbox": [x1, y1, x2, y2],
                            "distance": distance,
                            "confidence": conf
                        })

        return frame, detections
        if distance:
            wheel_speeds = self.get_wheel_speeds(distance)
            print(f"Wheel Speeds (L,R): {wheel_speeds}")


def main():
    # Initialize video capture (0 for webcam)
    cap = cv2.VideoCapture(0)

    # Initialize distance measurement
    distance_measure = DistanceMeasurement()

    # Flag for calibration mode
    calibration_mode = True

    while True:
        # Read frame
        ret, frame = cap.read()
        if not ret:
            break

        # Process frame
        processed_frame, detections = distance_measure.process_frame(frame, calibration_mode)

        # After first detection, turn off calibration mode
        if calibration_mode and distance_measure.calibrated:
            calibration_mode = False
            print(f"Calibration complete. Focal length: {distance_measure.focal_length:.2f}")

        # Display status
        status = "CALIBRATION MODE - Stand 3 meters from camera" if calibration_mode else "DISTANCE MEASUREMENT MODE"
        cv2.putText(processed_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Display frame
        cv2.imshow('YOLOv11 Distance Measurement', processed_frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()