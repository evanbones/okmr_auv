import okmr_object_detection.detector 
from okmr_object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import cv2
import ultralytics

class ExampleDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="gate_detector")
        self.model = YOLO("gate.pt")  # Update with the actual model path

    def inference(self, rgb, depth):
        results = self.model(rgb)  # Run YOLO inference

        # Create an empty label image
        label_img = np.zeros(self.target_size, dtype=np.float32)

        # Process detected objects
        for result in results:
            for box in result.boxes.xyxy:  # Bounding box format: (x1, y1, x2, y2)
                x1, y1, x2, y2 = map(int, box)  
                cv2.rectangle(label_img, (x1, y1), (x2, y2), 1.0, -1)  # Draw bounding box

        #cv2.imshow("label", label_img[:, :, 0])
        #cv2.waitKey(1)

        return label_img
    
def main(args=None):
    rclpy.init(args=args)
    node = ExampleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

