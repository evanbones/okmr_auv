import object_detection.detector
from object_detection.detector import ObjectDetectorNode
import rclpy
import numpy as np
import torch
from ultralytics import YOLO
import cv2

class LidDetector(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="lid_detector")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load YOLOv11 model
        self.model = YOLO("LidModelRGB.pt")
        #self.model.eval()

        # Define input size for YOLO
        self.img_size = 640  # Adjust based on your training

    def preprocess_image(self, rgb): #
        # Prepares an RGB image (NumPy array) for YOLO model inference.
        

        # Ensure the image is in float32 format
        img = rgb.astype(np.float32) / 255.0  # Normalize to [0,1]

        # Resize to (img_size, img_size) while keeping channels first (C, H, W)
        img = np.transpose(img, (2, 0, 1))  # Convert from (H, W, C) to (C, H, W)
        img = torch.tensor(img).unsqueeze(0).to(self.device)  # Add batch dimension
        
        return img

    def decode_predictions(self, output, orig_shape, conf_threshold=0.5):
        
        boxes = []
        output = output.cpu().detach().numpy()  # Move output to CPU

        for det in output[0]:  # Loop through detections
            x, y, w, h, confidence, class_probs = det[:4], det[4], det[5:]

            if confidence < conf_threshold:
                continue

            class_id = np.argmax(class_probs)
            confidence = confidence.item()

            # Convert YOLO's box format (center x, center y, w, h) to (x_min, y_min, x_max, y_max)
            x_min = int((x - w / 2) * orig_shape[1])  # Scale back to original image size
            y_min = int((y - h / 2) * orig_shape[0])
            x_max = int((x + w / 2) * orig_shape[1])
            y_max = int((y + h / 2) * orig_shape[0])

            boxes.append([x_min, y_min, x_max, y_max, confidence, class_id])

        return np.array(boxes)

    def inference(self, rgb, depth):
        # returns list of bounding boxes
        label_img = np.zeros(self.target_size, dtype=np.uint8)
        #input_tensor = self.preprocess_image(rgb)
        
        #with torch.no_grad():
        output = self.model(rgb)  # Inference

        print(output)

        #detected_boxes = self.decode_predictions(output, rgb.shape[:2])
        for result in output:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cv2.rectangle(label_img, (int(x1), int(y1)), (int(x2), int(y2)), 65535, -1)# for each bounding box, not worrying about overlapping boxes for now

        # If no detections, return an empty array matching target size
        #if detected_boxes.size == 0:
            #return np.zeros(self.target_size, dtype=np.float32)
        return label_img
        #return detected_boxes  

def main(args=None):
    rclpy.init(args=args)
    node = LidDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
