import okmr_object_detection.detector 
from okmr_object_detection.detector import ObjectDetectorNode 
import rclpy
import numpy as np
import cv2
import os
from datetime import datetime

class DataRecorder(ObjectDetectorNode):
    def __init__(self):
        super().__init__(node_name="data_recorder")
        self.counter = 0
        self.save_path = "data/" 
        start_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.run_folder = os.path.join(self.save_path, start_time)
    
        # Make sure the folder exists
        os.makedirs(self.run_folder, exist_ok=True)

    def inference(self,rgb,depth):
        np.save(os.path.join(self.run_folder, f"rgb_{self.counter}.npy"), rgb)  # Save RGB array
        np.save(os.path.join(self.run_folder, f"depth_{self.counter}.npy"), depth)  # Save Depth array
        self.counter += 1

        label_img = np.zeros(self.target_size, dtype=np.uint8)
        return label_img

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

