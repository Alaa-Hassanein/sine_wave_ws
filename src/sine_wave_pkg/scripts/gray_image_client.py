#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sine_wave_pkg.srv import GetImage
import numpy as np
import matplotlib.pyplot as plt
import os

class ImageClient(Node):

    def __init__(self):
        super().__init__('image_client')
        self.client = self.create_client(GetImage, 'get_image')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = GetImage.Request()

    def send_request(self, input_path):
        self.request.input_path = input_path
        
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def get_workspace_directory(current_path):

        path_parts = current_path.split(os.sep)

        if "install" in path_parts:
            workspace_index = path_parts.index("install") - 1
        elif "src" in path_parts:
            workspace_index = path_parts.index("src") - 1
        else:
            raise ValueError("Workspace directory not found in the path")

        workspace_directory = os.sep.join(path_parts[: workspace_index + 1])
        return workspace_directory

def main(args=None):
    rclpy.init(args=args)
    image_client = ImageClient()
    
    workspace_directory = ImageClient.get_workspace_directory(os.path.abspath(__file__))
    image_dir = os.path.join(workspace_directory, "src", "sine_wave_pkg", "image")

    
    response = image_client.send_request(os.path.join(image_dir, "test.jpg"))
    print(response)
    if response.success:
        print("Received grayscale image:")
        
        # Convert the received image data back to a NumPy array
        image_data = np.array(response.message, dtype=np.uint8)
        gray_image = image_data.reshape((response.height, response.width))

        # Save the image to a file
        output_path = os.path.join(image_dir, "output.jpg")
        plt.imsave(output_path, gray_image, cmap='gray')
        print(f"Grayscale image saved to {output_path}")

        # Plot the image using matplotlib
        plt.imshow(gray_image, cmap='gray')
        plt.title('Grayscale Image')
        plt.axis('off')
        plt.show()
        # Save the image to a file
    else:
        print("Failed to get image")

    rclpy.shutdown()

if __name__ == '__main__':
    main()