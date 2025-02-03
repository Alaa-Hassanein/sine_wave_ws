#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sine_wave_pkg.srv import GetImage
import cv2
import numpy as np

class ImageServer(Node):

    def __init__(self):
        super().__init__('image_server')
        self.srv = self.create_service(GetImage, 'get_image', self.get_image_callback)

    def get_image_callback(self, request, response):
        input_path = request.input_path
        try:
            # Read the image from the path
            image = cv2.imread(input_path)
            if image is None:
                response.success = False
                response.message = "Image not found"
                return response

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            image_array = np.array(gray_image)
            image_data = image_array.flatten().tolist()
            

            response.success = True
            response.message = image_data
            response.height, response.width = image_array.shape[:2]

        except Exception as e:
            response.success = False
            

        return response

def main(args=None):
    rclpy.init(args=args)
    image_server = ImageServer()
    rclpy.spin(image_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()