#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from exercise_image.srv import ImageAngle
import cv2
from cv_bridge import CvBridge

class ExerciseImageClient(Node):
    def __init__(self):
        super().__init__("exercise_image_client")
        self.client = self.create_client(ImageAngle, "/exercise_image")
        self.request = ImageAngle.Request()
    
    def send_request(self, request_msg):
        self.request.degree_turn = float(request_msg) 
        self.client.wait_for_service()
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        return self.response.image

    def display_image(self, angle):
        image_response = self.send_request(angle)
        my_image = CvBridge().imgmsg_to_cv2(image_response)
        cv2.imshow(f"Camera in {angle}°", my_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
def main():
    rclpy.init()
    print("Initializing client node!")
    my_node = ExerciseImageClient()
    try:
        my_angle = int(input("Enter an angle: "))
        my_node.display_image(my_angle)
        print("Done!")
    except KeyboardInterrupt:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()