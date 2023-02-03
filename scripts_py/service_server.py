#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from exercise_image.srv import ImageAngle
import cv2
from cv_bridge import CvBridge

class ExerciseImageServer(Node):
    def __init__(self):
        super().__init__("exercise_image_client")
        self.server = self.create_service(ImageAngle, "/exercise_image", self.turn_image)
        self.angles_available = [-30, -15, 0, 15, 30]
    
    def turn_image(self, request, response):
        if request.degree_turn in self.angles_available:
            path = f"/home/usuario/udemy_ros2_ws/src/exercise_image/images/{int(request.degree_turn)}.png"
            my_image = cv2.imread(path)
            print(f"Path: {path}")
            ros_image = CvBridge().cv2_to_imgmsg(my_image, "bgr8")
            response.image = ros_image
        else:
            return
        return response
            
def main():
    rclpy.init()
    print("Initializing server node!")
    my_node = ExerciseImageServer()
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()