#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectRecognitionNode(Node):
    def __init__(self):
        super().__init__('object_recognition_node')
        
        # CV Bridge pentru conversie ROS2 -> OpenCV
        self.bridge = CvBridge()
        
        # Subscribe la topicul camerei
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher pentru imaginea procesată
        self.processed_image_pub = self.create_publisher(
            Image, 
            '/camera/color/processed_image', 
            10
        )
        
        # Folosim doar detecția bazată pe culoare care merge sigur
        self.use_color_detection = True
        
        self.get_logger().info('Object Recognition Node started - Using color-based detection')
    
    def image_callback(self, msg):
        """Callback pentru procesarea imaginii"""
        try:
            # Converteste ROS Image -> OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Procesează imaginea pentru recunoaștere
            processed_image = self.detect_objects(cv_image)
            
            # Arată imaginea procesată în fereastră
            cv2.imshow('Object Recognition - Press Q to quit', processed_image)
            cv2.waitKey(1)
            
            # Publică imaginea procesată
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_objects(self, image):
        """Detectează obiecte în imagine bazat pe culoare"""
        # Facem o copie a imaginii originale
        result = image.copy()
        
        # Converteste în HSV pentru detecție mai bună a culorilor
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Definește range-uri de culoare pentru diferite obiecte
        colors = {
            'RED': [
                ([0, 120, 70], [10, 255, 255]),      # Red lower range
                ([170, 120, 70], [180, 255, 255])    # Red upper range
            ],
            'BLUE': ([100, 150, 0], [140, 255, 255]),
            'GREEN': ([40, 40, 40], [80, 255, 255]),
            'YELLOW': ([20, 100, 100], [30, 255, 255]),
            'ORANGE': ([10, 100, 20], [25, 255, 255]),
            'PURPLE': ([130, 50, 50], [160, 255, 255])
        }
        
        detected_objects = 0
        
        for color_name, ranges in colors.items():
            # Pentru roșu avem două range-uri
            if color_name == 'RED':
                mask1 = cv2.inRange(hsv, np.array(ranges[0][0]), np.array(ranges[0][1]))
                mask2 = cv2.inRange(hsv, np.array(ranges[1][0]), np.array(ranges[1][1]))
                mask = mask1 + mask2
            else:
                mask = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))
            
            # Operații morfologice pentru a îmbunătăți masca
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Găsește contururi
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:  # Filtrează contururi mici (zgomot)
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculează centrul obiectului
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Alege culoarea pentru bounding box
                    if color_name == 'RED':
                        color = (0, 0, 255)  # Roșu în BGR
                    elif color_name == 'BLUE':
                        color = (255, 0, 0)  # Albastru în BGR
                    elif color_name == 'GREEN':
                        color = (0, 255, 0)  # Verde în BGR
                    elif color_name == 'YELLOW':
                        color = (0, 255, 255)  # Galben în BGR
                    elif color_name == 'ORANGE':
                        color = (0, 165, 255)  # Portocaliu în BGR
                    else:
                        color = (255, 0, 255)  # Mov în BGR
                    
                    # Desenează bounding box
                    cv2.rectangle(result, (x, y), (x + w, y + h), color, 3)
                    
                    # Desenează centrul obiectului
                    cv2.circle(result, (center_x, center_y), 5, color, -1)
                    
                    # Adaugă text cu numele culorii și coordonatele
                    text = f'{color_name} ({center_x},{center_y})'
                    cv2.putText(result, text, (x, y - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    # Desenează linie de la centru la margine pentru orientare
                    cv2.line(result, (center_x, center_y), (center_x + 50, center_y), color, 2)
                    
                    detected_objects += 1
        
        # Adaugă informații generale pe imagine
        cv2.putText(result, f'Objects detected: {detected_objects}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, 'Press Q to quit', (10, result.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return result

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down object recognition...")
    finally:
        # Curăță fereastra OpenCV
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()