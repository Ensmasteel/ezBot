#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from cv2 import aruco
import numpy as np

class camerapubsub(Node):

    def __init__(self):
        super().__init__('camerapubsub')
        self.publisher = self.create_publisher(Image
, 'camera_namespace/camera2/computed_image', 10)
        self.subscription = self.create_subscription(
            Image,
            'camera_namespace/camera2/image_raw',
            self.image_callback,
            10)
        self.cv_bridge = CvBridge()
        fx,fy=1280,1280
        cx,cy=fx/2,fy/2

        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.distortion_coefficients = np.array([0, 0, 0, 0, 0])

    def image_callback(self,msg):
        marker_size=8
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        arucoParams=cv2.aruco.DetectorParameters_create()
        arucoParams.adaptiveThreshWinSizeMin=141
        arucoParams.adaptiveThreshWinSizeMax=251
        arucoParams.adaptiveThreshWinSizeStep=20
        arucoParams.adaptiveThreshConstant=4
        #incorrect aruco detection with bigger markers 2811
        markers, ids, regegted = cv2.aruco.detectMarkers(cv_image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),parameters=arucoParams)

        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):

                cv2.aruco.drawDetectedMarkers(cv_image, markers, ids)
                cv2.aruco.drawDetectedMarkers(cv_image, regegted, borderColor=(255,0,0))
                [rvecs, tvec, objPoints]= cv2.aruco.estimatePoseSingleMarkers(markers[i], marker_size, self.camera_matrix, self.distortion_coefficients)
                distance = tvec[0]
                #text = f"ID: {ids[i]}, x : {distance[0]} y :{distance[1]} z: {distance[2]}"
                cv2.putText(cv_image, f"{distance}", (int(markers[i][0, 0, 0]), int(markers[i][0, 0, 1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow('image',cv_image)
        processed_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(processed_msg)

def main(args=None):

    rclpy.init(args=args)
    camps = camerapubsub()
    rclpy.spin(camps)
    camps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()







