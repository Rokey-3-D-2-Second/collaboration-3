#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO + ArUco 통합 감지 노드

Author: Rujin Kim
Date: 2025-07-01
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32
from aruco_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np
import json
import time
from ultralytics import YOLO
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

# 카메라 보정 파라미터
mtx = np.array([
    [1381.01926, 0., 695.134361],
    [0., 1382.09839, 446.413163],
    [0., 0., 1.]
])
dist = np.array([[-0.01792346, 0.68685818, 0.0023631, -0.00455134, -2.06831632]])
pix_2_mm = 0.000153  # 필요시 조정

# ArUco 관련 함수들
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs, []

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)

def load_camera_parameters(yaml_file):
    package_share_directory = get_package_share_directory('aruco_yolo')
    calibration_file = os.path.join(package_share_directory, 'config', yaml_file)
    with open(calibration_file, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
    return camera_matrix, dist_coeffs

def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(image)
    detect_data = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        
        if rvecs is not None and tvecs is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw, pitch, roll = rotationMatrixToEulerAngles(rot_mat)
                marker_pos = np.dot(-rot_mat.T, tvec).flatten()
                distance = np.linalg.norm(tvec)
                detect_data.append([marker_id, marker_pos, (yaw, pitch, roll), distance])
    return image, detect_data

class YoloArucoDetector(Node):
    def __init__(self):
        super().__init__('yolo_aruco_detector')
        
        # YOLO 모델 로드
        model_path = "/home/rokey-jw/rokeyprj_ws/src/aruco_yolo/models/new_new_new_best.pt"
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded from {model_path}")

        # 카메라 보정 파라미터 로드
        self.camera_matrix, self.dist_coeffs = load_camera_parameters('calibration_params.yaml')
        self.marker_size = 0.04

        # 퍼블리셔/서브스크라이버 설정
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            1
        )
        
        # YOLO 관련 퍼블리셔
        self.img_publisher = self.create_publisher(CompressedImage, 'yolo/compressed', 1)
        self.info_publisher = self.create_publisher(String, 'yolo/detected_info', 5)
        
        # ArUco 관련 퍼블리셔
        self.marker_publisher = self.create_publisher(MarkerArray, 'detected_markers', 10)
        self.distance_publisher = self.create_publisher(Float32, '/aruco/distance', 10)
        
        self.last_pub_time = time.time()
        self.bridge = CvBridge()

    def publish_img(self, frame):
        time_cur = time.time()
        # if time_cur - 0.2 < self.last_pub_time:
        #     return
        self.last_pub_time = time_cur
        
        # 이미지 압축 및 퍼블리시
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
        _, compressed_image = cv2.imencode('.jpg', frame, encode_param)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = compressed_image.tobytes()
        self.img_publisher.publish(msg)

    def convert_box_center_2_mm(self, results, img_size):
        cls = results[0].boxes.cls.tolist()
        centers = []
        centers_img = []
        for i, r in enumerate(results[0].boxes.xywh):
            d = r.tolist()
            centers.append( (int(cls[i]), (d[0] - img_size[0]/2)*pix_2_mm, (d[1] - img_size[1]/2)*pix_2_mm) )
            centers_img.append( (d[0], d[1]) )
        return centers, centers_img

    def image_callback(self, msg):
        # 이미지 디코딩 및 보정
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w = image_np.shape[:2]
        newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        image_undistort = cv2.undistort(image_np, mtx, dist, None, newcameramtx)

        # YOLO 추론 실행
        results = self.model(image_undistort, conf=0.4)
        annotated_img = results[0].plot()
        
        # ArUco 마커 검출
        frame_aruco, detect_data = detect_markers(image_undistort.copy(), 
                                                 self.camera_matrix, 
                                                 self.dist_coeffs, 
                                                 self.marker_size)
        
        # ArUco 결과를 YOLO 이미지에 오버레이
        annotated_img = cv2.addWeighted(annotated_img, 0.7, frame_aruco, 0.3, 0)
        
        # ArUco 결과 처리 및 퍼블리시
        min_distance = float('inf')
        marker_array = MarkerArray()
        
        if detect_data:
            closest_marker = min(detect_data, key=lambda x: x[3])
            min_distance = closest_marker[3]
            
            # 거리 정보 퍼블리시
            distance_msg = Float32()
            distance_msg.data = min_distance
            self.distance_publisher.publish(distance_msg)
            
            # 마커 정보 퍼블리시
            for marker in detect_data:
                marker_msg = Marker()
                marker_msg.id = int(marker[0])
                marker_msg.pose.pose.position.x = marker[1][0]
                marker_msg.pose.pose.position.y = marker[1][1]
                marker_msg.pose.pose.position.z = marker[1][2]
                marker_msg.pose.pose.orientation.x = marker[2][0]
                marker_msg.pose.pose.orientation.y = marker[2][1]
                marker_msg.pose.pose.orientation.z = marker[2][2]
                marker_array.markers.append(marker_msg)
            
            self.marker_publisher.publish(marker_array)
            
            # 거리 정보 시각화
            distance_text = f"Distance: {min_distance:.2f}m"
            cv2.putText(annotated_img, distance_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # YOLO 정보 처리 및 퍼블리시
        centers, _ = self.convert_box_center_2_mm(results, (w, h))
        self.publish_info(json.dumps(centers))

        # 시각화 이미지 퍼블리시
        annotated_img = cv2.resize(annotated_img, (320, 240))
        self.publish_img(annotated_img)

    def publish_info(self, str_):
        msg = String()
        msg.data = str_
        self.info_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()