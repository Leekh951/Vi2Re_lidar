#!/usr/bin/env python3

import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import time
import rospy
import pandas as pd
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
import hdbscan
import pandas as pd
from std_msgs.msg import Header

def distance_from_origin(bbox_object, origin=[0.0, 0.0, 0.0]):
    x = (bbox_object.min_bound[0] + bbox_object.max_bound[0]) / 2
    y = (bbox_object.min_bound[1] + bbox_object.max_bound[1]) / 2
    z = (bbox_object.min_bound[2] + bbox_object.max_bound[2]) / 2
    
    return np.sqrt((x - origin[0])**2 + (y - origin[1])**2 + (z - origin[2])**2)

def pcd_to_pointcloud2(pcd):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "laser"  # 적절한 frame_id를 설정해주세요
    points = np.asarray(pcd.points)
    return pc2.create_cloud_xyz32(header, points)   

last_marker_ids = set()  # 마지막으로 사용한 마커 ID를 추적하기 위한 변수

def callback(data):
    t3 = time.time()
    global last_marker_ids  # 전역 변수를 사용
    
    points = np.array(list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    # downsampling
    pcd_1 = pcd.voxel_down_sample(voxel_size=0.1)
    # remove outliers
    pcd_2, inliers = pcd_1.remove_radius_outlier(nb_points=17, radius=0.5)
    # segment plane with RANSAC
    plane_model, road_inliers = pcd_2.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=100)
    pcd_3 = pcd_2.select_by_index(road_inliers, invert=True)
    # 기준 프레임 1미터 이내의 포인트 제거
    points = np.asarray(pcd_3.points)
    distances = np.linalg.norm(points, axis=1)
    filtered_indices = np.where(distances >= 1.7)[0]
    pcd_3 = pcd_3.select_by_index(filtered_indices)
    #HDBscan 클러스터링
    clusterer = hdbscan.HDBSCAN(min_cluster_size=10, gen_min_span_tree=True)
    clusterer.fit(np.array(pcd_3.points))
    labels = clusterer.labels_

    max_label = labels.max()
    print(f'point cloud has {max_label + 1} clusters')
    colors = plt.get_cmap("tab20")(labels / max_label if max_label > 0 else 1)
    negative_indices = np.where(labels < 0)[0]
    colors[negative_indices] = 0
    pcd_3.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # generate 3D Bounding Box

    bbox_objects = []
    indexes = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()

    MAX_POINTS = 1200
    MIN_POINTS = 5
    MAX_DIMENSION_DIFF = 5.0 
    
    for i in range(0, len(indexes)):
        nb_points = len(pcd_3.select_by_index(indexes[i]).points)
        if (nb_points > MIN_POINTS and nb_points < MAX_POINTS):
            sub_cloud = pcd_3.select_by_index(indexes[i])
            bbox_object = sub_cloud.get_axis_aligned_bounding_box()
            
            # min_bound와 max_bound의 차이가 n미터를 넘지 않는지 확인
            dimension_diff = np.abs(bbox_object.max_bound - bbox_object.min_bound)
            if np.any(dimension_diff > MAX_DIMENSION_DIFF):
                continue
            
            # # 여기에서 거리 계산
            # if distance_from_origin(bbox_object) >= 1.7:  #1.7m
                # bbox_object.color = (0, 0, 1)
                # bbox_objects.append(bbox_object)
            bbox_object.color = (0, 0, 1)
            bbox_objects.append(bbox_object)

    print("Number of Boundinb Box : ", len(bbox_objects))
    marker_array = MarkerArray()
    if len(bbox_objects) == 0:  # 객체가 없는 경우 마커 삭제
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
    else: 
        current_marker_ids = set()  # 현재 프레임에서 사용할 마커 ID를 저장하기 위한 변수  
        for i, bbox_object in enumerate(bbox_objects):
            current_marker_ids.add(i)
            marker = Marker()
            marker.header.frame_id = "laser"  # 적절한 frame_id를 설정
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = (bbox_object.min_bound[0] + bbox_object.max_bound[0]) / 2
            marker.pose.position.y = (bbox_object.min_bound[1] + bbox_object.max_bound[1]) / 2
            marker.pose.position.z = (bbox_object.min_bound[2] + bbox_object.max_bound[2]) / 2
            
            marker.scale.x = abs(bbox_object.max_bound[0] - bbox_object.min_bound[0])
            marker.scale.y = abs(bbox_object.max_bound[1] - bbox_object.min_bound[1])
            marker.scale.z = abs(bbox_object.max_bound[2] - bbox_object.min_bound[2])
            
            marker.color.a = 0.5  # 투명도
            marker.color.r = 0.0  # 빨강
            marker.color.g = 0.0  # 초록
            marker.color.b = 1.0  # 파랑
            marker_array.markers.append(marker)
        # 이전 프레임에서 사용했지만 현재 프레임에서 사용되지 않는 마커 ID를 찾는다
        to_delete_marker_ids = last_marker_ids - current_marker_ids
        
        for delete_id in to_delete_marker_ids:
            marker = Marker()
            marker.header.frame_id = "laser"  # 적절한 frame_id를 설정
            marker.id = delete_id
            marker.action = Marker.DELETE  # 마커 삭제
            marker_array.markers.append(marker)    
            
    
    marker_array_pub.publish(marker_array)
    last_marker_ids = current_marker_ids  # 마지막으로 사용한 마커 ID를 업데이트
    pointcloud2_msg = pcd_to_pointcloud2(pcd_3)
    pointcloud_pub.publish(pointcloud2_msg) 
    t4 = time.time()
    print(f'Time to cluster outliers using HDBSCAN {t4 - t3}')
    # list_of_visuals = []
    # list_of_visuals.append(pcd_3)
    # list_of_visuals.extend(bbox_objects)
if __name__ == '__main__':
    rospy.init_node('pointcloud_processing_node', anonymous=True)
    # rospy.Subscriber("/lidar3D", PointCloud2, callback)
    rospy.Subscriber("/lidar3D", PointCloud2, callback)
    pointcloud_pub = rospy.Publisher("/processed_pointcloud", PointCloud2, queue_size=10)  # 새로운 퍼블리셔
    marker_array_pub = rospy.Publisher("/bbox_marker_array", MarkerArray, queue_size=10)
    rospy.spin()




