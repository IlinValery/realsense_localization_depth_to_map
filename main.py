#!/usr/bin/env python3.7

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
# <<<<<<< ilin-develop
# import numpy as np
# import pyrealsense2 as rs
# from plot.plot_points import PointCloudVisualizer
# =======
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
# >>>>>>> master
from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor
from plot.plot_trajectory import plot_trajectory
import matplotlib.pyplot as plt

import time

is_device = False
is_write_to_bag = False
pose_number = 0
show_plot_trajectory = True

if __name__ == "__main__":
    if is_device:
        D435 = D435Sensor(is_device=True, source_name='845112070910')
        T265 = T265Sensor(is_device=True, source_name=None)  # source device ID is 0000943222110531
        # TODO: find a way to specify T265 serial number, now it is: '' or None
        # https://github.com/IntelRealSense/librealsense/issues/3434
        # https://github.com/IntelRealSense/librealsense/issues/5614
        if is_write_to_bag:
            D435.allow_writing_to_file("D435.bag")
            T265.allow_writing_to_file("T265.bag")
    else:
        D435 = D435Sensor(is_device=False, source_name='data/D435.bag')
        T265 = T265Sensor(is_device=False, source_name='data/T265.bag')

    D435.attach(T265)

# <<<<<<< ilin-develop
    point_viewer = PointCloudVisualizer(update_each_frames=30)
    D435.attach(point_viewer)

    T265.start_sensor()
# =======

# >>>>>>> master
    D435.start_sensor()
    if not is_device:
        time.sleep(0.25)  # give some time for 435.bag to startup
    T265.start_sensor()

    transformation_matrix_set = []

    fig = None
    ax = None
    if show_plot_trajectory:
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        plt.ion()



    try:
        while True:
            # TODO: all manupulations with data here
# <<<<<<< ilin-develop
#             # color_frame, depth_frame = D435.get_frames()
#             depth_frame = D435.get_depth_frame()
#             pose265 = T265.get_pose()

#             if (depth_frame is not None) and (pose265 is not None):
#                 # print('\nframeset435 gtab time', color_frame.get_timestamp())
#                 # print('pose265 grab time', pose265.get_timestamp())

#                 pc = rs.pointcloud()
#                 points = pc.calculate(depth_frame).as_points()
#                 coordinates = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(480, 848, 3)).reshape((-1,3))

#                 # coordinates = coo
#                 # vtx = np.asanyarray(points.get_vertices())
# =======
#             depth_frame = D435.get_frame()
#             pose265 = T265.get_pose()
# # <<<<<<< perminov-develop
# #             if (color_frame is not None) and (pose265 is not None):
#                 # print('\nframeset435 gtab time', color_frame.get_timestamp())
# # =======
#             if (depth_frame is not None) and (pose265 is not None):
#                 print('\nframeset435 gtab time', depth_frame.get_timestamp())
# # >>>>>>> master
#                 print('pose265 grab time    ', pose265.get_timestamp())
# >>>>>>> master

# <<<<<<< zainulina-develop
# #                 transformation_matrix = T265.get_transformation()
# #                 print('transformation_matrix',transformation_matrix)
                
# #                 tr_mx = D435.get_transformation(init_guess=transformation_matrix)
# #                 print('transformation_matrix435',tr_mx)

# #                 D435.update_trajectory(max_point_pair_dist=5.0)
# #                 print(D435.pose)
# =======
#                 transformation_matrix = T265.get_transformation()
#                 print('transformation_matrix',transformation_matrix)
# >>>>>>> master

# <<<<<<< ilin-develop
#                 point_viewer.set_points(coordinates)
#                 color_image, depth_image = D435.get_images()
#                 cv2.imshow('D435 RGB Frame', color_image)
# =======
# # <<<<<<< perminov-develop
# #                 color_image, depth_image = D435.get_images()
# # =======
#                 depth_image = D435.get_image()
# # >>>>>>> master
#                 # cv2.imshow('D435 RGB Frame', color_image)
# >>>>>>> master
                depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
                cv2.imshow('D435 Depth Frame', depth_image)
                cv2.waitKey(33)



            if show_plot_trajectory:
                transformation_matrix_set.append(T265.get_transformation())
                plot_trajectory(transformation_matrix_set, ax, trajectories=[1])

    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()
        if show_plot_trajectory:
            plt.show(block=True)
    finally:
        D435.stop_sensor()
        T265.stop_sensor()
