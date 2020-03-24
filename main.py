#!/usr/bin/env python3.7

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor
from plot.plot_trajectory import plot_trajectory
import matplotlib.pyplot as plt

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

    D435 = D435Sensor(is_device=False, source_name='data/435.bag')
    T265 = T265Sensor(is_device=False, source_name='data/265.bag')
    D435.attach(T265)

    T265.start_sensor()
    D435.start_sensor()

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
            color_frame, depth_frame = D435.get_frames()
            pose265 = T265.get_pose()
            if (color_frame is not None) and (pose265 is not None):
                # print('\nframeset435 gtab time', color_frame.get_timestamp())
                print('pose265 grab time    ', pose265.get_timestamp())

                transformation_matrix = T265.get_transformation()
                print('transformation_matrix',transformation_matrix)

                color_image, depth_image = D435.get_images()
                # cv2.imshow('D435 RGB Frame', color_image)
                depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
                cv2.imshow('D435 Depth Frame', depth_image)
                cv2.waitKey(1)

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
