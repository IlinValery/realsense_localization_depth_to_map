# TODO: import pip libs here
# TODO: next 2 lines with Stepan
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import sys
import cv2
import matplotlib.pyplot as plt
import time
import numpy as np
import pyrealsense2 as rs

# TODO: import project classes & functions here
from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor
from plot.plot_trajectory import plot_trajectory
from plot.plot_points import PointCloudVisualizer


def print_timestamps(delay=None, **kwargs):
    """
    Print timestamps of sensors in console. Print difference if more then 2
    :param delay: delay in seconds
    :param kwargs: should be in format SensorName1=timestamp1, SensorName2=timestamp2
    :return: None
    """
    timestamp = []
    for kw in kwargs.keys():
        print("{0} sensor timestamp: {1}".format(kw, kwargs[kw]))
        if type(kwargs[kw]) == float:
            timestamp.append(kwargs[kw])
    if len(timestamp) == 2:
        print("Difference between timestamps is [1]-[0]: {}".format(timestamp[1] - timestamp[0]))
    print('')
    if delay is not None:
        time.sleep(delay)


# TODO: initials constants
is_device = False
is_write_to_bag = False
show_plot_trajectory = True
show_points = False
tm_T265toD435 = np.load('config/T265toD435.npy')

if __name__ == "__main__":

    # TODO: initial variables
    pose_number = 0
    transformation_matrix_set = []

    # TODO: classes initialization + initial conditions for them
    if is_device:
        D435 = D435Sensor(is_device=is_device, source_name='845112070910')
        T265 = T265Sensor(is_device=is_device, source_name=None)  # source device ID is 0000943222110531
        # TODO: find a way to specify T265 serial number, now it is: '' or None
        # https://github.com/IntelRealSense/librealsense/issues/3434
        # https://github.com/IntelRealSense/librealsense/issues/5614
        if is_write_to_bag:
            D435.allow_writing_to_file('D435.bag')
            T265.allow_writing_to_file('T265.bag')
    else:
        D435 = D435Sensor(is_device=is_device, source_name='data/D435.bag')
        T265 = T265Sensor(is_device=is_device, source_name='data/T265.bag')

    D435.attach(T265)  # subscribe T265 on D435 updates

    point_viewer = PointCloudVisualizer(update_each_frames=30)
    if show_points:
        D435.attach(point_viewer)

    D435.start_sensor()
    if not is_device:
        time.sleep(0.250 + 0.02)  # TODO: include difference here (computers are different)
    T265.start_sensor()

    # TODO: refactor this
    fig = None
    ax = None
    if show_plot_trajectory:
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        plt.ion()

    try:
        while True:
            # TODO: all manipulations with data here

            depth_frame = D435.get_depth_frame()

            pose265 = T265.get_pose()

            if (depth_frame is not None) and (pose265 is not None):
                print_timestamps(D435=depth_frame.get_timestamp(), T265=pose265.get_timestamp())

                # TODO: extract grayscale image here and show images
                # gray_image, depth_image = D435.get_images()
                # depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
                # cv2.imshow('D435 Depth Frame', depth_image)
                # cv2.imshow('D435 RGB Frame', grey_image)
                # cv2.waitKey(33)

                transformation_matrix = T265.get_transformation()
                # print('transformation_matrix', transformation_matrix)

                # TODO: get transformation mask from D436
                # tr_mx = D435.get_transformation(init_guess=transformation_matrix)
                # print('transformation_matrix435', tr_mx)
                # D435.update_trajectory(max_point_pair_dist=5.0)
                # print(D435.pose)

                # TODO: Transformation points and append to existing (association)
                if show_points:
                    pc = rs.pointcloud()
                    points = pc.calculate(depth_frame).as_points()
                    coordinates = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(480, 848, 3)) \
                        .reshape((-1, 3))
                    point_viewer.set_points(coordinates)

                if show_plot_trajectory:
                    # TODO: save last N elements of trajectory to
                    # TODO: KeyboardInterrupt on plot
                    # print("TM: ", tm_T265toD435)
                    tr_mx_from_T265 = T265.get_transformation() @ tm_T265toD435
                    tr_mx_from_D435 = D435.get_transformation(init_guess=tr_mx_from_T265)
                    print(tr_mx_from_D435)
                    if tr_mx_from_D435 is not None:
                        transformation_matrix_set.append(tr_mx_from_D435)
                        plot_trajectory(transformation_matrix_set, ax, trajectories=[1])

    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()
        if show_plot_trajectory:
            plt.show(block=True)
    finally:
        D435.stop_sensor()
        T265.stop_sensor()
