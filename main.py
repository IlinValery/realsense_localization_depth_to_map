import cv2
import numpy as np
import pyrealsense2 as rs
from plot.plot_points import PointCloudVisualizer
from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor

is_device = False
is_write_to_bag = False
pose_number = 0

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

    point_viewer = PointCloudVisualizer(update_each_frames=30)
    D435.attach(point_viewer)

    T265.start_sensor()
    D435.start_sensor()



    try:
        while True:
            # TODO: all manupulations with data here
            # color_frame, depth_frame = D435.get_frames()
            depth_frame = D435.get_depth_frame()
            pose265 = T265.get_pose()

            if (depth_frame is not None) and (pose265 is not None):
                # print('\nframeset435 gtab time', color_frame.get_timestamp())
                # print('pose265 grab time', pose265.get_timestamp())

                pc = rs.pointcloud()
                points = pc.calculate(depth_frame).as_points()
                coordinates = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(480, 848, 3)).reshape((-1,3))

                # coordinates = coo
                # vtx = np.asanyarray(points.get_vertices())

                # transformation_matrix = T265.get_transformation()
                # print('transformation_matrix',transformation_matrix)

                point_viewer.set_points(coordinates)
                color_image, depth_image = D435.get_images()
                cv2.imshow('D435 RGB Frame', color_image)
                depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
                cv2.imshow('D435 Depth Frame', depth_image)
                cv2.waitKey(1)

    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()
    finally:
        D435.stop_sensor()
        T265.stop_sensor()
