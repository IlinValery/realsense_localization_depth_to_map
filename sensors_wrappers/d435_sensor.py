import time
from functools import wraps

import numpy as np
from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseSubject, BaseObserver
from typing import List
import pyrealsense2 as rs
import open3d as o3d


def timing(f):
    @wraps(f)
    def wrap(*args, **kw):
        time_start = time.time()
        result = f(*args, **kw)
        time_end = time.time()
        print('----------func:%r took: %2.4f sec' % (f.__name__, time_end-time_start))
        return result
    return wrap


class D435Sensor(BaseSensor, BaseSubject):
    def __init__(self, is_device, source_name):

        # Initialization of D435 sensor
        super(D435Sensor, self).__init__(is_device, source_name)
        self.cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        # TODO: for monochrome
        # self.cfg.enable_stream(rs.stream.infrared, 848, 480, rs.format.y8, 30) # uncomment if have color

        self.koef_sampling = 2 ** 2
        self.tm_T265toD435 = np.load('configs/T265toD435.npy')

        print(self.tm_T265toD435)
        # initialize observers
        self._observers: List[BaseObserver] = [] # pattern observer in common

        # TODO: insert initial conditions here:
        self.frameset = None
        self.gray_frame = None
        self.depth_frame = None
        self.color_image = None
        self.depth_image = None
        
        #for trajectory

        self.point_cloud = None
        self.prev_tm = None
        self.pose = np.eye(3)
        self.trajectory = [np.zeros(3)]

    def attach(self, observer: BaseObserver) -> None:
        self._observers.append(observer)

    def detach(self, observer: BaseObserver) -> None:
        self._observers.remove(observer)

    def notify(self) -> None:
        for observer in self._observers:
            observer.on_parent_update(self)

    def do_sensor_update(self):
        self.frameset = self.pipe.wait_for_frames()
        self.process_frameset()
        self.notify()

    def process_frameset(self):
        # TODO extract necessary data here to D435 sensor object:
        self.depth_frame = self.frameset.get_depth_frame()
        # self.gray_frame = self.frameset.get_infrared_frame()

    def get_frameset(self):
        return self.frameset

    # TODO: get functions here:
    def get_depth_frame(self):
        return self.depth_frame

# <<<<<<< zainulina-develop
#     def get_frames(self):
#         return self.color_frame, self.depth_frame
    
    def get_geom_pcl(self):
        pc = rs.pointcloud()
#         pc.map_to(self.color_frame)
        pcl = pc.calculate(self.depth_frame)
        pcl.export_to_ply('tmp.ply', self.color_frame)
        pcl = o3d.io.read_point_cloud("tmp.ply")
        os.remove('tmp.ply')
        return pcl

    def get_depth_image(self):
        return np.asanyarray(self.depth_frame.get_data())
        
    
    # def get_geom_pcl(self):
    #     pc = rs.pointcloud()
    #     pcl1 = pc.calculate(self.depth_frame)
    #     pcl1.export_to_ply('tmp.ply')
    #     pcl = o3d.io.read_point_cloud("tmp.ply")
    #     os.remove('tmp.ply')
    #     return pcl

    @timing
    def get_coordinates(self, make_sampling=True):
        """
         TODO
        :param make_sampling:
        :return:
        """
        pc = rs.pointcloud()
        if make_sampling:
            decimate = rs.decimation_filter()
            decimate.set_option(rs.option.filter_magnitude, self.koef_sampling)
            depth_frame = decimate.process(self.get_depth_frame())

            points = pc.calculate(depth_frame).as_points()
        else:
            points = pc.calculate(self.get_depth_frame()).as_points()

        coordinates = np.ndarray(buffer=points.get_vertices(), dtype=np.float32, shape=(points.size(), 3))
        coordinates = coordinates[coordinates[:, 2] != 0]

        coordinates = self.apply_transformation(self.tm_T265toD435, coordinates)
        return coordinates

    @staticmethod
    def apply_transformation(transformation, points):
        """

        :param transformation:
        :param points:
        :return:
        """
        if transformation is None or points is None:
            return None
        else:
            coordinates = np.hstack((points, np.ones((points.shape[0], 1))))
            return (transformation @ coordinates.T).T[:, :-1]

    @staticmethod
    def convert_to_pcl(points):
        """
            TODO
        :param points:
        :return:
        """
        if points is None:
            return None
        else:
            return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    @timing
    def get_transformation(self, max_point_pair_dist=0.2, init_guess=np.eye(4)):
        old_point_cloud = self.apply_transformation(self.prev_tm, self.point_cloud)
        old_pcl = self.convert_to_pcl(old_point_cloud) # n-1
        self.point_cloud = self.apply_transformation(self.prev_tm, self.get_coordinates())
        if old_pcl is None:
            self.prev_tm = init_guess
            print("\n\n\n{}\n\n\n".format(init_guess))
            return None

        new_pcl = self.convert_to_pcl(self.point_cloud) # n

        tr_mx = o3d.registration.registration_icp(old_pcl, new_pcl, max_point_pair_dist,
                                                  self.prev_tm, o3d.registration.TransformationEstimationPointToPoint())\
            .transformation
        self.prev_tm = tr_mx
        return tr_mx

    def update_trajectory(self, max_point_pair_dist=10.0, init_guess=np.eye(4)):
        tr_mx = self.get_transformation(max_point_pair_dist, init_guess)
        if tr_mx is None:
            return
        t_est = tr_mx[:3, -1]
        R_est = tr_mx[:3, :3]
        R = R_est @ self.pose
        t = self.trajectory[-1] + (np.linalg.inv(R) @ t_est).ravel()
                    
        self.pose = R
        self.trajectory.append(t)





