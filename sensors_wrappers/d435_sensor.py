import time
import os

import numpy as np

from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseSubject, BaseObserver
from typing import List
import pyrealsense2 as rs
import open3d as o3d


class D435Sensor(BaseSensor, BaseSubject):
    def __init__(self, is_device, source_name):

        # Initialization of D435 sensor
        super(D435Sensor, self).__init__(is_device, source_name)
        self.cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        # self.cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        # initialize observers
        self._observers: List[BaseObserver] = [] # pattern observer in common

        # TODO: insert initial conditions here:
        self.frameset = None
        # self.color_frame = None
        self.depth_frame = None
        self.color_image = None
        self.depth_image = None
        # self.frameset_grab_time = None
        
        #for trajectory?
        self.pointcloud = None
        self.pose = np.eye(3)
        self.trajectory = [np.zeros(3)]

    def attach(self, observer: BaseObserver) -> None:
        self._observers.append(observer)

    def detach(self, observer: BaseObserver) -> None:
        self._observers.remove(observer)

    def notify(self) -> None:
        for observer in self._observers:
            observer.parent_update(self)

    def do_sensor_update(self):
        self.frameset = self.pipe.wait_for_frames()
        self.process_frameset()
        self.notify()

    def process_frameset(self):
        # TODO extract necessary data here:
        # self.color_frame = self.frameset.get_color_frame()
        self.depth_frame = self.frameset.get_depth_frame()
        # self.color_image = np.asanyarray(self.color_frame.get_data())
        self.depth_image = np.asanyarray(self.depth_frame.get_data())

    def get_frameset(self):
        return self.frameset

    # TODO: get functions here:
    def get_image(self):
        return self.depth_image

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
    
    def get_geom_pcl_v2(self): #slower
        pc = rs.pointcloud()
#         pc.map_to(self.color_frame)
        pcl = pc.calculate(self.depth_frame)
        vertices = np.asarray(pcl.get_vertices())
        vs = []
        for i in range(pcl.size()):
            if vertices[i][2]:
                vs.append([*vertices[i]])
        vs = np.array(vs)
        vs[:, 1] *= -1
        vs[:, 2] *= -1
        pcl = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(vs))
        return pcl
    
    def get_transformation(self, max_point_pair_dist=10.0, init_guess=np.eye(4)):
        
        if self.pointcloud is None:
            self.pointcloud = self.get_geom_pcl()
            return None
        old_pcl = self.pointcloud
        self.pointcloud = self.get_geom_pcl()
        
        return o3d.registration.registration_icp(old_pcl, self.pointcloud, max_point_pair_dist, 
                                                 init_guess).transformation
    
    def update_trajectory(self, max_point_pair_dist=10.0, init_guess=np.eye(4)):
                
        tr_mx = self.get_transformation(max_point_pair_dist, init_guess)
        if tr_mx is None:
            return
        t_est = tr_mx[:3, -1]
        R_est = tr_mx[:3,:3]
                    
        R = R_est @ self.pose
        t = self.trajectory[-1] + (np.linalg.inv(R) @ t_est).ravel()
                    
        self.pose = R
        self.trajectory.append(t)
# =======
    def get_frame(self):
        return self.depth_frame
# >>>>>>> master


