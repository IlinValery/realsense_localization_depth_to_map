import time
import numpy as np
import glooey
import octomap
import pyglet  # http://pyglet.org/
import trimesh
import trimesh.transformations as tf
import trimesh.viewer


from helpers.custom_threading import StoppableThread


class OctoMapVisualiser:
    def __init__(self, update_time=1 / 30, resolution=0.01):
        self.update_time = update_time
        self.resolution = resolution
        self.octree = octomap.OcTree(resolution)

        self.point_cloud = None
        self.is_changed = False
        self.window = pyglet.window.Window(width=1024, height=720)

        @self.window.event
        def on_key_press(symbol, modifiers):
            if modifiers == 0:
                if symbol == pyglet.window.key.Q:
                    self.window.on_close()

        # self.gui = None

        aabb_min = np.array([-1.4, -1., 0.])
        aabb_max = np.array([0.45, 0.75, 2.])
        initial_bbox = trimesh.path.creation.box_outline(
            aabb_max - aabb_min,
            tf.translation_matrix((aabb_min + aabb_max) / 2),
        )
        self.scene = trimesh.Scene(geometry=[initial_bbox]) # initial_bbox
        print(self.scene)
        # self.scene = None
        self.app_visualiser = None
        self.start_visualiser()

    def __del__(self):
        print("Deleting octomap")
        pyglet.clock.unschedule(func=self.update_visualisation)
        self.app_visualiser.stop()
        self.window.close()

    def update_visualisation(self, dt):
        if self.is_changed:
            if self.point_cloud is not None:
                self.octree.insertPointCloud(
                    pointcloud=np.double(self.point_cloud),
                    origin=np.array([0, 0, 0], dtype=float),  # TODO this
                    maxrange=2,
                )

                occupied, _ = self.octree.extractPointCloud()
                aabb_min = self.octree.getMetricMin()
                aabb_max = self.octree.getMetricMax()

                bbox = trimesh.path.creation.box_outline(
                    aabb_max - aabb_min,
                    tf.translation_matrix((aabb_min + aabb_max) / 2),
                )
                geom = trimesh.voxel.ops.multibox(
                    occupied, pitch=self.resolution, colors=[1., 0, 0, 0.5]
                )
                self.scene = trimesh.Scene(geometry=[bbox, geom])

            #                 self.update_hbox(occupied, aabb_min, aabb_max)

            self.is_changed = False
        else:
            print("Didn't changed")
            pass

    def update_points(self, new_pc):
        self.point_cloud = new_pc
        self.is_changed = True

    def init_visualiser(self):
        gui = glooey.Gui(self.window)
        # pyglet.clock.schedule_interval(func=self.update_visualisation, interval=self.update_time)


        aabb_min = np.array([-1.4, -1., 0.])
        aabb_max = np.array([0.45, 0.75, 2.])
        initial_bbox = trimesh.path.creation.box_outline(
            aabb_max - aabb_min,
            tf.translation_matrix((aabb_min + aabb_max) / 2),
        )
        scene = trimesh.Scene(geometry=[initial_bbox]) # initial_bbox


        gui.add(trimesh.viewer.SceneWidget(scene))
        pyglet.app.run()

    def start_visualiser(self):
        self.app_visualiser = StoppableThread(target=self.init_visualiser)
        self.app_visualiser.start()

    def stop_visualiser(self):
        self.__del__()

