import threading
import time
import glooey
import numpy as np
import octomap
import open3d as o3d
from helpers.custom_threading import StoppableThread
import pyglet
import trimesh
import trimesh.viewer
import trimesh.transformations as tf
from functools import wraps


def timing(f):
    @wraps(f)
    def wrap(*args, **kw):
        time_start = time.time()
        result = f(*args, **kw)
        time_end = time.time()
        print('----------func:%r took: %2.4f sec' % (f.__name__, time_end-time_start))
        return result
    return wrap

class Application:
    def __init__(self, resolution=0.1):
        # create window
        self.resolution = resolution

        self.width, self.height = 700, 700
        window = self._create_window(width=self.width, height=self.height)
        gui = glooey.Gui(window)
        hbox = glooey.HBox()

        # scene widget for changing
        self.geometry_name = 'geometry_0'
        scene = trimesh.Scene()
        # initial cube
        geom = trimesh.path.creation.box_outline((1, 1, 1))
        scene.add_geometry(geom)

        self.scene_widget = trimesh.viewer.SceneWidget(scene)
        hbox.add(self.scene_widget)
        # clear from cube ¯\_(ツ)_/¯
        self.scene_widget.scene.delete_geometry(self.geometry_name)
        gui.add(hbox)

        # buffer here (points and all for processing
        self.points = np.random.uniform(-0.3, 0.3, (100, 3))
        self.render_points = self.points
        self.is_changed = True
        # self.is_updated = False
        pcd = o3d.io.read_point_cloud("notebooks/pcl_comb_opt.pcd")
        self.all_points = np.asarray(pcd.points)
        self.batch = self.all_points.shape[0]/1000
        self.current_batch = 1

        # launch updater and main thread
        pyglet.clock.schedule_interval(self._update_visualisation_callback, 1. / 60)
        self.main_thread = StoppableThread(target=self._start_processing)
        self.main_thread.start()
        pyglet.app.run()

    def _start_processing(self):
        """
        Like main.py, the same loop, but as different thread :)
        :return:
        """

        while threading.currentThread().is_execute():
            time.sleep(0.5)
            self.points = self.all_points[:self.current_batch]
            self.current_batch += int(self.batch)
            if self.current_batch > self.all_points.shape[0]:
                self.current_batch = self.all_points.shape[0]
            self.render_points = self.get_sorted_points(self.get_occupied_points(self.points))
            self.is_changed = True

    def _stop_processing(self):
        self.main_thread.stop()

    @staticmethod
    def get_colors(points):
        size = points.shape[0]
        blue = np.linspace(1., 0., size // 2)
        blue1 = np.zeros(size - (size // 2))
        blue = np.hstack((blue, blue1))

        green = np.linspace(0., 1., size // 2)
        green1 = np.linspace(1., 0., size - (size // 2))
        green = np.hstack((green, green1))

        red = np.zeros(size // 2)
        red1 = np.linspace(0., 1., size - (size // 2))
        red = np.hstack((red, red1))

        trans = 1.0 * np.ones(size)
        return np.vstack((red, green, blue, trans)).T

    @staticmethod
    def get_sorted_points(points):
        return points[points[:, 1].argsort()][::-1]

    # @timing
    def get_occupied_points(self, points):
        octree = octomap.OcTree(self.resolution)
        octree.insertPointCloud(
            pointcloud=np.double(points),
            origin=np.array([0, 0, 0], dtype=float),  # TODO this
        )
        occupied, _ = octree.extractPointCloud()

        return occupied

    # @timing
    def _update_visualisation_callback(self, dt):
        """
        Update all points in each iteration
        :param dt: interval, called in pyglet.clock.schedule_interval
        :return:
        """
        if self.is_changed:
            self.is_changed = False
            points = self.render_points  # from new points
            colors = self.get_colors(points)
            geom = trimesh.voxel.ops.multibox(
                points, pitch=self.resolution,
                colors=colors
            )

            if self.geometry_name is not None:
                self.scene_widget.scene.delete_geometry(self.geometry_name)
            self.scene_widget.scene.add_geometry(geom)
            self.scene_widget._draw()

    def _create_window(self, width, height):
        try:
            config = pyglet.gl.Config(sample_buffers=1,
                                      samples=4,
                                      depth_size=24,
                                      double_buffer=True)
            window = pyglet.window.Window(config=config,
                                          width=width,
                                          height=height)
        except pyglet.window.NoSuchConfigException:
            config = pyglet.gl.Config(double_buffer=True)
            window = pyglet.window.Window(config=config,
                                          width=width,
                                          height=height)

        def stop():
            self._stop_processing()

        @window.event
        def on_key_press(symbol, modifiers):
            if modifiers == 0:
                if symbol == pyglet.window.key.Q:
                    window.close()

        @window.event
        def on_close():
            stop()

        return window


if __name__ == '__main__':
    print("Scroll mouse down to zoom out")
    Application()
