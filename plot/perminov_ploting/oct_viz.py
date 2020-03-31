import numpy as np

import octomap
from oct_viz_tools import visualize
import open3d as o3d

is_pyglet_used = False
pcd = o3d.io.read_point_cloud("pcl_comb_opt.pcd")
coordinates = np.asarray(pcd.points)
if __name__ == "__main__":
    resolution = 0.08
    octree = octomap.OcTree(resolution)
    try:
        octree.insertPointCloud(np.double(coordinates), np.array([0, 1., 0]))
        occupied, empty = octree.extractPointCloud()

        visualize(
            occupied=occupied,
            resolution=resolution,
            is_pyglet_used=is_pyglet_used
        )

    except KeyboardInterrupt:
        pass
    finally:
        pass
