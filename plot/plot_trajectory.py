import numpy as np
import matplotlib.pyplot as plt


def get_xyzuvctt(T):
    t = T[:3, 3:4].flatten()
    t_ = t / (np.linalg.norm(t) + 1e-5) * 5
    R = T[:3, :3]
    (x, y, z), (u, v, c) = np.repeat(t.reshape(3, 1), 3, axis=1), R
    return x, y, z, u, v, c, t, t_


def plot_pose(param, ax, poses, show_coord=True, show_pose=True):

    lngth = 3
    ax.view_init(elev=22, azim=32)
    ax.view_init(elev=22, azim=20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis('on')

    # origin and coordinate axes (coordinate frame)
    xl = 'x'
    yl = 'y'
    zl = 'z'
    ol = 'O'
    x, y, z, u, v, c, t, t_ = get_xyzuvctt(np.eye(4))

    ax.scatter(t_[0], t_[1], t_[2], s=80, c='k')
    ax.quiver(x, y, z, u, v, c, color='k', length=lngth, arrow_length_ratio=0.2, linewidth=0.5)
    ax.text(0, -0.5, -0.1, ol)
    ax.text(lngth * 1.2, 0, 0, xl)
    ax.text(0, lngth * 1.2, 0, yl)
    ax.text(0, 0, lngth * 1.2, zl)
    # aid to have initial minimum dimensions of plot
    ax.scatter(4, 4, 4, alpha=0)

    prev_t = [0, 0, 0]
    for i, pose in enumerate(poses):
        # given frame
        x, y, z, u, v, c, t, t_ = get_xyzuvctt(pose)
        if np.linalg.norm(prev_t - t) < param:
            ax.plot([prev_t[0], t[0]], [prev_t[1], t[1]], [prev_t[2], t[2]], color='k', linewidth=2)
        prev_t = t

    if show_pose:
        ax.quiver(x, y, z, u, v, c, arrow_length_ratio=0, color=['r', 'g', 'b'], linestyle='-', linewidth=4)
    if show_coord:
        ax.text(t[0], t[1], t[2] - 0.7, '(' + str(t[0]) + ',' + str(t[1]) + ',' + str(t[2]) + ')')


def plot_trajectory(transformation_matrix_set, ax, trajectories=None):
    if trajectories is not None:
        param = 2
        for trajectory in trajectories:
            plt.cla()
            plt.title('Trajectory based on data from T265')
            plot_pose(poses=transformation_matrix_set, ax=ax, param=param)
            plt.draw()
            plt.pause(0.0001)
