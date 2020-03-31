import numpy as np
import matplotlib.pyplot as plt


def get_xyzuvctt(T):
    t = T[:3, 3:4].flatten()
    t_ = t / (np.linalg.norm(t) + 1e-5) * 5
    R = T[:3, :3]
    (x, y, z), (u, v, c) = np.repeat(t.reshape(3, 1), 3, axis=1), R
    return x, y, z, u, v, c, t, t_


def plot_pose(param, ax, poses, show_coord=True, show_pose=True, name='1'):

    lngth = 2
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

    ax.scatter(t_[0], t_[1], t_[2], s=40, c='k')
    ax.quiver(x, y, z, u, v, c, color='k', length=lngth, arrow_length_ratio=0.02, linewidth=0.5)
    ax.text(0, -0.5, -0.1, ol)
    ax.text(lngth * 1.2, 0, 0, xl)
    ax.text(0, lngth * 1.2, 0, yl)
    ax.text(0, 0, lngth * 1.2, zl)
    # aid to have initial minimum dimensions of plot
    ax.scatter(1.5, 1.5, 1.5, alpha=0)
    ax.scatter(-1, -1, -1, alpha=0)

    coords = np.array(poses)[:,:3,3]
    ax.plot(coords[:,0], coords[:,1], coords[:,2], linewidth=1, label=name)
    if show_pose:
        x, y, z, u, v, c, t, t_ = get_xyzuvctt(poses[-1])
        ax.quiver(x, y, z, u, v, c, arrow_length_ratio=0, color=['r', 'g', 'b'], linestyle='-', linewidth=3)
    if show_coord:
        ax.text(t[0], t[1], t[2] - 0.7, '(' + str(t[0]) + ',' + str(t[1]) + ',' + str(t[2]) + ')')




def get_title(names):
    baseline = 'Trajectory based on data from '
    for name in names:
        baseline += str(name) + ' '
    return baseline


def plot_trajectory(ax=None, **kwargs):
    if ax is not None:
        if len(kwargs) > 0:
            param = 2
            plt.cla()
            plt.title(get_title(list(kwargs.keys())))
            for kwarg in kwargs.keys():
                transformation_matrix_set = kwargs[kwarg]
                plot_pose(poses=transformation_matrix_set, ax=ax, param=param, name=kwarg)
            plt.legend()
            plt.draw()
            plt.pause(0.0001)
