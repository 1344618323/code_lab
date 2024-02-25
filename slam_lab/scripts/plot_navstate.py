from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot


def rot_mat_2d(angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


def plot_cov2d_ellipse_pts(x, y, cov2d, chi2=5.99):
    # chi2=5.99 corresponds 95% confidence for a 2d gaussian
    # The covariance matrix (which is a real symmetric matrix) can be orthogonally diagonalized,
    # that is, a principal component analysis can be performed:
    # The eigenvector is the direction vector of the principal component;
    # The eigenvalue is the variance in the corresponding direction.
    # eig_val are not necessarily ordered
    eig_val, eig_vec = np.linalg.eig(cov2d)
    sorted_indexes = np.argsort(eig_val)[::-1]  # descending order
    eig_val = eig_val[sorted_indexes]
    eig_vec = eig_vec[:, sorted_indexes]

    width, height = np.sqrt(np.abs(chi2*eig_val))
    angle = np.arctan2(eig_vec[1, 0], eig_vec[0, 0])
    t = np.arange(0, 2*np.pi+0.5, 0.5)
    ellipse_pts = np.stack([width * np.cos(t), height * np.sin(t)]
                           ).T @ rot_mat_2d(angle)
    ptx = np.array(ellipse_pts[:, 0] + x).flatten()
    pty = np.array(ellipse_pts[:, 1] + y).flatten()
    plt.plot(ptx, pty, '--r')


# this lib must be imported, or can't use `projection='3d'`
assert (len(sys.argv) == 2)
path_data = np.loadtxt(sys.argv[1])
plt.figure(figsize=(16, 12))

plt.subplot(231)
plt.scatter(x=path_data[:, 1], y=path_data[:, 2], s=2)

# draw cov ellipse
# if path_data.shape[1] > 20:
#     for i in np.arange(0, 10000, 100):
#         cov = np.array([[path_data[i, 20], path_data[i, 21]],
#                        [path_data[i, 22], path_data[i, 23]]])
#         plot_cov2d_ellipse_pts(
#             path_data[i, 1], path_data[i, 2], cov)

plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid()
plt.title('2D path')

plt.subplot(234, projection='3d')
plt.plot(path_data[:, 1], path_data[:, 2], path_data[:, 3])
plt.title('3D path')

plt.subplot(232)
plt.plot(path_data[:, 0], path_data[:, 4], 'r')
plt.plot(path_data[:, 0], path_data[:, 5], 'g')
plt.plot(path_data[:, 0], path_data[:, 6], 'b')
plt.plot(path_data[:, 0], path_data[:, 7], 'k')
plt.legend(['qw', 'qx', 'qy', 'qz'])
plt.title('q')

plt.subplot(235)
plt.plot(path_data[:, 0], path_data[:, 8], 'r')
plt.plot(path_data[:, 0], path_data[:, 9], 'g')
plt.plot(path_data[:, 0], path_data[:, 10], 'b')
plt.legend(['vx', 'vy', 'vz'])
plt.title('v')

plt.subplot(333)
plt.plot(path_data[:, 0], path_data[:, 11], 'r')
plt.plot(path_data[:, 0], path_data[:, 12], 'g')
plt.plot(path_data[:, 0], path_data[:, 13], 'b')
plt.legend(['bgx', 'bgy', 'bgz'])
plt.title('bg')

plt.subplot(336)
plt.plot(path_data[:, 0], path_data[:, 14], 'r')
plt.plot(path_data[:, 0], path_data[:, 15], 'g')
plt.plot(path_data[:, 0], path_data[:, 16], 'b')
plt.legend(['bax', 'bay', 'baz'])
plt.title('ba')

plt.subplot(339)
plt.plot(path_data[:, 0], path_data[:, 17], 'r')
plt.plot(path_data[:, 0], path_data[:, 18], 'g')
plt.plot(path_data[:, 0], path_data[:, 19], 'b')
plt.legend(['gx', 'gy', 'gz'])
plt.title('g')

plt.show()
