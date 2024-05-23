import numpy as np
T_b_cl = np.array([
    -2.7737148907138348e-03, -1.3001858713849823e-01,
       9.9150767697694353e-01, 5.1100000000000003e+00,
       -9.9998417017842200e-01, 5.2145925085985093e-03,
       -2.1136266329473460e-03, 6.0000140987774808e-01,
       -4.8954977560285743e-03, -9.9149784418498843e-01,
       -1.3003099276035271e-01, 1.9815665645599365e+00, 0., 0., 0., 1.
])

T_b_cl = T_b_cl.reshape((4,4))
print(T_b_cl)
T_cl_b = np.linalg.inv(T_b_cl)
print(T_cl_b)
print(T_cl_b @ T_b_cl)