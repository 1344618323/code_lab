import numpy as np
a = np.array([[1,2],[3,4]])
print(np.mat(a).I)
ainv = np.mat(a).I
print(ainv @ a)