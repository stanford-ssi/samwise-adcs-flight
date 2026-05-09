import numpy as np
import matplotlib.pyplot as plt

data = np.load('vectors.npy')  # shape: (N, 3)

mask = (
    (data[:, 0] > -1000) & (data[:, 0] < 1000) &
    (data[:, 1] > -1000) & (data[:, 1] < 1000) &
    (data[:, 2] > -1000) & (data[:, 2] < 1000)
)
data = data[mask]

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_aspect('equal');
plt.tight_layout()
plt.show()
