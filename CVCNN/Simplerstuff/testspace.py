from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import matplotlib.pyplot as plt

# Create figure and 3D axis
fig = plt.figure(figsize=(12, 5))

# RGB color space
ax1 = fig.add_subplot(121, projection='3d')

# RGB basis vectors
rgb_vectors = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
colors_rgb = ['red', 'green', 'blue']
labels_rgb = ['R', 'G', 'B']

# Plot RGB vectors
for i, (vector, color, label) in enumerate(zip(rgb_vectors, colors_rgb, labels_rgb)):
    ax1.quiver(0, 0, 0, vector[0], vector[1], vector[2], 
               color=color, arrow_length_ratio=0.1, linewidth=2, label=label)

ax1.set_xlim([0, 1.2])
ax1.set_ylim([0, 1.2])
ax1.set_zlim([0, 1.2])
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('RGB Color Space')
ax1.legend()
ax1.grid(True)

# YUV color space (approximate transformation basis)
ax2 = fig.add_subplot(122, projection='3d')

# YUV basis vectors (from RGB to YUV transformation)
yuv_vectors = np.array([[0.299, 0.587, 0.114],      # Y vector (luminance)
                        [-0.14713, -0.28886, 0.436],  # U vector
                        [0.615, -0.51499, -0.10001]])  # V vector
colors_yuv = ['gray', 'cyan', 'magenta']
labels_yuv = ['Y', 'U', 'V']


# Plot YUV vectors
for i, (vector, color, label) in enumerate(zip(yuv_vectors, colors_yuv, labels_yuv)):
    ax2.quiver(0, 0, 0, vector[0], vector[1], vector[2], 
               color=color, arrow_length_ratio=0.1, linewidth=2, label=label)



ax2.set_xlim([-0.8, 0.8])
ax2.set_ylim([-0.8, 0.8])
ax2.set_zlim([-0.2, 0.6])
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.set_title('YUV Color Space')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()