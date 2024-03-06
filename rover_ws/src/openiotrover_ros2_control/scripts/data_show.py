import numpy as np
import cv2
import time
# Load the depth_displays_array.npy file
depth_displays_array = np.load("depth_displays_array.npy")
for i in range(depth_displays_array.shape[2]):
    depth_display = depth_displays_array[:, :, i]
    cv2.imshow('Stereo Camera - Depth Image', depth_display)
    cv2.waitKey(1)  # You may need to adjust this depending on your system
    time.sleep(.2)
