from PIL import Image
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

img=cv2.imread('/home/roslab/mgr/maps/map.pgm')
img = np.asarray(img)

#path_map = img
path_map = np.ones((np.shape(img)[0],np.shape(img)[1],np.shape(img)[2]))
path_map = path_map*255
print(np.shape(img))
print(np.shape(path_map))
# cv2.imshow("obraz", img)
# cv2.imshow("obraz2", path_map)
# k = cv2.waitKey(0)
test1 = np.array([1,2,3])
test2 = np.array([1,3,4])
if test1.all() == test2.all():
    print('1')