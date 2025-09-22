import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
img = aruco.drawMarker(aruco_dict, 2, 170)  # marker ID 0, size 1000px
cv2.imwrite("Marker2.png", img)
