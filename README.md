# UR-Auto-3D-Pad

This code is a proof of concept for automating an industrial production process in the manufacturing industry. The proof of concept was developed to show that the currently used quality control process for scanning products with an ultrasonic scanner can be automated. This was achieved by making a 3D scan with an Intel Realsense camera of the product that was submerged in water, creating a pointcloud from the 3D scan, creating a path for the CoBot to follow and then to have the CoBot follow the path with the ultrasonic scanner and creating an ultrasonic scan.

Hardware:
CoBot UR10e
Laptop Windows11
Intel Realsense D435
Ultrasonic scanner (Dummy)

Software

python version                   3.9.0 64-bit

imports main:
os
time
keyboard
- PadGen2D
- CameraHandling
- testSending

imports PadGen2D/3D:
matplotlib			                3.7.1
enum
numpy				                    1.24.3
open3d				                  0.17.0
cv2(opencv-python)		          4.7.0.72
pyrealsense2			              2.54.2.5684
csv
heapq
copy

imports CameraHandling:
math
time
cv2(opencv-python)		         4.7.0.72
numpy				                   1.24.3
pyrealsense2			             2.54.2.5684

imports testSending:
sys
logging
rtde				                   2016 version was used!
numpy				                   1.24.3
threading
time
csv


reference
testSending.py
This is adapted code that was originally made by LaurentBimont from:
https://github.com/LaurentBimont/RTDE-URx/tree/master
It was downloaded/adapted on the 17th of april

CameraHandling.py
This is adapted code that was originally made by leggedrobotics from:
https://github.com/leggedrobotics/librealsense-rsl/blob/master/wrappers/python/examples/opencv_pointcloud_viewer.py
It was downloaded/adapted on the 17th of april

RTDE folder
https://github.com/UniversalRobots/RTDE_Python_Client_Library/tree/main/rtde
