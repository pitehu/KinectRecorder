# KinectRecorder
A Kinect recorder that captures and saves RGB image, depth(point cloud), depth frame mapped into color frame and skeleton position at 30FPS. Color images are stored in .jpg while point cloud is stored in .pcd and skeleton poistions in .txt.  For each frame, you will need about 435KB for color image, point cloud about 1.9MB, and depth in color image about 200KB. So please have enough disk space left. If you don't need any recording source, simply comment out the corresponding part and recompile. 

Also, please set your own paths for all the depended libraries in the project setting since your installation path may differ from mine.

Dependencies:
OpenCV
Kinect for Windows SDK 2.0
Point Cloud Library (for point cloud only)





