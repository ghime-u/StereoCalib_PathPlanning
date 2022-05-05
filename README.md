# StereoCalib_PathPlanning
Obstacle detection using stereo camera for depth estimation and path planning using RRT

%%%%%% Utkarsh Ghime

%%%%% Windows 10, python 3.9, pycharm

%%%%% instructions
1) Create a stereo Camera Setup, install pygame, numpy, opencv, imutils
2) Run the saveimages.py and press 's' to save images for calibration for both left and right matrix

3) Run stereo calibration.py (check if your images are saved in the same resource folder if not declare path)

4) Run Obstacledetection.py to find and locate obstacle and estimate its depth

5) Run main.py( RRT path planning) to run path planning file to find optimal path

%traffic sign detection:
The pytorch model was implemented on a small dataset of 950 images with very few images in some labels, the accuracy capped at 64%, tweaking parameters even further resulted in overfitting for some labels
The german traffic sign database was implemented using tensorflow keras. it gives around 93% accuracy

%%%%%%
Thank You
