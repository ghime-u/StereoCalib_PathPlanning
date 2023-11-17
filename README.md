# Path Planning, Obstacle Avoidance using Stereo Camera, RRT, and CNN

## Abstract

This project combines stereo camera techniques, Rapidly Exploring Random Trees (RRT), and Convolutional Neural Networks (CNNs) for depth estimation, optimal path planning, and traffic sign detection. The integration of these technologies provides a comprehensive solution for robotic navigation in complex environments.

## Table of Contents

1. [Introduction](#i-introduction)
2. [Related Work](#ii-related-work)
3. [Problem and Methodology](#iii-problem-and-methodology)
4. [Experiments and Results](#iv-experiments-and-results)
5. [Summary](#v-summary)
6. [References](#vi-references)
7. [Usage](#vii-usage)

## I. Introduction

In scenarios such as search and rescue, firefighting, or bomb detonation, controlling a robot's movement through obstacles can be challenging. This project addresses this issue by utilizing stereo cameras for depth estimation and RRT for optimal path planning. The combination of these techniques provides a systematic approach to navigating complex environments.

## II. Related Work

### A. [A Taxonomy and Evaluation of Dense Two-Frame Stereo Correspondence Algorithms – Daniel Scharstein, Richard Szeliski](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf)

Evaluation of stereo correspondence algorithms influencing the triangulation method used in this project.

### B. [Rapidly Exploring Random Trees: Progress and Prospects - Steven M LaValle, James J.Kuffner](http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf)

Paper exploring the design and implementation of RRT for path planning, emphasizing incremental tree construction.

### C. [High-quality Monocular Depth Estimation via Transfer Learning](http://www.cs.cornell.edu/~asaxena/learningdepth/)

Highlights challenges with monocular cameras and introduces solutions using transfer learning on CNNs.

## III. Problem and Methodology

### A. The Setup

- Stereo camera module created using a Logitech webcam and a Sony IMX586 camera module, establishing a baseline of approximately 28cm between them.

### B. Camera Calibration and Un-distortion

- Camera calibration involves obtaining the camera matrix and distortion parameters using OpenCV's functions.

### C. Obstacle Detection

- Implemented using red rectangles for simulation, utilizing contour detection and HSV color space for shape identification.

### D. Depth Estimation

- Depth estimated using feature points, frame shapes from both cameras, field of view, and baseline. Disparity and depth calculated based on triangulation methods.

### E. RRT Map

- Map created using Pygame, defining map dimensions, start and end points, and obstacle locations. RRT algorithm applied to explore the entire map efficiently.

### F. RRT Path Planning

- Involves generating random samples, checking collisions, and creating edges to find an optimal path. Algorithm biased to explore unexplored areas.

### G. Traffic Sign Recognition using PyTorch and TensorFlow Keras

- CNN trained to recognize traffic signs using two datasets. The trained model can be used to detect road signs in the environment, influencing the path planning process.

## IV. Experiments and Results

- **Depth Detection:** Algorithm showed promising results with an error of approximately 15cm compared to manual measurements and LiDAR readings.

- **RRT Path Planning:** Algorithm successfully planned optimal paths through complex environments.

- **CNN Experiment:** While CNN models achieved satisfactory accuracy, the importance of label weight distribution and the need for a substantial dataset are highlighted.

## V. Summary

Stereo cameras offer a cost-effective alternative to LiDAR for depth estimation and path planning. The experiment demonstrates the potential of self-made stereo cameras in overcoming challenges, paving the way for more accessible autonomous vehicle technology.

## VI. References

1. [Stereo Correspondence Taxonomy](https://vision.middlebury.edu/stereo/taxonomy-IJCV.pdf)
2. [Rapidly Exploring Random Trees: Progress and Prospects](http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf)
3. [Monocular Depth Estimation via Transfer Learning](http://www.cs.cornell.edu/~asaxena/learningdepth/)
4. [Pygame Documentation](https://www.pygame.org/news)
5. [Depth Perception using Stereo Camera](https://learnopencv.com/depth-perception-using-stereo-camera-python-c/)
6. [Tensorflow Keras Documentation](https://www.tensorflow.org/api_docs/python/tf/keras)
7. [Video: Self-Driving Car with Stereo Cameras](https://www.youtube.com/watch?v=KOSS24P3_fY)
8. [Middlebury Stereo Vision Datasets](https://benchmark.ini.rub.de/index.html)





## Usage

1. **Stereo Camera Setup and Calibration:**

    - Connect your stereo camera setup.
    - Run the `saveimages.py` script:

        ```bash
        python saveimages.py
        ```

    - Press 's' to save images for calibration for both left and right matrices.

    - Run the `stereocalibration.py` script:

        ```bash
        python stereocalibration.py
        ```

        - Check if your images are saved in the same resource folder. If not, declare the correct path in the script.

2. **Obstacle Detection:**

    - Run the `obstacledetection.py` script:

        ```bash
        python obstacledetection.py
        ```

        - This will find and locate obstacles and estimate their depth.

3. **Path Planning with RRT:**

    - Run the `main.py` script for RRT path planning:

        ```bash
        python main.py
        ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

