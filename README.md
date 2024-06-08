# Stereo Vision and Obstacle Detection

This project implements a stereo vision system for detecting obstacles using disparity maps and depth estimation. It utilizes OpenCV for image processing, disparity map calculation, and visualization.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Parameters](#parameters)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction
This project demonstrates the use of stereo vision for real-time obstacle detection. By calculating disparity maps from stereo image pairs and converting them into depth maps, the system identifies obstacles within a specified distance.

## Features
- Real-time stereo image processing
- Disparity map computation using StereoSGBM
- Depth map calculation from disparity
- Obstacle detection within a specified distance
- Visualization of left, right images, and the disparity map

## Installation
### Prerequisites
- C++17 or higher
- OpenCV 4.0 or higher
- CMake 3.10 or higher

### Building the Project
1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/stereo-vision-obstacle-detection.git
    cd stereo-vision-obstacle-detection
    ```

2. Create and enter a build directory:
    ```sh
    mkdir build
    cd build
    ```

3. Configure the project using CMake:
    ```sh
    cmake ..
    ```

4. Build the project:
    ```sh
    make
    ```

## Usage
Run the executable with the desired input source (e.g., image sequence, video):
```sh
./StereoVisionObstacleDetection
