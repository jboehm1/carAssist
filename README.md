# Car Assist: lane assist and environment awarness

This project is separated in two parts, the first using sgtandard machine visiojn algorithm like Hough Lines to detect road lanes and alert in case of lane change. The seccond part (in progress) is based on stereovision in order to get a vision of the surrounding, aiming to alert in case an obstacle is near the car.

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
This project demonstrates the use of mono and stereo vision for real-time obstacle detection. By using filtering and standard algorithm the system aims to detect and draw lanes. By calculating disparity maps from stereo image pairs and converting them into depth maps, the system identifies obstacles within a specified distance.

## Features
- Image procesing
- Lane detection
- Lane change detection
- Lane visualisation
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
    git clone https://github.com/jboehm1/cpp/tree/main/CarAssist/CarAssist
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
