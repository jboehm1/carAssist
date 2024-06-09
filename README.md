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









# CarAssist

CarAssist is a project for lane detection and stereovision using OpenCV. The project includes various image processing techniques and utilizes Kalman filters for lane detection. It can operate in both 2D and 3D modes to perform lane detection and disparity map computation.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
- [Contributing](#contributing)
- [License](#license)

## Features

- Lane detection using Kalman filters.
- Stereovision for disparity map computation.
- Configurable to run in 2D or 3D mode.
- Display of intermediate results.
- Multithreading for improved performance.

## Requirements

- OpenCV 4.9.0 or later.
- CMake 3.10 or later.
- C++17 compatible compiler.
- GoogleTest for unit testing.

## Installation

1. **Clone the repository:**
   ```sh
   git clone https://github.com/jboehm1/carAssist.git
   cd carAssist




# CarAssist

CarAssist is a project for lane detection and stereovision using OpenCV. The project includes various image processing techniques and utilizes Kalman filters for lane detection. It can operate in both 2D and 3D modes to perform lane detection and disparity map computation.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
- [Contributing](#contributing)
- [License](#license)

## Features

- Lane detection using Kalman filters.
- Stereovision for disparity map computation.
- Configurable to run in 2D or 3D mode.
- Display of intermediate results.
- Multithreading for improved performance.

## Requirements

- OpenCV 4.9.0 or later.
- CMake 3.10 or later.
- C++17 compatible compiler.
- GoogleTest for unit testing.

## Installation

1. **Clone the repository:**
   ```sh
   git clone https://github.com/jboehm1/carAssist.git
   cd carAssist
      ```
Build the project:
   ```sh
Copy code
mkdir build
cd build
cmake ..
make
   ```
## Usage
### Running the Program
To run the program, navigate to the build directory and execute the program:
   ```sh
Copy code
./CarAssist [options] [path]
### Options
3d: Run in 3D mode (stereovision).
disp: Display intermediate filtering results.
### Arguments
[path]: Path to the images. Defaults to ../CarAssist/img/20.
### Examples
Run in 2D mode with default path:
   ```
sh
Copy code
./CarAssist
   ```
Run in 3D mode with default path:
   ```
sh
Copy code
./CarAssist 3d
   ```
Run in 2D mode with intermediate results display and default path:

sh
Copy code
./CarAssist disp
Run in 3D mode with intermediate results display and specified path:
   ```
sh
Copy code
./CarAssist 3d disp /path/to/images
   ```
## Code Structure
css
Copy code
CarAssist/
├── CMakeLists.txt
├── README.md
├── src/
│ ├── imageProcessing.cpp
│ ├── imageProcessing.hpp
│ ├── laneDetection.cpp
│ ├── laneDetection.hpp
│ ├── main.cpp
│ ├── stereo.cpp
│ ├── stereo.hpp
│ ├── videoAnalysis.cpp
│ └── videoAnalysis.hpp
├── test/
│ ├── CMakeLists.txt
│ └── test_image_processing.cpp
└── img/
└── 20/
└── image_0/
└── *.png


## Contributing
Contributions are welcome! Please follow these steps:

Fork the repository.
Create a new branch.
Make your changes.
Submit a pull request.
##  License
This project is licensed under the MIT License. See the LICENSE file for details.
