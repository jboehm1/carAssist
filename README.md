# CarAssist

CarAssist is a project for lane detection and stereovision using OpenCV. The project includes various image processing techniques and utilizes Hough Lines for lane detection. It can operate in both 2D and 3D modes to perform lane detection and disparity map computation.
Work is still needed to get a real object detection from stereovision.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
- [Contributing](#contributing)
- [License](#license)

## Features

- Lane detection using Hough Lines.
- Stereovision for disparity map computation.
- Configurable to run in 2D (lane detection) or 3D mode (obstacle detection).
- Multithreading for improved performance.

## Requirements

- OpenCV 4.9.0 or later.
- CMake 3.10 or later.
- C++17 compatible compiler.
- GoogleTest for unit testing.

## Installation

1. **Clone the repository:**
   ```
   git clone https://github.com/jboehm1/carAssist.git
   cd carAssist
   ```
2. **Build the project:**
   ```
    mkdir build
    cd build
    cmake ..
    make
    ```
## Usage
### Running the Program
To run the program, navigate to the **`build/src`** directory and execute the program:
    ```
    cd build/src
    ./CarAssist [options] [path]
    ```
    
### Image dataset
Download from: https://we.tl/t-wbNQkKkW4B 

### Options
- `3d`: Run in 3D mode (stereovision).
- `disp`: Display intermediate filtering results.
### Arguments
- `[path]`: Path to the images. Defaults to `../CarAssist/img/20`.

### Examples
- Run in 2D mode with default path:
   ```
    ./CarAssist
   ```
- Run in 3D mode with default path:
   ```
    ./CarAssist 3d
   ```
- Run in 2D mode with intermediate results display and default path:
    ```
    ./CarAssist disp
    ```
- Run in 3D mode with intermediate results display and specified path:
   ```
    ./CarAssist 3d disp /path/to/images
   ```

### Testing
GoogleTests are implemented to test the main function from the lane detection algorithm.
   ```
   cd buidl/test
./test_image_processing
   ```
## Code Structure

```bash
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
│ └── stereo.hpp
├── test/
│ ├── CMakeLists.txt
│ └── test_image_processing.cpp
└── Img
```

## Contributing
Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a new branch.
3. Make your changes.
4. Submit a pull request.
##  License
This project is licensed under the MIT License.
