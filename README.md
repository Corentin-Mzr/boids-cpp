# Boids-Cpp

## Description

Flock simulation based on [**Craig W. Reynolds rules**](https://en.wikipedia.org/wiki/Boids).

## Example

## Table of contents

## Installation

### Prerequisites

- C++ compiler (gcc, g++, cl, clang)
- CMake >= 3.30

For Linux, you also need to install the following dependencies:  

```bash
sudo apt-get update
sudo apt-get install -y libx11-dev \
                        libxcursor-dev \
                        libxi-dev \
                        libxrandr-dev \
                        libudev-dev \
                        libgl1-mesa-dev \
                        libfreetype6-dev \
                        libjpeg-dev \
                        libopenal-dev \
                        libflac-dev \
                        libvorbis-dev \
                        libxcb1-dev \
                        libxcb-image0-dev \
                        libxcb-randr0-dev \
                        libxcb-xtest0-dev
```

### Clone the repository

```bash
git clone https://github.com/Corentin-Mzr/boids-cpp.git
```

### Build the project

From the root folder, execute the following commands:

```bash
cmake -B build
cmake --build build --parallel
```

### Run the program

To run the program, launch it from the build/bin folder:

```bash
cd build/bin
./<filename>.exe
```

## Libraries

The following libraries have been used for this program:

- [**SFML**](https://github.com/SFML/SFML)
- [**ImGui**](https://github.com/ocornut/imgui)

## License

This program is under the [**MIT License**](LICENSE.md)
