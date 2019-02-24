# 18h-krang-utils
This repo contains helper files containing functions used throughout the repos of 18.

## Dependencies
- DART
 [Dart Homepage](https://dartsim.github.io)
 
- lapack

      sudo apt install liblapack-dev

## Installation

    git clone https://github.gatech.edu/WholeBodyControlAttempt1/18h-krang-utils
    cd 18h-krang-utils
    mkdir build
    cd build
    cmake ..
    sudo make install
    sudo ldconfig

## Uninstall
To remove system files created by the installation of this repo.

    sudo make uninstall

### adrc
TODO: Add Description

### balance
TODO: Add Description

### collision
TODO: Add Description

### convert\_pose\_formats
Includes methods that converts between Munzir's and DART's coordinates.

### eso
TODO: Add Description

### file\_ops
Contains methods that help in file operations, such as reading from a file and
extracting base file name

### lqr
TODO: Add Description

### random
Contains methods dealing with random number generation
