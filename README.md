# Trifecta Linux Setup Guide

![CI](https://github.com/Triangle-Man-LLC/Trifecta-Sample-Linux/actions/workflows/main.yml/badge.svg)

This repository contains examples for using Trifecta devices with Linux systems.
It has been tested on mainline Linux machines, but may also work on other POSIX machines.

For examples using Python (also Linux-compatible), see the following: <a href = "https://github.com/Triangle-Man-LLC/Trifecta-Python-Samples/tree/main/Trifecta-Python">Trifecta Python Example</a>

## System Requirements
- Linux system (Ubuntu/Debian recommended) or other POSIX (untested)
- CMake (version 3.10 or higher)
- GCC/G++ compiler
- Basic build tools (make, git)
- Serial port access permissions

## Installation Steps

### 1. Clone the repository
```bash
git clone https://github.com/Triangle-Man-LLC/Trifecta-Sample-Linux.git
cd Trifecta-Sample-Linux
```

### 2. Create the build folder, and build the project:
```bash
mkdir build && cd build && cmake .. && make
```

### 3. Run the executable:
```bash
./example_serial_read
```

## Additional Information

NOTE: If you have never used a serial port before on your machine, you will need to give the appropriate permissions:
`sudo usermod -a -G dialout $USER`

Then, verify that the serial port appears using:
`ls /dev/tty*`
You should see a port like `/dev/ttyACM0` appear in this list.