# 1. **GPSOpenCl**

[![GPSOpenClTest](https://github.com/beratatmaca/GPSOpenCl/actions/workflows/debug.yml/badge.svg?branch=main)](https://github.com/beratatmaca/GPSOpenCl/actions/workflows/debug.yml)

- [1. **GPSOpenCl**](#1-gpsopencl)
  - [1.1. **Installation**](#11-installation)
  - [1.2. **To Do List**](#12-to-do-list)


## 1.1. **Installation**
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libopencl-dev
sudo apt-get install libclfft2 clfft-client libclfft-dev
cmake -S . -B ./build
cd build && make
```
## 1.2. **To Do List**
- [ ] Read Settings
- [ ] Read Raw Data Stream
- [x] Do Acquisiton
- [ ] Do Tracking
- [ ] Decode bitstream
- [ ] Find psuedoranges
- [ ] Calculate position
