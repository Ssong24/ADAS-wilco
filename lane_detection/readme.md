# 🚗 Lane Detection with OpenCV

This is a basic C++ implementation of a **lane detection pipeline** using OpenCV. The code performs **gradient-based** and **color-based** filtering to identify lane lines in images or video frames, with an optional feature to display or save the output using your webcam.

---

## 🔧 Features

* Reads an input road image and resizes it.
* Sobel Edge Detection in X/Y direction
* Gradient Magnitude Thresholding
* Gradient Direction Thresholding
* HLS Color Thresholding on S and L channels
* Intended to be extended with lane masking and perspective warping.

---

## 📦 Dependencies

* C++
* OpenCV 3 or 4 (Tested with OpenCV 4+)
* CMake or `g++` with `pkg-config` for compilation
---
