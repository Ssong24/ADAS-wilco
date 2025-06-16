# CAN Radar Target Visualizer

A real-time radar data visualization tool that reads CAN messages from a radar sensor, parses target information, and displays the targets in a 2D bird’s-eye view using OpenCV.

---

## 🚀 Features

- Receives radar data over a CAN interface (e.g., `slcan0`)
- Parses object position and velocity from CAN frames
- Converts radar coordinates to pixel positions
- Renders a bird’s-eye view of the scene with target markers and labels
- Real-time performance with OpenCV rendering

---

## 🧩 Dependencies

Install required packages:
```bash
sudo apt update
sudo apt install libopencv-dev can-utils build-essential
