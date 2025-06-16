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

## 🧠 Code Structure & Explanation

#### 📦 Radar Target Data Structure
Each radar target is represented using the following structure:

```cpp
typedef struct radar_target {
    float x;    // position x-axis
    float y;    // position y-axis
    float vx;   // Relative velocity x-axis 
    float vy;   // Relative velocity y-axis
    float rcs;  // Radar Cross Section
} radar_target;
```


#### 🔧 Core Components

| **Function / Section**   | **Description**                                                                 |
| ------------------------ | ------------------------------------------------------------------------------- |
| `open_port(const char*)` | Initializes and opens a CAN socket using SocketCAN (e.g., for `slcan0` device). |
| `close_port()`           | Closes the CAN socket cleanly.                                                  |
| `radar_target` struct    | Defines a radar target with position `(x, y)`, velocity `(vx, vy)`, and RCS.    |
| `extract_dist_long()`    | Parses and scales of x from raw CAN data.                                       |
| `extract_dist_lat()`     | Parses and scales of y from raw CAN data.                                       |
| `extract_vrel_long()`    | Extracts relative velocity vx.                                                  |
| `extract_vrel_lat()`     | Extracts relative velocity vy.                                                  |
| `queryDisplayVid()`      | Prompts user to choose whether to display live camera feed.                     |
| `querySaveVid()`         | Prompts user to enter filename if saving video is enabled.                      |
| `displayNoSave()`        | Displays camera stream with radar overlays but does not save.                   |
| `displaySave()`          | Displays and records camera stream with radar overlays to a video file.         |




