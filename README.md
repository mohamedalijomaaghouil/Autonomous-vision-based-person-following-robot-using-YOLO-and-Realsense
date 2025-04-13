 🤖 Autonomous Vision-Based Person Following Robot Using YOLO and Realsense

This project implements a real-time person-following robot using YOLOv11 object detection and an Intel Realsense camera. It estimates the distance to a detected person and allows the robot to follow them using computer vision techniques.

---

 📌 Features

- Person detection with YOLOv11
- Distance estimation using focal length and bounding box
- Real-time video processing
- Easy calibration mode
- Python + OpenCV based implementation

---

 📂 Files

| File | Description |
|------|-------------|
| `test_camera.py` | Main Python script for detection & distance estimation |
| `Rapport_Avancement_Robot_Follow_Me_Mohamed_Ali_Jomaa_Ghouil.pdf` | Project report in French |
| `README.md` | Project overview |

---

 🚀 How It Works

1. The model detects people in each video frame.
2. Calibrates the focal length using a known distance.
3. Calculates real-time distance to each person.
4. Displays bounding boxes and distances on screen.

> 📏 Formula used:  
> `Distance = (Real Height × Focal Length) / Pixel Height`

---

 🧪 Requirements

- Python 3.8+
- [Ultralytics YOLOv11](https://github.com/ultralytics/ultralytics)
- OpenCV (`pip install opencv-python`)
- Realsense SDK (optional for hardware)



🔧 Setup

```bash
pip install ultralytics opencv-python
