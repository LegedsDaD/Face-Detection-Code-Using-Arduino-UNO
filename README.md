# 🤖 Face Tracking Pan-Tilt System

## Project Overview

This project implements an **automated face-tracking system** using a Python script, OpenCV for computer vision, and an Arduino-controlled pan-tilt mechanism. The system captures video from a webcam, detects the largest human face in the frame, and sends corresponding angle commands to an Arduino Uno to precisely move two servos (for **Pan** and **Tilt** movement), keeping the face centered in the camera's view.

It is an excellent beginner-to-intermediate project for those interested in **robotics, computer vision, and physical computing**.

### Key Features

* **Real-time Face Detection:** Utilizes OpenCV's Haar Cascades for reliable, real-time face detection.
* **Two-Axis Control (Pan & Tilt):** Controls two independent servo motors to move a camera or sensor along the X (pan) and Y (tilt) axes.
* **Automated Dependency Installation:** The Python script automatically checks for and installs required packages (`opencv-python` and `pyserial`).
* **Serial Communication:** Robust communication between the Python host and the Arduino using the `pyserial` library.
* **Simple Angle Mapping:** Maps the face's screen position directly to the servo's physical angle range (0-180 degrees).
* **Movement Smoothing:** Implements a basic smoothing factor to prevent jittery or sudden servo movements.

<br>

## ⚙️ Hardware & Software Requirements

### Hardware

* **Arduino Uno** (or compatible board)
* **Two Servo Motors** (e.g., SG90 or MG996R)
* **Pan-Tilt Bracket** (recommended for mounting the servos and webcam)
* **Webcam** (any standard USB webcam)
* **Power Supply** (A separate power supply for the servos is highly recommended to avoid damaging the Arduino)

### Software

* **Arduino IDE**
* **Python 3.x**
* **Required Python Libraries:**
    * `opencv-python` (for video capture and face detection)
    * `pyserial` (for serial communication with Arduino)

<br>

## 🛠 Installation & Setup

### 1. Arduino Setup (The Pan-Tilt Mechanism)

1.  **Wire the Servos:**
    * [cite_start]Connect the **Pan** (X-axis) servo data pin to **Arduino Digital Pin 9**[cite: 1].
    * [cite_start]Connect the **Tilt** (Y-axis) servo data pin to **Arduino Digital Pin 10**[cite: 2].
    * **Crucially**, connect the servo **VCC** (Power) and **GND** (Ground) pins to an **external 5V power supply**, *not* directly to the Arduino's 5V pin, as the motors can draw too much current.
2.  **Upload the Code:**
    * Open the `Code Arduino IDE.ino` file in the Arduino IDE.
    * Ensure you have the **Servo** library installed (it's usually built-in).
    * Select your board and correct serial port.
    * Upload the code to your Arduino Uno.

[cite_start]The Arduino code initializes both servos to a **neutral 90-degree position** [cite: 3, 4] [cite_start]and then listens for a comma-separated angle command (e.g., `"90,90"`) over the serial port[cite: 6].

### 2. Python Setup (The Computer Vision Host)

1.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/LegedsDaD/Face-Detection-Code-Using-Arduino-UNO](https://github.com/LegedsDaD/Face-Detection-Code-Using-Arduino-UNO)
    cd Face-Detection-Code-Using-Arduino-UNO
    ```
2.  **Run the Script:**
    The Python script (`Code Python.py`) will automatically check for and install the necessary libraries (`opencv-python`, `pyserial`) when you run it for the first time.
    ```bash
    python "Code Python.py"
    ```

3.  **Configuration (Crucial Step!):**
    Before running, you **must** edit the **Configuration** section near the top of the `Code Python.py` file to match your setup:

    * **`ARDUINO_PORT`**: Change `'COM3'` to the serial port your Arduino is connected to.
        * **Windows:** e.g., `'COM4'`, `'COM5'`
        * **Linux/macOS:** e.g., `'/dev/ttyACM0'`, `'/dev/ttyUSB0'`
    * **`CAMERA_INDEX`**: Change `0` to `1` or `2` if your default webcam is not the one you want to use.

<br>

## 🚀 Usage

1.  **Ensure all connections are secure** and the Arduino is powered on.
2.  **Run the Python script** (after making sure the `ARDUINO_PORT` is correct):
    ```bash
    python "Code Python.py"
    ```
3.  The script will:
    * Connect to the specified serial port.
    * Initialize the webcam and display a window titled **'Face Tracking'**.
    * Start tracking.
4.  **Stand in front of the webcam.** A **blue rectangle** will be drawn around your face.
5.  As you move, the Pan-Tilt mechanism will automatically adjust the servos to keep the **green circle** (the calculated center of your face) in the center of the frame.
6.  To stop the program, click on the **'Face Tracking'** window and press the **`q`** key.

<br>

## Troubleshooting

| Problem | Possible Solution |
| :--- | :--- |
| **`SerialPortError`** (CRITICAL ERROR) | 1. **Check `ARDUINO_PORT`:** Is the port in `Code Python.py` correct? 2. **Close Serial Monitor:** Make sure the Arduino IDE's **Serial Monitor is closed**, as it will block the port. 3. **Arduino Connection:** Is the Arduino properly connected? |
| **"Could not open webcam"** | 1. **Check `CAMERA_INDEX`:** Try changing the index from `0` to `1` or `2`. 2. **Permissions:** Ensure the application has camera access permissions on your OS. 3. **Drivers:** Check if your webcam drivers are properly installed. |
| **"Error: Could not load Haar cascade XML file"** | The Haar cascade file is essential for face detection. Ensure your OpenCV installation is complete and the file `haarcascade_frontalface_default.xml` is in the correct path. |
| **Servos are jittery or reset** | The servos are drawing too much current. **You must use an external power supply** for the servos, as the Arduino's 5V line cannot power two motors reliably. |

<br>

## 🤝 Contributing

Contributions are welcome! If you have suggestions for improvement, such as implementing a faster object detection model (e.g., Mediapipe or Tiny-YOLO) or advanced PID control for smoother movement, please feel free to fork the repository and submit a Pull Request.

<br>

## 📄 License

This project is licensed under the **MIT License**. See the `LICENSE` file (if available) or the repository for details.

---

[cite_start]*(**Note:** The Arduino code requires the standard `<Servo.h>` library [cite: 1] [cite_start]and uses the serial port at a baud rate of **9600** [cite: 5] [cite_start]to receive angle commands in the format **`X_ANGLE,Y_ANGLE`**[cite: 6]. [cite_start]It uses the `constrain()` function [cite: 9] [cite_start]to ensure the received angles are safely between 0 and 180 degrees[cite: 10].)*
