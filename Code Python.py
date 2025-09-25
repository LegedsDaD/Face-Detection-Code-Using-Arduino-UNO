import subprocess
import sys
import time

# --- Function to install packages ---
def install_package_if_missing(package_name):
    """
    Checks if a Python package is installed and installs it if missing.
    Exits the script if installation fails.
    """
    try:
        # Attempt to import the package
        # Special handling for opencv-python as it imports as 'cv2'
        if package_name == "opencv-python":
            import cv2
        elif package_name == "pyserial":
            import serial
        else:
            __import__(package_name.replace('-', '_')) # General case for other packages
        print(f"'{package_name}' is already installed.")
    except ImportError:
        print(f"'{package_name}' not found. Attempting to install...")
        try:
            # Use sys.executable to ensure pip is run with the current Python interpreter
            subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
            print(f"Successfully installed {package_name}")
            # Try importing again after successful installation
            if package_name == "opencv-python":
                import cv2
            elif package_name == "pyserial":
                import serial
            else:
                __import__(package_name.replace('-', '_'))
        except subprocess.CalledProcessError as e:
            print(f"Error installing {package_name}: {e}")
            print("Please ensure you have an active internet connection and sufficient permissions.")
            print("You might need to run your terminal/command prompt as an administrator.")
            sys.exit(1) # Exit if essential package fails to install
        except Exception as e:
            print(f"An unexpected error occurred during installation of {package_name}: {e}")
            sys.exit(1)

# --- Ensure required packages are installed before proceeding ---
# numpy is typically installed as a dependency of opencv-python,
# so we don't explicitly list it for installation here.
required_packages = ["opencv-python", "pyserial"]
for pkg in required_packages:
    install_package_if_missing(pkg)

# --- Now, it's safe to import the installed packages ---
import cv2
import serial
import numpy # numpy is used by opencv, good to explicitly import it if you use its functions

# --- Configuration ---
# !!! IMPORTANT: Change this to your Arduino's serial port !!!
# Windows: 'COMx' (e.g., 'COM3')
# Linux/macOS: '/dev/ttyACM0' or '/dev/ttyUSB0'
ARDUINO_PORT = 'COM3'
BAUD_RATE = 9600
CAMERA_INDEX = 0      # Usually 0 for default webcam, try 1, 2 etc. if 0 doesn't work

# Servo angle ranges (0-180 degrees)
MIN_ANGLE = 0
MAX_ANGLE = 180

# Initial servo angles (can be adjusted to match your physical setup)
# These are the angles the servos will go to at startup.
INITIAL_SERVO_X = 90
INITIAL_SERVO_Y = 90

# Camera resolution (adjust if needed, ideally match your webcam's capabilities)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- Face Detection ---
# Load OpenCV's pre-trained Haar Cascade for face detection
# This file is usually located in your opencv-python installation path
# e.g., C:\Python3x\Lib\site-packages\cv2\data\haarcascade_frontalface_default.xml
# You might need to provide the full path if it's not found automatically.
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
if face_cascade.empty():
    print("Error: Could not load Haar cascade XML file.")
    print("Ensure 'haarcascade_frontalface_default.xml' is present and accessible.")
    print("It should be in your OpenCV data directory, typically part of the 'cv2.data.haarcascades' path.")
    sys.exit(1)

# --- Helper Functions ---
def map_value(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    # Ensure no division by zero
    if (in_max - in_min) == 0:
        return out_min # Or handle as an error
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_servo_angles(ser, angle_x, angle_y):
    """Sends servo angles to Arduino via serial."""
    data_to_send = f"{int(angle_x)},{int(angle_y)}\n"
    try:
        ser.write(data_to_send.encode())
        # Uncomment the line below for debugging serial output
        # print(f"Sent: {data_to_send.strip()}")
    except serial.SerialTimeoutException:
        print("Serial write timed out.")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"Error sending data: {e}")

# --- Main Program ---
def main():
    ser = None # Initialize ser to None
    cap = None # Initialize cap to None

    try:
        # Initialize serial communication with Arduino
        print(f"Attempting to connect to Arduino on {ARDUINO_PORT}...")
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Give some time for Arduino to reset after serial connection

        if not ser.is_open:
            raise serial.SerialException(f"Failed to open serial port {ARDUINO_PORT}")

        print(f"Connected to Arduino on {ARDUINO_PORT}")

        # Initialize camera
        print(f"Attempting to open webcam (index: {CAMERA_INDEX})...")
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        if not cap.isOpened():
            print("Error: Could not open webcam.")
            print("Ensure the webcam is connected, drivers are installed, and it's not in use by another application.")
            sys.exit(1)

        print("Webcam opened successfully. Press 'q' to quit.")

        current_angle_x = INITIAL_SERVO_X
        current_angle_y = INITIAL_SERVO_Y

        # Send initial servo positions to ensure they are at a known state
        send_servo_angles(ser, current_angle_x, current_angle_y)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame from webcam. Exiting.")
                break

            # Flip frame horizontally to make it more intuitive (like a mirror)
            frame = cv2.flip(frame, 1)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,       # Reduce image size by 10% at each scale
                minNeighbors=5,        # Minimum number of neighboring rectangles to retain a detection
                minSize=(30, 30),      # Minimum face size to detect (width, height)
                flags=cv2.CASCADE_SCALE_IMAGE
            )

            if len(faces) > 0:
                # Assuming we only care about the first (and usually largest) face found
                (x, y, w, h) = faces[0]

                # Draw rectangle around the face
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # Calculate the centroid of the face
                face_center_x = x + w // 2
                face_center_y = y + h // 2

                # Draw a circle at the center of the face
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)

                # Map face position to servo angles
                # The mapping assumes camera's left (0) maps to MIN_ANGLE, right (FRAME_WIDTH) to MAX_ANGLE.
                # Adjust MIN_ANGLE/MAX_ANGLE or swap them if servo moves in the wrong direction.
                angle_x = map_value(face_center_x, 0, FRAME_WIDTH, MIN_ANGLE, MAX_ANGLE)
                angle_y = map_value(face_center_y, 0, FRAME_HEIGHT, MIN_ANGLE, MAX_ANGLE)

                # Simple smoothing for servo movement
                # A value of 1.0 means no smoothing (instantaneous update)
                # A value closer to 0.0 means more smoothing (slower, delayed movement)
                smoothing_factor = 0.5
                current_angle_x = (1 - smoothing_factor) * current_angle_x + smoothing_factor * angle_x
                current_angle_y = (1 - smoothing_factor) * current_angle_y + smoothing_factor * angle_y

                send_servo_angles(ser, current_angle_x, current_angle_y)

            # Display the frame
            cv2.imshow('Face Tracking', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except serial.SerialException as e:
        print(f"CRITICAL ERROR: Serial Port Error: {e}.")
        print("Please check:")
        print(f"1. Is the Arduino connected to '{ARDUINO_PORT}'?")
        print("2. Is the Arduino IDE's Serial Monitor closed?")
        print("3. Is the correct port selected in the Python script's ARDUINO_PORT variable?")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Clean up resources
        if cap is not None and cap.isOpened():
            cap.release()
        cv2.destroyAllWindows()
        if ser is not None and ser.is_open:
            ser.close()
            print("Serial connection closed.")
        print("Program finished.")

if __name__ == '__main__':
    main()
