import cv2
import numpy as np
import time
import math
import socket

# Robot's IP address and port
robot_ip = "169.254.56.120"
robot_port = 30003

from scipy.sparse import hstack

import URBasic
from scipy.spatial.transform import Rotation as R

# Load the predefined dictionary for ArUco markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
# Define the size of the marker (in meters or any consistent unit)
marker_length = 0.03  # Example: 3 cm

# Load camera calibration parameters (replace with your values)
# Use camera calibration tools to get these values if you don't have them
camera_matrix = np.array([[542.6002139, 0, 352.284796], [0, 540.00620752, 236.2310265], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([[ 0.03296275,0.28236317, -0.001922,0.00700259,-0.73875485]], dtype=np.float32)
# initialise robot with URBasic

print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host='169.254.56.120',robotModel=robotModel)
# robot = URBasic.urScriptExt.UrScriptExt(host='192.168.56.101',robotModel=robotModel)

# Initialize webcam
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("Error: Cannot access the camera.")
    exit()

def get_rotation_matrix(samples): #function that returns rvec and tvec of the aruco
    # Capture frame-by-frame
    vectorTable = []
    for i in range(samples):
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            return None

        # Rotate the image by 180 degrees if necessary
        # frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            newRow = [rvecs, tvecs]
            print(newRow)
            vectorTable.append(newRow)

            show_frame(ids,frame, corners, rvecs, tvecs)

    # average of all the measurement taken
    for row in vectorTable:
        #mean translation vector
        rvecs2 = row[0]
        tvecs2 = row[1]

        TXsum =+ tvecs2[0,0,0]
        TYsum =+ tvecs2[0,0,1]
        TZsum =+ tvecs2[0,0,2]

        #mean rotation vector
        RXsum = + rvecs2[0, 0, 0]
        RYsum = + rvecs2[0, 0, 1]
        RZsum = + rvecs2[0, 0, 2]

    rvecs[0, 0, 0] = RXsum / len(vectorTable)
    rvecs[0, 0, 1] = RYsum / len(vectorTable)
    rvecs[0, 0, 2] = RZsum / len(vectorTable)

    tvecs[0, 0, 0] = TXsum / len(vectorTable)
    tvecs[0, 0, 1] = TYsum / len(vectorTable)
    tvecs[0, 0, 2] = TZsum / len(vectorTable)

    return rvecs, tvecs

def show_frame(ids,frame,corners,rvecs,tvecs):#function tha draw axis on image
    # print(command)
    for i in range(len(ids)):
        # Draw the marker border
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Draw the axis on the marker
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length / 2)

        # Display the ID of the marker
        center = np.mean(corners[i][0], axis=0).astype(int)
        cv2.putText(frame, f"ID: {ids[i][0]}", (center[0], center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Calculate the offset from the center of the image
        image_center = np.array([frame.shape[1] / 2, frame.shape[0] / 2])
        offset = center - image_center
        # print(f"Marker ID: {ids[i][0]} - Offset from center (x, y): ({offset[0]}, {offset[1]})")

    # Display the resulting frame
    cv2.imshow('ArUco Detection with Axis', frame)

def matrix_calculation(rvecs, tvecs): #function that outputs the command for the robot according to aruco position
    position = robot.get_actual_tcp_pose()
    tcp_rotation = np.array([[[position[3], position[4], position[5]]]])
    tcp_translation = np.array([[position[0]], [position[1]], [position[2]]])

    # Convert rotation vector to rotation matrix
    for rvec, tvec in zip(rvecs, tvecs):
        # Convert rvec to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        print("Rotation Vector (rvec):\n", rvec)
        print("Rotation Matrix:\n", rotation_matrix)

    # Convert rotation vector to rotation matrix from ur
    for tcp_rotation, tcp_translation in zip(tcp_rotation, tcp_translation):
        # Convert rvec to rotation matrix
        rotation_matrix2, _ = cv2.Rodrigues(tcp_rotation)
        print("Rotation Vector (rvec):\n", tcp_rotation)
        print("Rotation Matrix:\n", rotation_matrix)

    # convertion en matrice 4x4 of the UR
    popo = [[position[0]], [position[1]], [position[2]]]
    T_base_tcp = np.hstack([rotation_matrix2, popo])
    T_base_tcp = np.vstack([T_base_tcp, [0, 0, 0, 1]])
    # print(T_base_tcp)

    # convertion en matrice 4x4 of the aruco
    pipi = [[tvecs[0,0,0]], [tvecs[0,0,1]], [tvecs[0,0,2]]]
    T_tcp_aruco = np.hstack([rotation_matrix, pipi])
    T_tcp_aruco = np.vstack([T_tcp_aruco, [0, 0, 0, 1]])
    # print(T_tcp_aruco)

    T_base_aruco = np.dot(T_base_tcp, T_tcp_aruco)
    # print(T_base_aruco)
    time.sleep(1)

    # Extract position and orientation
    R_base_aruco = T_base_aruco[:3, :3]
    t_base_aruco = T_base_aruco[:3, 3]

    # Convert rotation matrix to rotation vector
    rotation_, _ = cv2.Rodrigues(R_base_aruco)
    rotation_ = np.round(rotation_, 4)
    # mise en forme
    tutu = np.array2string(rotation_, separator=', ', suppress_small=True, formatter={'all': lambda x: str(x)}).strip(
        "[]")
    clean_output = tutu.replace("[", "").replace("]", "").replace("\n", "").strip()
    # clean_output = ", ".join(clean_output.split())  # Ensure proper spacing

    command = "current_pose = p[" + str(round(t_base_aruco[0], 4)) + "," + str(round(t_base_aruco[1], 4)) + "," + str(
        round(t_base_aruco[2], 4)) + "," + clean_output + "]"
    return command

def get_move(sample):
    # With cv2, finds the aruco and outputs rvec and tvec
    # and displays image with axis
    rotation = get_rotation_matrix(sample) #two variables are outputed rvecs and tvecs

    # calculation of the new moovement of the robot
    outputCommand = matrix_calculation(rotation[0],rotation[1])

    return outputCommand

for i in range(4):
    command=get_move(10)
    tcp_command2 = """
def move():
    """+command+"""
    target_pose = pose_trans(current_pose, p[0.0, 0, 0.2, 0.0, 3.142, 0.0])
    movel(target_pose, a=0.1, v=0.1)
end
move()
"""
    # send the command
    # Create a socket connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((robot_ip, robot_port))

    # Send the URScript
    print(tcp_command2)
    # s.send(str.encode(tcp_command2))
    # Close the connection
    time.sleep(2)
s.close()
# Release the capture and close windows
# cap.release()
# cv2.destroyAllWindows()
