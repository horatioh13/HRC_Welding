import cv2
import numpy as np
import time

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

print("Press 'q' to exit the program.")

def get_move():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        return None

    # Rotate the image by 180 degrees
# frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Estimate pose for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        position = robot.get_actual_tcp_pose()
        print(position)
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

        #convertion en matrice 4x4 of the UR
        popo=[[position[0]],[position[1]],[position[2]]]
        T_base_tcp = np.hstack([rotation_matrix2,popo])
        T_base_tcp = np.vstack([T_base_tcp, [0,0,0,1]])
        print(T_base_tcp)

        #convertion en matrice 4x4 of the aruco
        pipi=[[tvecs[0,0,0]],[tvecs[0,0,1]],[tvecs[0,0,2]]]
        T_tcp_aruco = np.hstack([rotation_matrix,pipi])
        T_tcp_aruco = np.vstack([T_tcp_aruco, [0,0,0,1]])
        print(T_tcp_aruco)

        T_base_aruco = np.dot(T_base_tcp, T_tcp_aruco)
        print(T_base_aruco)
        time.sleep(1)

        # Extract position and orientation
        R_base_aruco = T_base_aruco[:3, :3]
        t_base_aruco = T_base_aruco[:3, 3]

        # Convert rotation matrix to rotation vector
        rotation_, _ = cv2.Rodrigues(R_base_aruco)
        rotation_=np.round(rotation_,4)
        #mise en forme
        tutu = np.array2string(rotation_, separator=', ', suppress_small=True, formatter={'all': lambda x: str(x)}).strip("[]")
        clean_output = tutu.replace("[", "").replace("]", "").replace("\n", "").strip()
        # clean_output = ", ".join(clean_output.split())  # Ensure proper spacing

        command="movel(posetrans(p["+str(round(t_base_aruco[0],4))+","+str(round(t_base_aruco[1],4))+","+str(round(t_base_aruco[2],4))+","+clean_output+"],p[0,0,0.3,0,3.142,0]),a=0.2,v=0.2)"
        print(command)
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

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return None

while True:
    get_move()
    time.sleep(1)
# Release the capture and close windows
# cap.release()
# cv2.destroyAllWindows()
