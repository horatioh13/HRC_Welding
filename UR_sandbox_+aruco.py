"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
Demonstration Video: https://youtu.be/HHb-5dZoPFQ
Explanation Video: https://www.youtube.com/watch?v=9XCNE0BmtUg

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html

"""
from PIL.ImageChops import offset

import URBasic
import math
import numpy as np
import sys
import cv2
import time
import imutils
from imutils.video import VideoStream
import math3d as m3d

"""SETTINGS AND VARIABLES ________________________________________________________________"""

# If this is run on a linux system, a picamera will be used.
# If you are using a linux system, with a webcam instead of a raspberry pi delete the following if-statement
ROBOT_IP = '169.254.56.120'

ACCELERATION = 0.1  # Robot acceleration value
VELOCITY = 0.1  # Robot speed value
"""open cv init variables-----------------------------------------------------------------------------"""
# Load the predefined dictionary for ArUco markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
# Define the size of the marker (in meters or any consistent unit)
marker_length = 0.03  # Example: 5 cm

# Load camera calibration parameters (replace with your values)
# Use camera calibration tools to get these values if you don't have them
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

video_resolution = (700, 400)  # resolution the video capture will be resized to, smaller sizes can speed up detection
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio  = video_resolution[0] / video_resolution[1]  # Aspect ration of each frame
video_viewangle_hor = math.radians(25)  # Camera FOV (field of fiew) angle in radians in horizontal direction

# Variable which scales the robot movement from pixels to meters.
m_per_pixel = 00.00006

# Initialize webcam
vs = VideoStream(src= 2 ,
                 usePiCamera= False,
                 resolution=video_resolution,
                 framerate = 13,
                 meter_mode = "backlit",
                 exposure_mode ="auto",
                 shutter_speed = 8900,
                 exposure_compensation = 2,
                 rotation = 0).start()
time.sleep(0.2)

# The Joint position the robot starts at
robot_startposition = (math.radians(-15),
                    math.radians(-70),
                    math.radians(50),
                    math.radians(-70),
                    math.radians(-90),
                    math.radians(0))

def set_lookorigin():
    """
    Creates a new coordinate system at the current robot tcp position.
    This coordinate system is the basis of the face following.
    It describes the midpoint of the plane in which the robot follows faces.

    Return Value:
        orig: math3D Transform Object
            characterises location and rotation of the new coordinate system in reference to the base coordinate system

    """
    position = robot.get_actual_tcp_pose()
    orig = m3d.Transform(position)
    return orig

# def move_to_face(list_of_facepos,robot_pos):
#     """
#     Function that moves the robot to the position of the face
#
#     Inputs:
#         list_of_facepos: a list of face positions captured by the camera, only the first face will be used
#         robot_pos: position of the robot in 2D - coordinates
#
#     Return Value:
#         prev_robot_pos: 2D robot position the robot will move to. The basis for the next call to this funtion as robot_pos
#     """
#
#
#     face_from_center = list(list_of_facepos[0])  # TODO: find way of making the selected face persistent
#
#     prev_robot_pos = robot_pos
#     scaled_face_pos = [c * m_per_pixel for c in face_from_center]
#     print("initialising robot1")
#
#     robot_target_xy = [a + b for a, b in zip(prev_robot_pos, scaled_face_pos)]
#     # print("..", robot_target_xy)
#
#     robot_target_xy = check_max_xy(robot_target_xy)
#     prev_robot_pos = robot_target_xy
#     print("initialising robo2")
#
#     x = robot_target_xy[0]
#     y = robot_target_xy[1]
#     z = 0
#     xyz_coords = m3d.Vector(x, y, z)
#
#     x_pos_perc = x / max_x
#     y_pos_perc = y / max_y
#
#     x_rot = x_pos_perc * hor_rot_max
#     y_rot = y_pos_perc * vert_rot_max * -1
#
#     tcp_rotation_rpy = [y_rot, x_rot, 0]
#     # tcp_rotation_rvec = convert_rpy(tcp_rotation_rpy)
#     tcp_orient = m3d.Orientation.new_euler(tcp_rotation_rpy, encoding='xyz')
#     position_vec_coords = m3d.Transform(tcp_orient, xyz_coords)
#
#     #oriented_xyz = origin * position_vec_coords
#     #oriented_xyz_coord = oriented_xyz.get_pose_vector()#ICI
#
#     #coordinates = oriented_xyz_coord
#     coordinates = position_vec_coords
#
#     qnear = robot.get_actual_joint_positions()
#     next_pose = coordinates
#     robot.set_realtime_pose(next_pose)
#     print("initialising robot5")
#
#     return prev_robot_pos
def show_frame(frame):
    # Rotate the frame by 180 degrees
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    cv2.imshow('RobotCamera', frame)
    k = cv2.waitKey(6) & 0xff

"""def convert_rpy(angles):

    # This is very stupid:
    # For some reason this doesnt work if exactly  one value = 0
    # the following simply make it a very small value if that happens
    # I do not understand the math behind this well enough to create a better solution
    zeros = 0
    zero_pos = None
    for i,ang in enumerate(angles):
        if ang == 0 :
            zeros += 1
            zero_pos = i
    if zeros == 1:
        #logging.debug("rotation value" + str(zero_pos+1) +"is 0 a small value is added")
        angles[zero_pos] = 1e-6

    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]

    # print ("roll = ", roll)
    # print ("pitch = ", pitch)
    # print ("yaw = ", yaw)
    # print ("")

    for ang in angles:
        # print(ang % np.pi)
        pass

    if roll == pitch == yaw:

        if roll % np.pi == 0:
            rotation_vec = [0, 0, 0]
            return rotation_vec

    yawMatrix = np.matrix([
    [math.cos(yaw), -math.sin(yaw), 0],
    [math.sin(yaw), math.cos(yaw), 0],
    [0, 0, 1]
    ])
    # print("yawmatrix")
    # print(yawMatrix)

    pitchMatrix = np.matrix([
    [math.cos(pitch), 0, math.sin(pitch)],
    [0, 1, 0],
    [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    # print("pitchmatrix")
    # print(pitchMatrix)

    rollMatrix = np.matrix([
    [1, 0, 0],
    [0, math.cos(roll), -math.sin(roll)],
    [0, math.sin(roll), math.cos(roll)]
    ])
    # print("rollmatrix")
    # print(rollMatrix)

    R = yawMatrix * pitchMatrix * rollMatrix
    # print("R")
    # print(R)

    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    # print("theta = ",theta)
    multi = 1 / (2 * math.sin(theta))
    # print("multi = ", multi)


    rx = multi * (R[2, 1] - R[1, 2]) * theta
    ry = multi * (R[0, 2] - R[2, 0]) * theta
    rz = multi * (R[1, 0] - R[0, 1]) * theta

    rotation_vec = [rx,ry,rz]
    # print(rx, ry, rz)
    return rotation_vec
"""
def find_aruco(frame):
    # Convert frame to grayscale

    offset=None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        # Estimate pose for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
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
    else:
        print("can't find marker")    # Display the resulting frame
    return offset
def move_to_aruco(xyoffset):
    #get actual robot pose
    qnear = robot.get_actual_tcp_pose()
    if (xyoffset[0]<10):
        qnear[1]-= xyoffset[0]*m_per_pixel#xcoord
    elif (xyoffset[0]>-10):
        qnear[1]= (qnear[1]-(xyoffset[0]*m_per_pixel))
    elif (xyoffset[0]>2):
        qnear[1] -= 0.001
    elif (xyoffset[0]<-2):
        qnear[1] += 0.001

    if (xyoffset[1]>10):
        qnear[2]= (qnear[2]+(xyoffset[1]*m_per_pixel))
    elif (xyoffset[1]<-10):
        qnear[2]= (qnear[2]+(xyoffset[1]*m_per_pixel))
    if (xyoffset[1]>2):
        qnear[2] += 0.001
    elif (xyoffset[1]<-2):
        qnear[2] -= 0.001
    #move the tcp accordingly
    qnear[0]=-0.4
    robot.set_realtime_pose(qnear)
"""FACE TRACKING aruco ____________________________________________________________________"""

# initialise robot with URBasic
print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)

robot.reset_error()
print("robot initialised")
time.sleep(1)

# Move Robot to the midpoint of the lookplane
# robot.movej(q=robot_startposition, a= ACCELERATION, v= VELOCITY )

robot_position = [0,0]
origin = set_lookorigin()

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised

try:
    print("starting loop")
    while True:
        frame = vs.read()
        xyoffset=find_aruco(frame)
        show_frame(frame)
        if xyoffset is not None:
            move_to_aruco(xyoffset)
        time.sleep(0.05)
        if (xyoffset[0] < 2 and xyoffset[0] > -2 and xyoffset[1] < 2 and xyoffset[1] > -2 and xyoffset != None):
            break
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()
    # Release the capture and close windows

except:
    # Release the capture and close windows
    robot.close()