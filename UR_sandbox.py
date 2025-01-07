"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
Demonstration Video: https://youtu.be/HHb-5dZoPFQ
Explanation Video: https://www.youtube.com/watch?v=9XCNE0BmtUg

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html

"""

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

ACCELERATION = 0.5  # Robot acceleration value
VELOCITY = 0.5  # Robot speed value

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

"""FACE TRACKING LOOP ____________________________________________________________________"""

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
        qnear = robot.get_actual_tcp_pose()
        qnear[1]+=0.01
        print(qnear[1])
        robot.set_realtime_pose(qnear)
        time.sleep(0.3)
        qnear[1]-=0.01
        robot.set_realtime_pose(qnear)
        time.sleep(0.3)
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()

except:
    robot.close()