#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading
from multiprocessing import Process

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def gripperGoTo(perc, router, base):
    if perc > 1:
        perc = 1
    if perc < 0.01:
        perc = 0.01
            
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()


    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = perc
    finger.finger_identifier = 1
    finger.value = position
    print("Going to position {:0.2f}...".format(perc))
    base.SendGripperCommand(gripper_command)
    time.sleep(3)


def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def angular_waypoints(base, posList, router):
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # go to posList

    #
    
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = posList[joint_id]
    


    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    print("Executing action")
    base.ExecuteAction(action)
    

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
        if(len(posList)==7):
            gripperGoTo(posList[6], router, base)
    else:
        print("Timeout on action notification wait")
    return finished


def example_go_to_pos(base, base_cyclic, arr):
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()
    kTheta_x = arr[3]
    kTheta_y = arr[4]
    kTheta_z = arr[5]
    cartesian_pose = action.reach_pose.target_pose
    #cartesian_pose.x = feedback.base.tool_pose_x          # (meters)
    #cartesian_pose.y = feedback.base.tool_pose_y - 0.1    # (meters)
    #cartesian_pose.z = feedback.base.tool_pose_z - 0.2    # (meters)
    #cartesian_pose.theta_x = feedback.base.tool_pose_theta_x # (degrees)
    #cartesian_pose.theta_y = feedback.base.tool_pose_theta_y # (degrees)
    #cartesian_pose.theta_z = feedback.base.tool_pose_theta_z # (degrees)
    cartesian_pose.x = arr[0]          # (meters)
    cartesian_pose.y = arr[1]    # (meters)
    cartesian_pose.z = arr[2]    # (meters)
    cartesian_pose.theta_x = kTheta_x # (degrees)
    cartesian_pose.theta_y = kTheta_y # (degrees)
    cartesian_pose.theta_z = kTheta_z # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def arm1():
    
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities
    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True
        arr = [0.1,.2,.2, 180, -1, 90]
        #success&= example_go_to_pos(base, base_cyclic, arr)

        success &= example_move_to_home_position(base)
        #success &= example_cartesian_action_movement(base, base_cyclic)
        #success &= example_angular_action_movement(base)
        #input position list for arm 2 below. 7th array index is optional. It is used to control gripper. 1.0 is fully closed, 0.0 is fully open. if left blank, will not move gripper.
        posList = [[58.258,	105.954,	333.679,	104.577,	40.673,	83.145, 0.64]]
        for a in range(len(posList)):
            success &= angular_waypoints(base, posList[a], router)

        #example = GripperCommandExample(router)
        #example.goToPos(posList[6])
        #success &= example_move_to_home_position(base)
        #success &= example_cartesian_trajectory_movement(base, base_cyclic)
        #success &= example_angular_trajectory_movement(base)

        return 0 if success else 1

def arm2():
    import utilities
    
    args = utilities.parseConnectionArguments(ip = "192.168.2.10")

    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        gripperGoTo(0.01, router, base)


        #input position list for arm 2 below. 7th array index is optional. It is used to control gripper. 1.0 is fully closed, 0.0 is fully open. if left blank, will not move gripper.
        posList = [[97.106,	57.617,	284.744,	88.349,	317.377,	141.103, 0.795], \
[53.121,	44.163,	281.918,	92.749,	306.484,	94.1], \
[53.121,	44.163,	281.918,	92.749,	306.484,	94.1,	0], \
[56.505,	48.035,	279.751,	86.761,	312.73,	97.607], \
[56.505,	48.035,	279.751,	86.761,	312.73,	97.607,	0.71344], \
[56.504,	52.665,	279.055,	86.392,	318.01,	98.06], \
[56.504,	41.212,	283.466,	87.184,	302.146,	96.923], \
[56.871,	48.01,	299.328,	87.805,	293.035,	9.902,	0.81], \
[57.078,	48.71,	293.776,	87.652,	299.3,	10.38], \
]
        for a in range(len(posList)):
            success &= angular_waypoints(base, posList[a], router)

        return 0 if success else 1
def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    a1 = Process(target=arm1)
    a2 = Process(target=arm2)
    a1.start()
    a2.start()
    a1.join()
    a2.join()

if __name__ == "__main__":
    exit(main())
