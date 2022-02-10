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
from multiprocessing import Process, Value, Lock

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
    gripper_request = Base_pb2.GripperRequest()
    finger = gripper_command.gripper.finger.add()


    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    gripper_request.mode = Base_pb2.GRIPPER_POSITION

    position = perc
    finger.finger_identifier = 1
    finger.value = position
    print("Going to position {:0.2f}...".format(perc))
    base.SendGripperCommand(gripper_command)

    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            dif = gripper_measure.finger[0].value - position
            if (dif < 0.01) & (dif > - 0.01):
                break
        else: # Else, no finger present in answer, end loop
            break
    time.sleep(0.1)
    


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
        if(posList[6] != -1):
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

def arm1(stage,next_stage_1, next_stage_2, lockNext):
    
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
        posList = [[347.82,	54.416,	277.96,	348.14,	99.69,	129.178,	0.80602, 1, 2], \
[358.34,	54.015,	275.507,	355.766,	92.43,	128.207, -1, 2, 2], \
[358.583,	61.386,	291.016,	356.22,	92.928,	136.312, -1, 2, 2], \
[359.672,	53.735,	274.388,	4.682,	85.34,	127.161, -1, 2, 2],\
[0.792,	50.714,	267.752,	9.92,	82.32,	123.048, -1, 2, 2],\
[4.118,	45.921,	257.384,	21.645,	77.676,	115.663, -1, 2, 2],	\
[8.74,	42.413,	250.077,	34.639,	74.294,	109.163, -1, 2, 2],	\
[18.34,	38.674,	243.361,	57.72,	70.355,	99.83, -1, 2, 2],	\
[30.133,	36.197,	242.651,	84.33,	65.078,	89.848, -1, 2, 2],	\
[44.12,	36.976,	255.742,	128.82,	61.698,	67.668, -1, 2, 2],	\
[45.352,	38.638,	257.728,	130.209,	62.046,	66.685, -1, 2, 2],\
[48.848,	41.161,	264.742,	135.503,	60.292,	61.258, -1, 2, 2], \
[49.287,	37.91,	264.232,	137.096,	58.556,	58.684, -1, 2, 2], \
[53.762,	34.298,	256.951,	139.901,	63.344,	59.475, -1, 2, 2], \
[54.218,	40.363,	256.442,	138.043,	67.965,	64.74, -1, 2, 2],	\
[53.994,	35.606,	256.663,	139.524,	64.47,	60.73, -1, 2, 2], \
[52.671,	23.631,	260.932,	146.266,	53.824,	46.247,	0.75, 2, 2],\
[70.319,	34.873,	266.425,	149.025,	62.313,	50.364, -1, 2, 2], \
[52.187,	94.168,	2.272,	80.891,	359.89,	94.21, -1, 2, 3], \
]
        for a in range(len(posList)):
            while(stage.value!=posList[a][7]):
                lockNext.acquire()
                print(stage.value)
                if(next_stage_1.value==next_stage_2.value & next_stage_1.value!=stage.value):
                    stage.value = next_stage_1.value;
                lockNext.release()
                time.sleep(0.2)

 
            success &= angular_waypoints(base, posList[a], router)
            lockNext.acquire()
            next_stage_1.value = posList[a][8]
            lockNext.release()


        #example = GripperCommandExample(router)
        #example.goToPos(posList[6])
        #success &= example_move_to_home_position(base)
        #success &= example_cartesian_trajectory_movement(base, base_cyclic)
        #success &= example_angular_trajectory_movement(base)

        return 0 if success else 1

def arm2(stage,next_stage_1, next_stage_2, lockNext):
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
        posList = [[100.303,	26.532,	308.35,	90.975,	258.818,	13.896,	0, 2, 2],\
[99.132,	50.892,	267.378,	91.6,	324.319,	11.363,	0.72246, 2, 2],\
[98.358,	25.616,	306.351,	90.995,	259.881,	12.058, -1, 2, 3],\
[55.867,	24.694,	313.488,	91.835,	254.441,	283.211,	0, 3, 3],\
[50.662,	22.938,	301.725,	92.073,	264.234,	277.766,	1, 3, 3],\
[50.661,	22.344,	292.15,	92.066,	273.251,	277.404, -1, 3, 3],\
[57.044,	20.989,	290.152,	91.679,	274.127,	283.82, -1, 3, 3],	\
[57.051,	21.43,	284.381,	91.747,	280.274,	283.637, -1, 3, 3],	\
[54.214,	21.98,	285.043,	92.002,	280.053,	280.666, -1, 3, 3],	\
[55.158,	23.995,	303.573,	91.825,	263.6,	282.201, -1, 3, 3],	\
[65.016,	22.491,	299.497,	91.204,	266.446,	291.934, -1, 3, 3],	\
[65.016,	22.124,	291.643,	91.206,	273.917,	291.751, -1, 3, 3],	\
[59.485,	22.805,	292.741,	91.568,	273.366,	286.275, -1, 3, 3],	\
[59.686,	23.579,	287.584,	91.561,	279.306,	286.298, -1, 3, 3],	\
[62.189,	22.99,	286.68,	91.366,	279.633,	288.866, -1, 3, 3],	\
[62.231,	26.276,	317.238,	91.43,	252.419,	289.525, -1, 3, 3],	\
[58.309,	39.65,	336.815,	95.053,	228.344,	13.813, -1, 3, 3],	\
[56.685,	34.682,	314.724,	93.709,	245.326,	10.488, -1, 3, 3],	\
[57.098,	49.427,	340.428,	94.248,	234.397,	11.809, -1, 3, 3],	\
[56.977,	50.26,	356.323,	95.355,	219.5,	13.336, -1, 3, 3],	\
[58.19,	35.285,	321.966,	94.377,	238.822,	12.659,	0.78913, 3, 3],\
[57.952,	45.488,	336.643,	94.488,	234.319,	12.824, -1, 3, 3],	\
[56.678,	48.526,	340.881,	94.207,	233.017,	11.5, -1, 3, 3],	\
[56.677,	46.592,	334.124,	93.974,	237.856,	11.064, -1, 3, 3],	\
[56.556,	46.338,	335.812,	94,	235.861,	11.09,	0.8191, 3, 3],\
[56.295,	48.091,	334.764,	93.811,	238.648,	10.505, -1, 3, 3],	\
[57.676,	44.28,	326.479,	93.991,	243.309,	11.774, -1, 3, 3],	\
[57.582,	53.218,	343.315,	94.3,	235.312,	12.247, -1, 3, 3],	\
[56.713,	50.823,	336.102,	93.871,	240.097,	10.88, -1, 3, 3],	\
[49.244,	35.391,	326.348,	91.827,	234.086,	2.88, -1, 3, 3]]	


        for a in range(len(posList)):

            while(stage.value!=posList[a][7]):
                lockNext.acquire()
                if(next_stage_1.value==next_stage_2.value & next_stage_1.value!=stage.value):
                    stage.value = next_stage_1.value;
                lockNext.release()
                time.sleep(0.2)

            success &= angular_waypoints(base, posList[a], router)

            lockNext.acquire()
            next_stage_2.value = posList[a][8]
            lockNext.release()

        return 0 if success else 1
def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    stage = Value('i', 1)
    next_stage_1 = Value('i', 1)
    next_stage_2 = Value('i', 2)
    lockCurrent = Lock()
    lockNext = Lock()
    a1 = Process(target=arm1, args = (stage,next_stage_1, next_stage_2, lockNext))
    a2 = Process(target=arm2, args = (stage,next_stage_1, next_stage_2, lockNext))
    a1.start()
    a2.start()
    a1.join()
    a2.join()

if __name__ == "__main__":
    exit(main())
