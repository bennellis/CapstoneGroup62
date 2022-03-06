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
        posList = [ \
[339.857,13.981,285.868,267.119,92.039,66.731,0,1,1],\
[345.411,57.817,351.299,271.991,109.649,73.144,-1,1,1],\
[347.142,53.686,340.947,270.998,103.48,75.395,0.315,1,1],\
[347.05,51.692,336.811,270.946,101.532,75.249,-1,1,1],\
[346.829,48.242,330.236,270.946,98.444,75,-1,1,1],\
[346.704,46.572,327.111,270.949,96.985,74.837,-1,1,1],\
[346.575,45.284,324.792,270.951,95.915,74.751,-1,1,1],\
[346.408,43.196,321.038,270.955,94.25,74.541,-1,1,1],\
[351.488,41.235,320.295,261.335,93.38,78.816,-1,1,1],\
[356.132,41.495,322.294,254.3,95.011,82.443,0.34,1,1],\
[358.773,43.033,324.721,249.352,97.13,84.186,0.48,1,1],\
[4.506,44.685,327.912,238.118,100.224,86.996,-1,1,1],\
[5.012,37.718,315.556,238.408,94.642,90.379,-1,1,1],\
[5.591,27.888,299.696,238.525,89.851,93.922,-1,1,1],\

#[345.653,52.159,335.42,271.795,99.471,75.542,0.34,1,1],\
#[348.025,51.33,334.57,266.849,98.554,77.183,-1,1,1],\
#[350.237,53.316,341.265,262.274,103.28,78.073,-1,1,1],\
#[350.217,50.152,335.506,262.327,100.69,78.337,-1,1,1],\
#[349.998,46.373,328.707,262.457,97.666,78.545,-1,1,1],\
#[350.65,38.065,313.981,262.463,91.379,80.037,-1,1,1],\


[3.867,26.318,295.344,261.563,86.376,94.091,-1,1,1],\
[3.868,26.321,295.307,261.552,357.887,94.088,0.48,1,1],\
[6.901,32.696,262.613,272.712,318.824,83.512,-1,1,1],\
[18.406,37.019,263.868,304.597,309.871,61.392,-1,1,1],\
[18.18,41.712,262.517,301.784,304.772,65.74,-1,1,1],\
[18.068,43.623,262.281,300.941,302.985,67.177,-1,1,1],\
[18.062,44.495,262.203,300.647,302.165,67.714,-1,1,1],\
[18.051,45.311,262.156,300.311,301.467,68.285,-1,1,1],\
[18,47.322,262.154,299.651,299.725,69.441,-1,1,1],\
[17.961,47.818,262.154,299.55,299.302,69.766,0.27,1,1],\
[18.059,43.877,262.252,300.877,302.747,67.364,0.492,1,1],\
[18.054,44.53,262.199,300.63,302.136,67.7,-1,1,1],\
[18.046,45.327,262.156,300.306,301.456,68.291,-1,1,1],\
[18.034,46.081,262.155,300.053,300.762,68.747,-1,1,1],\
[18.011,47.06,262.157,299.784,299.931,69.334,-1,1,1],\
[17.941,48.121,262.168,299.433,299.063,69.891,0.27,1,1],\
[19.123,30.468,268.6,312.245,318.328,50.983,-1,1,1],\
[0.573,29.688,274.923,27.015,42.309,322.172,-1,1,1],\


##[28.32,47.3,272.53,124.948,57.107,241.961,-1,1,1],\
##[18.276,44.782,261.899,90.891,56.97,262.706,-1,1,1],\
##[14.054,44.884,260.57,78.887,58.203,269.302,-1,1,1],\
##[10.861,44.884,259.258,75.269,58.797,271.127,-1,1,1],\


[1.782,46.682,261.715,50.877,61.894,283.797,-1,1,1],\
[3.019,48.307,262.314,52.451,62.72,286.19,-1,1,1],\
[3.287,50.45,267.023,51.845,60.102,287.885,-1,1,1],\
[4.691,53.222,273.25,52.037,56.872,289.596,-1,1,1],\
[5.48,52.466,273.565,52.482,55.637,292.305,-1,1,1],\
[4.929,54.519,277.73,50.961,54.2,294.273,-1,1,1],\
[6.269,53.56,278.16,51.789,52.36,294.592,0.779,1,1],\
[6.338,53.759,277.911,52.042,52.856,294.051,-1,1,1],\
[6.271,51.906,277.99,51.115,51.437,295.672,-1,1,1],\
[6.214,50.84,278.103,50.462,50.558,296.61,-1,1,1],\
[6.078,49.354,278.403,49.364,49.166,298.264,-1,1,1],\
[4.99,41.105,283.964,37.927,40.259,313.325,-1,1,1],\

            
[348.432,54.379,277.506,348.85,97.47,307.65,0.78,1,1],\
[357.114,52.698,273.62,355.166,91.4,306.2,-1,1,1],\
[357.348,59.221,287.163,355.573,91.807,313.3,-1,1,1],\
[357.327,58.074,284.815,355.53,91.76,312.05,-1,1,1],\
[357.558,55.624,279.347,358.183,89.229,309.066,-1,1,1],\
[357.89,53.724,275.106,0.644,87.287,306.656,-1,1,1],\
[358.854,51.765,268.477,6.189,85.154,301.831,-1,1,1],\
[0.425,48.711,261.528,12.717,82.22,297.269,-1,1,1],\
[2.262,46.285,256.078,19.257,80.045,293.347,-1,1,1],\
[5.09,43.881,250.626,27.77,77.956,288.85,-1,1,1],\
[10.161,41.225,244.793,40.93,75.876,283.192,-1,1,1],\
[15.274,39.584,241.511,52.84,74.468,278.866,-1,1,1],\
[13.839,38.73,242.65,50.933,73.124,280.718,-1,1,1],\
[20.905,37.167,240.398,66.715,71.1,275.168,-1,1,1],\
[25.157,36.621,240.184,76.01,69.93,271.935,-1,1,1],\
[31.037,35.729,241.188,89.155,67.278,267.093,-1,1,1],\
[29.271,36.472,242.598,87.21,66.643,267.959,-1,1,1],\
[36.967,36.621,246.458,106.413,64.441,259.122,-1,1,1],\
[36.944,35.899,246.483,106.483,63.7,258.903,-1,1,1],\
[40.964,36.522,250.362,118.24,63.021,252.313,-1,1,1],\
[44.273,36.668,255.476,130.7,61.351,243.88,-1,1,1],\
[43.112,37.527,257.331,129.846,60.116,243.826,-1,1,1],\
[45.54,39.47,261.537,133.206,59.547,243.181,-1,1,1],\
[48.607,41.283,266.599,137.711,58.787,238.707,-1,1,1],\
[48.063,40.215,265.48,137.141,58.411,239.05,-1,1,1],\
[53.897,35.674,255.984,140.784,64.694,240.373,-1,1,1],\
[54.092,39.809,255.761,139.476,67.597,243.911,-1,1,1],\
[53.051,25.52,258.697,145.93,56.477,228.898,-1,1,1],\
[48.351,68.869,330.538,43.9,12.107,317.239,-1,1,1],\
[54.885,82.522,340.902,100.773,11.517,261.16,-1,1,1],\
]
        for a in range(len(posList)):
            while(stage.value!=posList[a][7]):
                lockNext.acquire()
                print(stage.value)
                print(posList[a][7])
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
    
    args = utilities.parseConnectionArguments(ip = "192.168.1.10")

    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        gripperGoTo(0.01, router, base)


        #input position list for arm 2 below. 7th array index is optional. It is used to control gripper. 1.0 is fully closed, 0.0 is fully open. if left blank, will not move gripper.
        posList = [[100.047,31.613,336.03,88.836,238.112,12,0,1,1],\
[100.045,47.843,262.207,88.108,328.172,14.226,0.874,1,1],\
[100.045,28.803,328.041,88.942,243.296,12.082,-1,1,1],\
[56.049,28.961,324.99,91.284,246.413,280.292,-1,1,1],\
[56.24,23.05,297.83,91.198,267.667,280.04,0,1,1],\
[49.52,24.901,302.664,91.484,264.498,273.372,1,1,1],\
[49.52,24.443,294.978,91.483,271.741,273.145,-1,1,1],\
[54.958,24.684,295.309,91.228,271.796,278.56,-1,1,1],\
[54.961,24.897,289.915,91.249,277.404,278.548,-1,1,1],\
[54.959,25.4,304.878,91.24,262.921,278.816,-1,1,1],\
[63.538,26.188,305.918,90.831,262.765,287.405,-1,1,1],\
[63.538,23.365,294.987,90.827,272.915,287.237,-1,1,1],\
[57.177,25.028,294.555,91.153,272.908,280.898,-1,1,1],\
[57.189,25.245,289.793,91.163,277.96,280.748,-1,1,1],\
[55.758,24.991,297.4,91.211,270.038,279.468,-1,1,1],\
[55.758,25.239,289.367,91.208,278.329,279.354,0,1,1],\
[55.758,25.239,289.367,91.208,278.329,279.354,1,1,1],\
[55.758,28.76,320.1,91.249,251.129,279.87,0,1,1],\
[57.824,18.897,305.05,89.602,257.918,6.873,-1,1,1],\
[57.825,17.494,296.95,89.606,264.63,6.895,1,1,1],\
[59.015,20.438,300.988,89.474,263.503,8.126,-1,1,1],\
[59.023,19.667,294.329,89.487,269.415,8.156,-1,1,1],\
[58.506,27.344,305.533,89.541,265.908,7.553,-1,1,1],\
[58.505,31.454,324.493,89.508,250.988,7.458,-1,1,1],\
[65.466,342.605,255.4,79.913,302.759,17.086,0.86,1,1],\
[69.508,345.158,240.655,63.681,319.527,35.677,-1,1,1],\
[68.022,348.257,243.077,64.501,320.695,34.147,-1,1,1],\
[67.866,349.349,243.702,64.369,321.214,34.204,-1,1,1],\
[67,351.148,248.134,69.462,316.738,29.019,-1,1,1],\
[67.32,352.528,249.215,69.074,316.918,29.645,-1,1,1],\
[64.333,356.145,257.788,78.42,308.537,19.73,-1,1,1],\
[62.623,357.728,259.032,79.484,309.162,17.704,-1,1,1],\
[59.846,2.08,267.639,85.723,299.942,11.23,-1,1,1],\
[59.2,3.751,268.47,85.991,300.818,10.566,-1,1,1],\
[58.942,4.709,269.616,86.154,300.683,10.264,-1,1,1],\
[57.226,11.846,281.762,89.337,286.949,7.1,-1,1,1],\
[58.818,15.988,287.63,86.823,279.979,9.055,-1,1,1],\
[58.81,16.685,287.468,86.816,280.864,9.073,-1,1,1],\
[59.168,17.404,287.28,86.694,281.717,9.504,-1,1,1],\
[57.256,23.104,296.903,89.844,271.971,7.175,-1,1,1],\
[56.033,30.147,308.274,91.835,260.763,6.198,-1,1,1],\
[55.739,31.152,308.833,91.835,261.209,5.829,-1,1,1],\
[55.827,31.776,308.792,91.831,261.854,5.963,-1,1,1],\
[55.438,37.977,318.938,92.56,252.498,5.813,-1,1,1],\
[54.624,43.2,327.875,94.205,244.921,5.807,-1,1,1],\
[53.945,43.626,327.639,94.051,245.535,5.074,-1,1,1],\
[53.941,43.551,327.388,94.041,245.739,5,0.8977,1,1],\
[53.94,43.088,324.971,93.987,247.677,4.905,-1,1,1],\
[54.044,39.045,318.252,93.64,253.494,4.71,-1,1,1],\
[54.224,33.839,309.771,93.221,261.33,4.653,-1,1,1],\
[54.512,27.846,300.176,92.745,270.765,4.682,-1,1,1],\
[54.513,27.797,299.406,92.744,271.555,4.666,0.82,1,1],\
[54.649,25.287,295.354,92.524,275.73,4.712,-1,1,1],\
[54.811,22.796,291.323,92.292,280.019,4.891,-1,1,1],\
[54.807,22.799,294.94,92.283,276.349,4.991,1,1,1],\
[57.528,23.333,295.6,91.87,276.327,7.736,-1,1,1],\
[57.193,29.402,303.411,92.675,268.786,7.482,-1,1,1],\
[57.194,29.257,300.32,92.665,271.807,7.306,-1,1,1],\
[57.193,29.682,306.874,92.676,265.678,7.566,-1,1,1],\
[52.089,29.985,308.052,92.931,264.484,2.53,-1,1,1],\
[52.088,29.528,302.379,92.92,269.75,2.303,-1,1,1],\
[52.086,29.481,301.378,92.916,270.7,2.274,-1,1,1],\
[52.087,30.539,312.047,92.945,261.117,2.73,-1,1,1],\
]


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
    #a2 = Process(target=arm2, args = (stage,next_stage_1, next_stage_2, lockNext))
    a1.start()
    #a2.start()
    a1.join()
    #a2.join()

if __name__ == "__main__":
    exit(main())
