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

    startTime = time.time()



    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            dif = gripper_measure.finger[0].value - position
            now = time.time()
            if (dif < 0.01) & (dif > - 0.01):
                break
            if(now-startTime > 1):
                if(dif < 0.05) & (dif > -0.05):
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


        success &= example_move_to_home_position(base) #home position

         #input position list for arm 2 below. 7th array index is used to control gripper. 1.0 is fully closed, 0.0 is fully open. if left as -1, will not change gripper. 8th and 9th array indexes are for
        #synchronization between the steps. 8th index is step required by both arms before execution, and 9th index is step to proceed to for that arm after executing that position.
        posList = [ \
[339.857,13.981,285.868,267.119,92.039,66.731,0,1,1],\
[345.411,57.817,351.299,271.991,109.649,73.144,-1,1,1],\
[346.658,53.41,339.26,270.958,102.2,75,0.315,1,1],\
[347.05,51.692,336.811,270.946,101.532,75.249,-1,1,1],\
[346.829,48.242,330.236,270.946,98.444,75,-1,1,1],\
[346.704,46.572,327.111,270.949,96.985,74.837,-1,1,1],\
[346.575,45.284,324.792,270.951,95.915,74.751,-1,1,1],\
[346.408,43.196,321.038,270.955,94.25,74.541,-1,1,1],\
[351.488,41.235,320.295,261.335,93.38,78.816,-1,1,1],\
[356.132,41.495,322.294,254.3,95.011,82.443,0.34,1,1],\
[358.773,43.033,324.721,249.352,97.13,84.186,0.4,1,1],\
[4.506,44.685,327.912,238.118,100.224,86.996,-1,1,1],\
[5.012,37.718,315.556,238.408,94.642,90.379,-1,1,1],\
[5.591,27.888,299.696,238.525,89.851,93.922,-1,1,1],\



[3.867,26.318,295.344,261.563,86.376,94.091,0.48,1,1],\
[3.868,26.321,295.307,261.552,357.887,94.088,-1,1,1],\
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




[1.782,46.682,261.715,50.877,61.894,283.797,-1,1,1],\
[3.019,48.307,262.314,52.451,62.72,286.19,-1,1,1],\
[3.287,50.45,267.023,51.845,60.102,287.885,-1,1,1],\
[4.691,53.222,273.25,52.037,56.872,289.596,-1,1,1],\
[5.48,52.466,273.565,52.482,55.637,292.305,-1,1,1],\
[5.849,52.607,275.002,52.224,54.222,293.2,-1,1,1],\
[7.196,53.908,277.377,53.542,53.301,292.63,0.78,1,1],\
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
[54.395,77.835,344.616,147.31,7.963,211.047,-1,1,1],\
[54.395,79.887,346.114,144.675,8.246,213.714,-1,1,1],\
[55.9,83.208,347.582,141.465,10.924,217.091,0,1,1],\
[57.369,72.806,323.982,117.364,21.902,242.473,-1,1,2],\
[60.184,49.348,286.833,112.337,35.499,249.514,-1,2,2],\
[3.149,348.471,224.924,92.551,36.172,267.888,-1,2,2],\

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

        success &= example_move_to_home_position(base) # go to home position




        #input position list for arm 2 below. 7th array index is used to control gripper. 1.0 is fully closed, 0.0 is fully open. if left as -1, will not change gripper. 8th and 9th array indexes are for
        #synchronization between the steps. 8th index is step required by both arms before execution, and 9th index is step to proceed to for that arm after executing that position.
        posList = [
[98.277,28.822,319.456,88.489,251.482,11.715,0,1,1],\
[100.222,49.636,266.163,87.234,325.225,13.89,0.8,1,1],\
[97.521,49.139,265.575,87.456,325.686,13.625,-1,1,1],\
[97.522,26.357,311.787,88.567,256.666,11.106,-1,1,2],\
[56.506,28.136,317.414,88.524,252.9,282.31,-1,2,2],\
[56.509,25.142,300.836,88.573,266.388,282.7,0.5,2,2],\
[56.509,25.142,300.836,88.573,266.388,282.7,1,2,2],\
[55.533,25.051,284.741,88.796,282.4,274.046,-1,2,2],\
[56.235,25.184,302.313,88.586,265.012,282.323,-1,2,2],\
[47.593,24.841,299.78,88.964,267.631,273.822,-1,2,2],\
[49.667,24.114,293.426,88.8,272.953,275.976,-1,2,2],\
[54.442,25.352,295.227,88.652,272.298,280.775,-1,2,2],\
[54.45,25.66,289.247,88.652,278.584,280.876,-1,2,2],\
[54.446,25.867,303.134,88.65,264.913,280.58,-1,2,2],\
[63.722,25.5,302.494,88.319,265,289.754,-1,2,2],\
[63.725,25.015,294.445,88.324,272.474,289.99,-1,2,2],\
[58.29,23.552,292.385,88.523,272.288,284.6,-1,2,2],\
[58.291,24,286.232,88.46,279.866,284.758,-1,2,2],\
[58.291,26.064,312.795,88.456,255.347,284.096,-1,2,2],\
[56.411,15.864,301.927,89.895,256.022,9.556,-1,2,2],\
[56.741,14.297,286.07,89.894,270.346,8.669,-1,2,2],\
[55.568,25.048,300.865,89.934,266.221,7.445,-1,2,2],\
[55.57,24.75,294.222,89.915,272.618,7.456,-1,2,2],\
[54.715,30.181,303.784,90,268.53,6.7,0.78,2,2],\

[54.006,38.793,318.009,91.18,255.2,8.633,-1,2,2],\
[54.008,38.641,316.725,91.177,256.31,8.6,-1,2,2],\
[53.557,46.611,330.366,92.095,244.726,8.6,-1,2,2],\
[53.559,46.23,328.728,92.074,245.99,8.519,-1,2,2],\
[53.315,52.477,339.805,92.744,237.498,8.758,-1,2,2],\
[53.136,58.082,350.139,93.46,229.991,9.2,-1,2,2],\
[52.999,61.726,355.31,94.399,224.679,9.763,-1,2,2],\

[53.136,58.082,350.139,93.46,229.991,9.2,0.746,2,2],\

[53.17,58.796,354.256,93.731,226.483,9.557,-1,2,2],\
[53.232,58.212,354.977,93.786,225.323,9.735,-1,2,2],\
[53.214,58.775,355.494,93.978,224.446,9.794,-1,2,2],\
[53.194,59.297,355.2,93.981,225.093,9.738,-1,2,2],\
[53.186,59.453,354.905,93.839,225.935,9.587,-1,2,2],\
[53.185,59.453,354.904,93.837,229.368,9.587,-1,2,2],\
[53.186,59.452,354.906,93.834,232.268,9.587,-1,2,2],\
[53.186,59.452,354.906,93.834,233.952,9.587,-1,2,2],\
[53.338,58.104,358.541,94.191,225.677,10.219,-1,2,2],\
[52.785,33.901,314.576,88.801,258.171,358.99,1,2,2],\
[52.784,32.862,305.722,88.807,266.014,359.162,-1,2,2],\
[52.538,33.522,310.028,88.813,262.429,358.899,-1,2,2],\
[58.806,33.987,310.7,88.949,262,5.2,-1,2,2],\
[58.804,33.596,306.733,88.948,265.663,5.208,-1,2,2],\
[58.425,31.761,323.402,88.815,247.1,100.328,0,2,2],\
[65.84,357.142,232.335,125.591,40.182,65.186,-1,2,2],\
[70.356,21.924,218.76,117.673,74.52,85.971,-1,2,2],\
[70.518,38.583,218.836,116.737,89.32,93.562,-1,2,2],\
[69.254,38.236,223.076,116.473,85.258,87.69,0.7356,2,2],\
[69.102,24.882,222.855,117.465,73.424,81.622,-1,2,2],\
[323.87,44.694,253.488,332.665,104.848,114.5,-1,2,2],\
[323.325,90.655,277.734,328.567,90.816,95.301,0.415,2,2],\
[323.9,38.452,254.396,332.716,105.072,120.8,-1,2,2],\



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
    next_stage_1 = Value('i', 2)
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
