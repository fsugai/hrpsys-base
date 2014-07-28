#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    import OpenHRP
except:
    print "import without hrpsys"
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

def getRTCList ():
    return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['kf', "KalmanFilter"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['ic', "ImpedanceController"],
            ]

def init ():
    global hcf
    hcf = HrpsysConfigurator()
    hcf.getRTCList = getRTCList
    hcf.init ("SampleRobot(Robot)0")

def demo():
    init()
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()

    # 1. get
    hcf.ic_svc.getImpedanceControllerParam("rhsensor")
    ret1=hcf.ic_svc.getImpedanceControllerParam("rhsensor")
    if ret1[0]:
        print "getImpedanceControllerParam => OK"
    # 2. set and start
    ret1[1].base_name="CHEST"
    ret1[1].target_name="RARM_WRIST_R"
    hcf.ic_svc.setImpedanceControllerParam(ret1[1])
    ret2=hcf.ic_svc.setImpedanceControllerParam(ret1[1])
    ret3=hcf.ic_svc.getImpedanceControllerParam("rhsensor")
    if ret2 and ret1[1].base_name == ret3[1].base_name and ret1[1].target_name == ret3[1].target_name:
        print "setImpedanceControllerParam => OK"
    # 3. stop
    ret4=hcf.ic_svc.deleteImpedanceController("rhsensor")
    if ret4:
        print "deleteImpedanceController => OK"