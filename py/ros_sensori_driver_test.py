#!/usr/bin/python

from lib.sensorimotor import Sensorimotor
from time import sleep
import rospy
import rospkg
import roslib
from std_msgs.msg import Float32MultiArray
from rosparam import upload_params
from yaml import load


class ROS_Sensorimotor:
    def __init__(self, name, motors):
        print("Starting ROS Sensorimotor Driver "+"[" + name +"]")
        self.name = name
        pkg_path = rospkg.RosPack().get_path('sensoriboard_ros')
        self.config = self.load_params_from_yaml(pkg_path+'/py/motor_param_test.yaml')['sensori'][self.name]
        self.rawPosRad = 0
        self.cmdPosRad = 0
        self.calibPos = 0
        motors.set_pos_ctrl_params(self.config['motor_id'], Kp = self.config['kp'], Ki = self.config['ki'], Kd = self.config['kd'], deadband = self.config['deadband'], pulse_threshold = self.config['pulse_threshold'])

        #print(self.config['calib_0']+self.config['calib_180'])
    #randian to [-1, 1]

    def load_params_from_yaml(self, path):
        f = open(path, 'r')
        yamlfile = load(f)
        return yamlfile
    #sensorival
    def radToSensori(self, radVal):
        #1. rad / 3.14159  = percentage
        #2. percentage * 2 or range = [0-2]
        #3. [0-2] - 1 = [-1 - 1]
        senVal = radVal / self.config['act_rad'] * (self.config['act_max'] - self.config['act_min'])
        senVal = self.unnormSensori(senVal)

        return senVal

    #rad value
    def sensoriToRad(self, sensoriVal):
        # [-1 1] +1 = [0 2]
        # /2 or range = percentage
        # 3.14159 or actual degree * percentage
        radVal = (self.normSensori(sensoriVal))/(self.config['act_max'] - self.config['act_min']) * self.config['act_rad']
        return radVal

    #return 0 to max_range
    def normSensori(self, rawSensoriVal):
        # make it start from 0 to max range
        normSensoriVal = rawSensoriVal + abs(self.config['act_min'])
        return normSensoriVal

    #return act_min to act_max
    def unnormSensori(self, normSensoriVal):
        rawSensoriVal = normSensoriVal - abs(self.config['act_min'])
        return rawSensoriVal

    # decides center of the head, center would be calibRange/2
    def calibRadbyMid(self, rawRad, calibRange):
        sensoriMid = self.config['calib_mid']
        radMid = self.sensoriToRad(sensoriMid)
        calibRadMin = radMid - abs(calibRange)/2
        calibRad = rawRad + calibRadMin
        return calibRad
    # reads uncalib radian change it to calibrated value

    # goes back to original scale
    def unCalibRadbyMid(self, calibRad, calibRange):
        sensoriMid = self.config['calib_mid']
        radMid = self.sensoriToRad(sensoriMid)
        calibRadMin = radMid - abs(calibRange)/2
        rawRad = calibRad - calibRadMin
        return rawRad

    #Calib input to sensori value
    #HeadAction [-+RAD] -->[+-RAD] --> Sensori [+- 1]
    def calibHead (self, input_calib, calibRange):
        mid_sensori = self.config['calib_mid']
        mid_uncalib_rad = self.sensoriToRad(mid_sensori)
        mid_calib_rad = self.changeScale(mid_uncalib_rad)
        goal_calib = input_calib + mid_calib_rad
        goal_uncalib_rad = self.changeScale(goal_calib)
        goal_sensori = self.radToSensori(goal_uncalib_rad)
        return goal_sensori


    #Sensori [+- 1] --> [+- RAD] --> Pub [-+ RAD]
    #return calib
    def uncalibHead(self, input_sensori, calibRange):
        mid_sensori = self.config['calib_mid']
        mid_uncalib_rad = self.sensoriToRad(mid_sensori)
        mid_calib_rad = self.changeScale(mid_uncalib_rad)
        input_rad  = self.sensoriToRad(input_sensori)
        input_calib_rad = self.changeScale(input_rad)
        goal_calib = input_calib_rad - mid_calib_rad
        return goal_calib

    def calibRadbyMid2(self, rawRad, calibRange):
        sensoriMid = self.config['calib_mid']
        radMid = self.changeScale(self.sensoriToRad(sensoriMid))
        radMid = (self.sensoriToRad(sensoriMid))

        calibRad = rawRad + radMid
        return calibRad

    def unCalibRadbyMid2(self, rawRad, calibRange):
        sensoriMid = self.config['calib_mid']
        radMid =  (self.sensoriToRad(sensoriMid))
        calibRadMin = radMid - abs(calibRange)
        calibRad = rawRad - radMid
        return calibRad

    #Changes scale of the radian
    #30 degrees can be 150 degree with 180 range
    def changeScale(self, rad):
        range_rad = self.config['act_rad']
        changedRad = range_rad - rad
        return changedRad


    #getPos
    #calError
    #setPos
    #Radto [-1-1]
    #[-1, 1] to []
    #check limit
    #

pan_global = 0
tilt_global = 0
# 45 degree to radian
degree_45 =  0.785398

pkg_path = rospkg.RosPack().get_path('sensoriboard_ros')
class ROS_Sensorimotors:
    def __init__(self):
        #Sensorimotor related Init
        print "Initializing Sensorimotors"
        self.motors = Sensorimotor(number_of_motors= 2, verbose=False)
        self.motors.set_voltage_limit([0.18, 0.18])
        self.motors.start()
        self.motors.set_pos_ctrl_params(0, Kp = 0.9, Ki = 1.0, Kd = 0.008, deadband = 0.04, pulse_threshold = 0.10)
        self.motors.set_pos_ctrl_params(1, Kp = 0.9, Ki = 1.0, Kd = 0.008, deadband = 0.04, pulse_threshold = 0.10)


        self.pan_motor= ROS_Sensorimotor("pan", self.motors)
        self.tilt_motor= ROS_Sensorimotor("tilt", self.motors)

        #ROS related inits
        self.jointPub = rospy.Publisher("joint/poses", Float32MultiArray, queue_size = 10)


        self.unCalibjointPub = rospy.Publisher("/joint/uncalib/poses", Float32MultiArray, queue_size = 10)

        rospy.Subscriber("joint/cmd", Float32MultiArray, self.cmdCallback, queue_size = 10)
        rospy.Timer(rospy.Duration(0.1), self.calibjointPubCallback)
        rospy.Timer(rospy.Duration(0.1), self.unCalibjointPubCallback)
        print "Waiting for Commands"

    #Gets commands
    def cmdCallback(self, cmd):
        print "taking commands from user"
        #a = test.radToSensori(test.calibRadbyMid(0.785398,1.57 ))
        pan = self.pan_motor.calibHead(cmd.data[0], 1.57)
        tilt = self.tilt_motor.calibHead(cmd.data[1], 1.57)
        print(pan )
        print(tilt )

        self.motors.set_position([pan, tilt])

    #Publish joints
    def calibjointPubCallback(self,event):
        #print "publishing"
        joints = Float32MultiArray()
        values = self.motors.get_position()
        values_origin = [values[0], values[1]] # pan, tilt
        #print(test.unCalibRadbyMid(test.sensoriToRad(values[0]),1.57))
        #pan = self.pan_motor.unCalibRadbyMid(2.87979-self.pan_motor.sensoriToRad(values[0]), 1.57)
        pan = self.pan_motor.uncalibHead(values[0], 1.57)
        tilt = self.tilt_motor.uncalibHead(values[1], 1.57)

        #tilt = self.tilt_motor.unCalibRadbyMid(2.87979-self.tilt_motor.sensoriToRad(values[1]), 1.57)
        joints.data = [ pan,  tilt, values_origin[0], values_origin[1]]
        print joints
        self.jointPub.publish(joints)

    def unCalibjointPubCallback(self,event):
        #print "publishing"
        joints = Float32MultiArray()
        values = self.motors.get_position()
        values_origin = [values[0], values[1]] # pan, tilt

        #print(test.unCalibRadbyMid(test.sensoriToRad(values[0]),1.57))

        pan = self.pan_motor.sensoriToRad(values[0])
        tilt = self.tilt_motor.sensoriToRad(values[1])
        joints.data = [pan, tilt, values_origin[0], values_origin[1]]
        self.unCalibjointPub.publish(joints)


def main():
    rospy.init_node('sensori_driver', anonymous=True)
    motorDriver = ROS_Sensorimotors()
    N = motorDriver.motors.ping()
    #motorDriver.motors.set_position([0.0, 0.0])
    print("Found {0} sensorimotors.".format(N))
    #motorDriver.motors.set_position([0.0, 0.0])

    sleep(1.0)
    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except (KeyboardInterrupt, SystemExit):
        print("\rAborted, stopping motors")
        motorDriver.motors.stop()
    except:
        print("\rException thrown, stopping motors")
        motorDriver.motors.stop()
        raise
    print("____\nDONE.")
    motorDriver.motors.stop()

if __name__ == "__main__":
    main()
