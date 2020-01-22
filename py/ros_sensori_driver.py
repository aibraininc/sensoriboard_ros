#!/usr/bin/python

from lib.sensorimotor import Sensorimotor
from time import sleep
import rospy
import rospkg
import roslib
from std_msgs.msg import Float32MultiArray
from rosparam import upload_params
from yaml import load

def load_params_from_yaml(path):
    f = open(path, 'r')
    yamlfile = load(f)
    return yamlfile

pan_global = 0
tilt_global = 0
# 45 degree to radian
degree_45 =  0.785398

pkg_path = rospkg.RosPack().get_path('sensoriboard_ros')
config = load_params_from_yaml(pkg_path+'/config/motor_param.yaml')['sensori']['config']

def reverseTransformAngle(motor_num, motor_pos):
    pan_rate = (config['pan_45'] - config['pan_mid']) / degree_45
    tilt_rate = (config['tilt_45'] - config['tilt_mid']) / degree_45

    ret = 0
    if motor_num == 0: # pan
        ret =  (motor_pos - config['pan_mid']) / pan_rate
    elif motor_num == 1:
        ret = (motor_pos -  config['tilt_mid']) / tilt_rate
    return ret

# radian to [-1,1]
def transformAngle(motor_num, motor_radian):

    pan_rate = (config['pan_45'] - config['pan_mid']) / degree_45
    tilt_rate = (config['tilt_45'] - config['tilt_mid']) / degree_45
    print(pan_rate, tilt_rate)

    ret = 0
    if motor_num == 0: # pan
        ret = pan_rate *  motor_radian + config['pan_mid']
    elif motor_num == 1:
        ret = tilt_rate * motor_radian + config['tilt_mid']
    return ret

def cutPosition(_min, _max, val):
    if val > _max:
        val = _max
    if val < _min:
        val = _min
    return val

class ros_sensori_driver:
    def __init__(self):
        print(config)
        self.motors = Sensorimotor(number_of_motors= config['motor_cnt'], verbose=False)
        self.setMotorConfig(self.motors)
        self.jointPub = rospy.Publisher("joint/poses", Float32MultiArray, queue_size = 10)
        rospy.Subscriber("joint/cmd", Float32MultiArray, self.cmdCallback, queue_size = 10)
        # create timer
        rospy.Timer(rospy.Duration(0.1), self.updateTimerCallback)

    def cmdCallback(self, cmd):
        global pan_global, tilt_global
        pan  = transformAngle(0, cmd.data[0])
        tilt  = transformAngle(1, cmd.data[1])
        pan = cutPosition(config['pan_min'], config['pan_max'], pan)
        tilt = cutPosition(config['tilt_min'], config['tilt_max'], tilt)
        pan_global = pan
        tilt_global = tilt
        print("--- cmd callback ---")
        print(pan_global, tilt_global)

        self.motors.set_position([pan, tilt])

    def updateTimerCallback(self,event):
        global pan_global, tilt_global
        joints = Float32MultiArray()
        values = self.motors.get_position()
        values_origin = [values[0], values[1]] # pan, tilt
        pan = reverseTransformAngle(0,values[0])
        tilt = reverseTransformAngle(1,values[1])
        joints.data = [pan, tilt, values_origin[0], values_origin[1]]
        self.jointPub.publish(joints)
        # print("--- updateTimerCallback ---")
        # print(values_origin[0], values_origin[1])
        print 'error : '+str(pan_global-values_origin[0])+', '+ str(tilt_global-values_origin[1])

    def setMotorConfig(self, motors):
        motors.set_voltage_limit([0.18, 0.18])
        motors.start()
        motors.set_pos_ctrl_params(0, Kp = 1.9, Ki = 1.0, Kd = 0.008, deadband = 0.04, pulse_threshold = 0.05)
        motors.set_pos_ctrl_params(1, Kp = 1.9, Ki = 1.0, Kd = 0.008, deadband = 0.04, pulse_threshold = 0.05)
        # motors.set_pos_ctrl_params(0, Kp = 1.8, Ki = 0, Kd = 0.05, deadband = 0.00, pulse_threshold = 0.00)
        # motors.set_pos_ctrl_params(1, Kp = 1.8, Ki = .8, Kd = 0.03, deadband = 0.00, pulse_threshold = 0.00)

def main():
    rospy.init_node('sensori_driver', anonymous=True)
    motorDriver = ros_sensori_driver()
    N = motorDriver.motors.ping()
    print("Found {0} sensorimotors.".format(N))
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
