[RUN PROJECT]

 Update control launch  
  <!-- start low level motor driver-->
  <node name="motor_driver" pkg="sensoriboard_ros" type="ros_sensori_driver_test.py" respawn="false" output="screen"
    ns="/gretchen">
    <remap from="joint/poses" to="/gretchen/joint/poses" />
    <remap from="joint/cmd" to="/gretchen/joint/cmd" />
  </node>
[EXAMPLE]

[CODE EXPLAINED] 
	 1. 
			- 
[CONFIG EXPLAINED] 

act_rad - actual range of motor in radian, HS82 is 165 degree
act_min - actual min of the motor in sensorimotor values 
act_max - actual max of the motor in sensorimotor values
calib_mid - actual middle position of motor in sensorimotor values --> should change it to act_mid later 

others are not used 

  pan : {'motor_id': 1, 'kp': 0.9, 'ki': 1.0, 'kd': 0.008, 'deadband': 0.04, 'pulse_threshold': 0.10,
          'limit_volt': 0.18,
          'act_rad': 2.87979, 'act_min': -0.94, 'act_max': 0.81, 'calib_min': -0.85, 'calib_max': 0.85, 'calib_mid': -0.03,
          'limit_min': 0.174533, 'limit_max': 1.39626,
          'motor_cnt': 2}
