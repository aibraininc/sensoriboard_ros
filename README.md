# [RUN PROJECT]
  
  <!-- start low level motor driver-->
  <node name="motor_driver" pkg="sensoriboard_ros" type="ros_sensori_driver.py" respawn="false" output="screen"
    ns="/gretchen">
    <remap from="joint/poses" to="/gretchen/joint/poses" />
    <remap from="joint/cmd" to="/gretchen/joint/cmd" />
  </node>
 

# [CONFIG EXPLAINED] 

* act_rad - actual range of motor in radian, HS82 is 165 degree
* act_min - actual min of the motor in sensorimotor values 
* act_max - actual max of the motor in sensorimotor values
* calib_mid - actual middle position of motor in sensorimotor values --> should change it to act_mid later 

## Change the parameters of pan and tilt
 
  pan : {'motor_id': 1, 'kp': 0.9, 'ki': 1.0, 'kd': 0.008, 'deadband': 0.04, 'pulse_threshold': 0.10,
          'limit_volt': 0.18,
          'act_rad': 2.87979, 'act_min': -0.863, 'act_max': 0.892, 'calib_mid': 0.146,'motor_cnt': 2}


