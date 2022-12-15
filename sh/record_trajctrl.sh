#! /bin/bash
###
 # @Author: xindong324
 # @Date: 2022-09-24 10:52:05
 # @LastEditors: xindong324
 # @LastEditTime: 2022-09-24 10:52:15
 # @Description: file content
### 
rosbag record /position_cmd /mavros/local_position/odom /debugPx4ctrl /mavros/setpoint_raw/attitude
