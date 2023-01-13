#!/bin/bash
#  如果想要提到更高的频率只需要减小10000这个参数，这个就是设置时间间隔的现在间隔为10000us所以是100hz。
#  /mavros/imu/data_raw
#rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
#  /mavros/imu/data
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
# /mavros/local_position/odom
rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0


