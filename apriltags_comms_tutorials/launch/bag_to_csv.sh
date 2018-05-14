#!/bin/bash
BAGNAME=/home/nick/bag.bag
rostopic echo -b ${BAGNAME} -p /gazebo/model_states              > model_states.txt
rostopic echo -b ${BAGNAME} -p /jackal0/odometry/tracked         > 0_tracked.txt
rostopic echo -b ${BAGNAME} -p /jackal0/odometry/robot2robot     > 0_robot2robot.txt
rostopic echo -b ${BAGNAME} -p /jackal0/odometry/global_filtered > 0_ekf.txt
rostopic echo -b ${BAGNAME} -p /jackal0/odometry/gps             > 0_gps.txt
rostopic echo -b ${BAGNAME} -p /jackal0/gps/filtered             > 0_ekf_navsat.txt
rostopic echo -b ${BAGNAME} -p /jackal0/navsat/fix               > 0_gps_navsat.txt

rostopic echo -b ${BAGNAME} -p /jackal1/odometry/tracked         > 1_tracked.txt
rostopic echo -b ${BAGNAME} -p /jackal1/odometry/robot2robot     > 1_robot2robot.txt
rostopic echo -b ${BAGNAME} -p /jackal1/odometry/global_filtered > 1_ekf.txt
rostopic echo -b ${BAGNAME} -p /jackal1/odometry/gps             > 1_gps.txt
rostopic echo -b ${BAGNAME} -p /jackal1/gps/filtered             > 1_ekf_navsat.txt
rostopic echo -b ${BAGNAME} -p /jackal1/navsat/fix               > 1_gps_navsat.txt
