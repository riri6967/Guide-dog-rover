#!/bin/bash
source /opt/ros/jazzy/setup.bash
stop() { ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1; }
yaw_odom() { ros2 topic echo /odom --once --field pose.pose.orientation 2>/dev/null | /usr/bin/python3 -c "
import sys,math
d={l.split(':')[0].strip():float(l.split(':')[1]) for l in sys.stdin if ':' in l}
print(math.degrees(2*math.atan2(d['z'],d['w'])))"; }
yaw_gz() { gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: "assistive_rover"' | /usr/bin/python3 -c "
import sys,math,re
t=sys.stdin.read(); o=t.split('orientation')[1]
z=float(re.search(r'z: (\S+)',o).group(1)); w=float(re.search(r'w: (\S+)',o).group(1))
print(math.degrees(2*math.atan2(z,w)))"; }
stop; sleep 3
o0=$(yaw_odom); g0=$(yaw_gz)
timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}" > /dev/null 2>&1
stop; stop; sleep 3
o1=$(yaw_odom); g1=$(yaw_gz)
/usr/bin/python3 -c "
w=lambda a:(a+180)%360-180
o=w($o1-($o0)); g=w($g1-($g0))
print(f'commanded ~103 deg | odom turned {o:.1f} | gazebo turned {g:.1f} | ratio odom/gazebo = {o/g:.2f}')"
