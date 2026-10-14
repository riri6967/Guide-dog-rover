#!/bin/bash
source /opt/ros/jazzy/setup.bash
stop() { ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1; }
pos_odom() { ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | /usr/bin/python3 -c "
import sys
d={l.split(':')[0].strip():float(l.split(':')[1]) for l in sys.stdin if ':' in l}
print(d['x'],d['y'])"; }
pos_gz() { gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: "assistive_rover"' | /usr/bin/python3 -c "
import sys,re
t=sys.stdin.read(); p=t.split('position')[1].split('orientation')[0]
print(re.search(r'x: (\S+)',p).group(1), re.search(r'y: (\S+)',p).group(1))"; }
stop; sleep 3
read ox0 oy0 <<< "$(pos_odom)"; read gx0 gy0 <<< "$(pos_gz)"
timeout 10 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" > /dev/null 2>&1
stop; stop; sleep 3
read ox1 oy1 <<< "$(pos_odom)"; read gx1 gy1 <<< "$(pos_gz)"
/usr/bin/python3 -c "
import math
o=math.hypot($ox1-($ox0),$oy1-($oy0)); g=math.hypot($gx1-($gx0),$gy1-($gy0))
print(f'odom moved {o:.2f} m | gazebo moved {g:.2f} m | ratio odom/gazebo = {o/g:.2f}')"
