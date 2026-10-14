#!/bin/bash
source /opt/ros/jazzy/setup.bash
read ox oy <<< "$(ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | /usr/bin/python3 -c "
import sys
d={l.split(':')[0].strip():float(l.split(':')[1]) for l in sys.stdin if ':' in l}
print(d['x'],d['y'])")"
read gx gy <<< "$(gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: \"assistive_rover\"' | /usr/bin/python3 -c "
import sys,re
t=sys.stdin.read(); p=t.split('position')[1].split('orientation')[0]
print(re.search(r'x: (\S+)',p).group(1), re.search(r'y: (\S+)',p).group(1))")"
/usr/bin/python3 -c "
import math
print(f'odom=({$ox:.2f},{$oy:.2f})  real=({$gx:.2f},{$gy:.2f})  odom error={math.hypot($ox-($gx),$oy-($gy)):.2f} m')"
