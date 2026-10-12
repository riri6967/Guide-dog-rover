#!/usr/bin/env python3
"""
Drive the rover from a phone browser, with the full ROS stack in the loop.

Serves a touch control page on the Pi and publishes /cmd_vel from it, so the
path is:  phone -> Pi web page -> /cmd_vel -> micro-ROS -> ESP32 -> motors,
with /odom and the encoder ticks coming back the other way.

    python3 web_teleop.py

Then open http://<pi-ip>:8080 on any phone on the same network.

Hold-to-drive: the browser repeats the command every 150 ms and the node
publishes at 10 Hz. Stop sending and the rover stops — both here, after
500 ms, and again on the ESP32 after 900 ms. Two independent failsafes, which
is the right number for something that moves on its own.
"""
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

PORT = 8080
CMD_TIMEOUT = 0.5          # seconds of silence before we stop commanding
PUBLISH_HZ = 10

PAGE = """<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Guide Dog Rover</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;padding:14px;font-family:system-ui,sans-serif;background:#12151a;
      color:#e8eaed;user-select:none;-webkit-user-select:none}
 h1{font-size:17px;margin:0 0 12px;font-weight:600}
 #pad{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;max-width:340px;margin:0 auto}
 #pad button{aspect-ratio:1;font-size:26px;border:1px solid #333a45;border-radius:12px;
      background:#1c2028;color:#e8eaed;touch-action:none}
 #pad button:active{background:#2d6cdf}
 #pad .estop{background:#c0392b;border-color:#c0392b;font-size:15px;font-weight:700}
 .sp{max-width:340px;margin:16px auto 0}
 .sp label{font-size:13px;color:#9aa4b2;display:flex;justify-content:space-between}
 input[type=range]{width:100%;margin-top:8px}
 #st{max-width:340px;margin:14px auto 0;padding:11px;background:#1c2028;border-radius:9px;
      font-size:13px;font-family:ui-monospace,monospace;color:#9aa4b2;line-height:1.6}
 .warn{color:#e8a33d}
</style></head><body>

<h1>Guide Dog Rover</h1>

<div id="pad">
  <span></span>
  <button data-v="1"  data-w="0">&#9650;</button>
  <span></span>
  <button data-v="0"  data-w="1">&#9664;</button>
  <button class="estop" onclick="stopHold()">STOP</button>
  <button data-v="0"  data-w="-1">&#9654;</button>
  <span></span>
  <button data-v="-1" data-w="0">&#9660;</button>
  <span></span>
</div>

<div class="sp">
  <label>Speed <span id="spv">0.30</span> m/s</label>
  <input type="range" id="sp" min="10" max="80" value="30">
</div>

<div id="st">connecting...</div>

<script>
var speed = 0.30, held = null, timer = null;

document.getElementById('sp').oninput = function(e){
  speed = e.target.value / 100;
  document.getElementById('spv').textContent = speed.toFixed(2);
};

function send(v, w){
  fetch('/cmd?v=' + v + '&w=' + w).catch(function(){});
}

function startHold(b){
  held = b;
  var tick = function(){
    if (!held) return;
    send((held.dataset.v * speed).toFixed(3), (held.dataset.w * 1.2).toFixed(3));
  };
  tick();
  timer = setInterval(tick, 150);
}

function stopHold(){
  held = null;
  clearInterval(timer);
  send(0, 0);
}

document.querySelectorAll('#pad button[data-v]').forEach(function(b){
  ['mousedown','touchstart'].forEach(function(ev){
    b.addEventListener(ev, function(e){ e.preventDefault(); startHold(b); });
  });
  ['mouseup','mouseleave','touchend','touchcancel'].forEach(function(ev){
    b.addEventListener(ev, function(e){ e.preventDefault(); stopHold(); });
  });
});

// Backgrounding the tab stops timers on mobile, so drop the command explicitly.
document.addEventListener('visibilitychange', function(){
  if (document.hidden) stopHold();
});

setInterval(function(){
  fetch('/status').then(function(r){ return r.json(); }).then(function(s){
    document.getElementById('st').innerHTML =
      'cmd &nbsp;' + s.v.toFixed(2) + ' m/s &nbsp;' + s.w.toFixed(2) + ' rad/s<br>' +
      'ticks &nbsp;L ' + s.tl + ' &nbsp; R ' + s.tr + '<br>' +
      'odom &nbsp;' + s.x.toFixed(2) + ', ' + s.y.toFixed(2);
  }).catch(function(){
    document.getElementById('st').innerHTML = '<span class="warn">link lost</span>';
  });
}, 500);
</script></body></html>
"""


class Teleop(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Int32, 'ticks_left', self._tl, 10)
        self.create_subscription(Int32, 'ticks_right', self._tr, 10)
        self.create_subscription(Odometry, 'odom', self._odom, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        self.v = self.w = 0.0
        self.last_cmd = 0.0
        self.tl = self.tr = 0
        self.x = self.y = 0.0

    def _tl(self, m): self.tl = m.data
    def _tr(self, m): self.tr = m.data

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y

    def set_cmd(self, v, w):
        self.v, self.w = v, w
        self.last_cmd = time.time()

    def _tick(self):
        # Stop commanding if the browser has gone quiet. The ESP32 has its own
        # 900ms failsafe behind this one; neither relies on the other.
        if time.time() - self.last_cmd > CMD_TIMEOUT:
            self.v = self.w = 0.0

        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)
        self.pub.publish(msg)


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass            # silence per-request logging

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)

            if u.path == '/':
                self._send(PAGE.encode(), 'text/html')

            elif u.path == '/cmd':
                q = parse_qs(u.query)
                try:
                    node.set_cmd(float(q.get('v', [0])[0]), float(q.get('w', [0])[0]))
                except ValueError:
                    pass
                self._send(b'ok', 'text/plain')

            elif u.path == '/status':
                body = json.dumps({
                    'v': node.v, 'w': node.w,
                    'tl': node.tl, 'tr': node.tr,
                    'x': node.x, 'y': node.y,
                }).encode()
                self._send(body, 'application/json')

            else:
                self.send_response(404)
                self.end_headers()

    return Handler


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return 'localhost'


def main():
    rclpy.init()
    node = Teleop()

    server = HTTPServer(('0.0.0.0', PORT), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()

    print()
    print('  Open this on your phone:   http://%s:%d' % (local_ip(), PORT))
    print('  (phone must be on the same network as the Pi)')
    print()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
