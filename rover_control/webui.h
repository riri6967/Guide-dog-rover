/*
 * webui.h — the control page, served from flash.
 *
 * Hold-to-drive: the browser repeats the command every 150 ms while a button
 * is held and stops repeating on release. That repetition IS the failsafe
 * signal — if the phone walks out of range or the tab closes, the commands
 * stop arriving and the rover's watchdog cuts the motors.
 */
#pragma once
#include <Arduino.h>

const char PAGE_INDEX[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Guide Dog Rover</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;padding:14px;font-family:system-ui,sans-serif;background:#12151a;color:#e8eaed;
      user-select:none;-webkit-user-select:none}
 h1{font-size:17px;margin:0 0 12px;font-weight:600}
 .modes{display:flex;gap:8px;margin-bottom:14px}
 .modes button{flex:1;padding:12px 4px;font-size:14px;font-weight:600;border:1px solid #333a45;
      border-radius:9px;background:#1c2028;color:#e8eaed}
 .modes button.on{background:#2d6cdf;border-color:#2d6cdf}
 .modes button.stop.on{background:#c0392b;border-color:#c0392b}
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

<div class="modes">
  <button id="mStop" class="stop on" onclick="setMode('stop')">STOP</button>
  <button id="mManual" onclick="setMode('manual')">Manual</button>
  <button id="mAuto" onclick="setMode('auto')">Auto</button>
</div>

<div id="pad">
  <span></span>
  <button data-l="1"  data-r="1">&#9650;</button>
  <span></span>
  <button data-l="-1" data-r="1">&#9664;</button>
  <button class="estop" onclick="estop()">STOP</button>
  <button data-l="1"  data-r="-1">&#9654;</button>
  <span></span>
  <button data-l="-1" data-r="-1">&#9660;</button>
  <span></span>
</div>

<div class="sp">
  <label>Speed <span id="spv">180</span></label>
  <input type="range" id="sp" min="60" max="255" value="180">
</div>

<div id="st">connecting...</div>

<script>
var speed = 180, held = null, timer = null;

document.getElementById('sp').oninput = function(e){
  speed = +e.target.value;
  document.getElementById('spv').textContent = speed;
};

function send(url){ return fetch(url).catch(function(){}); }

function setMode(m){
  send('/mode?m=' + m);
  ['Stop','Manual','Auto'].forEach(function(x){
    document.getElementById('m'+x).classList.toggle('on', x.toLowerCase()===m);
  });
}

function estop(){ stopHold(); send('/estop'); setMode('stop'); }

function startHold(b){
  held = b;
  var tick = function(){
    if (!held) return;
    send('/cmd?l=' + Math.round(held.dataset.l*speed) +
             '&r=' + Math.round(held.dataset.r*speed));
  };
  tick();
  timer = setInterval(tick, 150);
}

function stopHold(){
  held = null;
  clearInterval(timer);
  send('/cmd?l=0&r=0');
}

document.querySelectorAll('#pad button[data-l]').forEach(function(b){
  ['mousedown','touchstart'].forEach(function(ev){
    b.addEventListener(ev, function(e){ e.preventDefault(); startHold(b); });
  });
  ['mouseup','mouseleave','touchend','touchcancel'].forEach(function(ev){
    b.addEventListener(ev, function(e){ e.preventDefault(); stopHold(); });
  });
});

// Releasing the button is not the only way control ends. Backgrounding the
// tab stops timers on mobile, so drop the command explicitly.
document.addEventListener('visibilitychange', function(){
  if (document.hidden) stopHold();
});

setInterval(function(){
  fetch('/status').then(function(r){ return r.json(); }).then(function(s){
    document.getElementById('st').innerHTML =
      'mode &nbsp;' + s.mode + '<br>' +
      'motor L/R &nbsp;' + s.l + ' / ' + s.r + '<br>' +
      'battery &nbsp;' + (s.v > 0 ? s.v.toFixed(2) + ' V' : 'not monitored') +
      (s.lowbat ? ' <span class="warn">LOW</span>' : '') + '<br>' +
      'uptime &nbsp;' + s.up + ' s';
  }).catch(function(){
    document.getElementById('st').innerHTML = '<span class="warn">link lost</span>';
  });
}, 500);
</script></body></html>
)HTML";
