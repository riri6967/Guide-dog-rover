#!/bin/sh
# Assemble the single-file simulator from its parts.
# p0_meshes.js is generated from the URDF by tools/export_urdf_meshes.py.
# three.js r128 (MIT licence) and the two fonts (SIL OFL, tools/embed_fonts.py) are embedded from vendor/,
# so the page runs fully offline with no server.
cd "$(dirname "$0")"
OUT=../guide_dog_simulator_3d.html
THREE=vendor/three.r128.min.js
FONTS=vendor/fonts/fonts.css
PARTS="p0_meshes.js p2_core.js p2w_worlds.js p3_people.js p3h_hospital.js p4a_sim.js p4b_brain.js p4c_speech.js p4d_doors_lift.js p5a_scene.js p5b_furniture.js p5c_hospital3d.js p6_render.js p7_ui.js"
if [ ! -f "$THREE" ]; then
  echo "missing $THREE - download it once with:"
  echo "  curl -L -o $THREE https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
  exit 1
fi
if [ ! -f "$FONTS" ]; then echo "missing $FONTS - run: python tools/embed_fonts.py"; exit 1; fi
{ echo '<meta charset="utf-8">'
  head -n 1 p1_head.html
  echo '<style>'
  cat "$FONTS"
  echo '</style>'
  tail -n +3 p1_head.html
  echo '<script>'
  cat "$THREE"
  echo
  echo '</script>'
  echo '<script>'
  cat $PARTS
  echo '</script>'
} > "$OUT"
cat $PARTS > "${TMPDIR:-/tmp}/gdr_all.js"
echo "built $OUT ($(wc -c < "$OUT") bytes, offline)"
