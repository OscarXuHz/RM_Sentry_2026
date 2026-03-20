#!/bin/bash
# Test that localization constraints are working:
# - pose.position.z should be ~0
# - orientation should be yaw-only (roll/pitch ≈ 0)
# - twist should have non-zero velocity when moving

echo "=== Localization Constraint Test ==="
echo "Sampling 10 /odom messages..."
echo ""

rostopic echo -n 10 /odom -p 2>/dev/null | while IFS=',' read -r ts frame seq px py pz ox oy oz ow vx vy vz rest; do
  if [[ "$ts" == "%time" ]]; then
    echo "  time, pos_z, roll(deg), pitch(deg), yaw(deg), vx, vy"
    continue
  fi
  # Use python to compute euler angles
  python3 -c "
import math, sys
ox,oy,oz,ow = float('$ox'), float('$oy'), float('$oz'), float('$ow')
pz = float('$pz')
vx, vy = float('$vx'), float('$vy')
# quaternion to euler (ZYX)
sinr_cosp = 2*(ow*ox + oy*oz)
cosr_cosp = 1 - 2*(ox*ox + oy*oy)
roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
sinp = 2*(ow*oy - oz*ox)
pitch = math.degrees(math.asin(max(-1,min(1,sinp))))
siny_cosp = 2*(ow*oz + ox*oy)
cosy_cosp = 1 - 2*(oy*oy + oz*oz)
yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
print('  Z=%+.4f, roll=%+.2f, pitch=%+.2f, yaw=%+.1f, vx=%+.3f, vy=%+.3f' % (pz, roll, pitch, yaw, vx, vy))
"
done

echo ""
echo "Expected: Z≈0, roll≈0°, pitch≈0°, vx/vy non-zero when moving"
