#!/usr/bin/env bash
# Starts COUNT ArduCopter SITL instances in one container.
# Instance i (0-based): TCP 5760+10*i, SYSID i+1, home offset 6 m east per drone (rows of 5).
set -euo pipefail
COUNT="${COUNT:-5}"
HOME_LAT="${HOME_LAT:-40.9925}"
HOME_LON="${HOME_LON:-29.1240}"
HOME_ALT="${HOME_ALT:-30}"
SPEEDUP="${SPEEDUP:-1}"
mkdir -p /sitl && cd /sitl

python3 - "$COUNT" "$HOME_LAT" "$HOME_LON" > homes.txt <<'PY'
import math, sys
n, lat, lon = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
R = 6378137.0
for i in range(n):
    north = -(i // 5) * 6.0
    east = (i % 5 - 2) * 6.0
    print(f"{lat + north / R * 180 / math.pi:.8f},{lon + east / (R * math.cos(math.radians(lat))) * 180 / math.pi:.8f}")
PY

i=0
while read -r ll; do
  mkdir -p "inst$i" && cd "inst$i"
  sysid=$((i + 1))
  printf 'SYSID_THISMAV %d\n' "$sysid" > identity.parm
  arducopter --model + --speedup "$SPEEDUP" -I"$i" --sysid "$sysid" \
    --home "${ll},${HOME_ALT},0" \
    --defaults /opt/params/copter.parm,identity.parm > "../sitl$i.log" 2>&1 &
  echo "SITL #$i  sysid=$sysid  home=$ll  tcp:$((5760 + 10 * i))"
  cd ..
  i=$((i + 1))
done < homes.txt
wait
