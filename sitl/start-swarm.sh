#!/usr/bin/env bash
# Native alternative to Docker: start N ArduCopter SITL instances with ArduPilot's sim_vehicle.py.
# Requires an ArduPilot checkout (https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
#   ARDUPILOT=~/ardupilot ./sitl/start-swarm.sh 5
# Instance i listens on TCP 5760 + 10*i  →  SkyHive: "ArduPilot SITL (TCP)", port 5760.
set -euo pipefail
COUNT="${1:-5}"
ARDUPILOT="${ARDUPILOT:-$HOME/ardupilot}"
HOME_LAT=40.9925; HOME_LON=29.1240; HOME_ALT=30
for ((i = 0; i < COUNT; i++)); do
  read -r lat lon < <(python3 -c "import math;i=$i;R=6378137.0;n=-(i//5)*6.0;e=(i%5-2)*6.0;print(f'{$HOME_LAT+n/R*180/math.pi:.8f} {$HOME_LON+e/(R*math.cos(math.radians($HOME_LAT)))*180/math.pi:.8f}')")
  (cd "$ARDUPILOT/ArduCopter" && sim_vehicle.py -v ArduCopter -I"$i" --sysid $((i + 1)) --no-mavproxy \
     -l "$lat,$lon,$HOME_ALT,0" > "/tmp/skyhive-sitl$i.log" 2>&1 &)
  echo "SITL #$i sysid=$((i + 1)) tcp:$((5760 + 10 * i))"
done
echo "Logs: /tmp/skyhive-sitl*.log  ·  stop: pkill -f arducopter"
