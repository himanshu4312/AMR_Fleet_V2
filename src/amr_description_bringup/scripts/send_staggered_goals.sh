#!/usr/bin/env bash

set -euo pipefail

STAGGER_S="${1:-6.0}"

send_goal() {
  local robot="$1" x="$2" y="$3" qz="$4" qw="$5"
  ros2 action send_goal "/${robot}/navigate_to_pose" nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {z: ${qz}, w: ${qw}}}}}" \
    > "/tmp/${robot}_goal.log" 2>&1 &
}

echo "Dispatching robot1 -> robot2's spawn"
send_goal robot1 2.65 -13.3 -0.4805 0.8770
sleep "${STAGGER_S}"

echo "Dispatching robot2 -> robot1's spawn"
send_goal robot2 -12.2 9.95 0.8770 0.4805
sleep "${STAGGER_S}"

echo "Dispatching robot3 -> robot4's spawn"
send_goal robot3 3.90 -5.85 -0.7071 0.7071
sleep "${STAGGER_S}"

echo "Dispatching robot4 -> robot3's spawn"
send_goal robot4 3.90 5.25 0.7071 0.7071

echo "All 4 goals dispatched (staggered ${STAGGER_S}s apart). Per-robot logs: /tmp/robotN_goal.log"
wait
echo "All goal actions finished."
