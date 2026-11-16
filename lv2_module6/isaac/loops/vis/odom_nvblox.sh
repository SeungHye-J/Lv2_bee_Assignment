#!/usr/bin/env bash
# odom_nvblox.rviz via tools/rviz/start.sh — same ROS env checks + record.sh bag (~/bee_bags/nav_*) as the operator RViz.
#   ./odom_nvblox.sh                 # RVIZ_RECORD=0 ./odom_nvblox.sh to skip the bag
#   ./odom_nvblox.sh 8               # first arg = ROS_DOMAIN_ID (numeric); rest goes to rviz2
HERE="$(cd "$(dirname "$0")" && pwd)"
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then export ROS_DOMAIN_ID="$1"; shift; fi
RVIZ_CONFIG=../../isaac/loops/vis/odom_nvblox exec "$HERE/../../../tools/rviz/start.sh" "$@"
