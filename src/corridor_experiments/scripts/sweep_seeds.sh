#!/usr/bin/env bash
# Phase 2, part 8: run the same 100 m corridor run under several Gazebo seeds,
# recording one bag per seed. Each seed is a different noise realisation of the
# same experiment, so the spread across seeds is what the headline figures need.
#
# One run at a time, deliberately: two Gazebo instances would compete for the
# GPU and change the timing of the thing being measured.
#
# Usage, from ~/ros2_ws in a sourced shell:
#   ./src/corridor_experiments/scripts/sweep_seeds.sh 42 7 13 101 2024
#   ./src/corridor_experiments/scripts/sweep_seeds.sh            # default set
#
# Writes bags/ekf_sweep_s<seed> and bags/sweep_logs/s<seed>.log. A seed whose
# bag already exists is skipped, so an interrupted sweep can be resumed by
# running the same command again.
 
set -u
 
SEEDS=("$@")
if [ ${#SEEDS[@]} -eq 0 ]; then
  SEEDS=(42 7 13 101 2024 31337 5 88 404 1234)
fi
 
WS="$HOME/ros2_ws"
BAGS="$WS/bags"
LOGS="$BAGS/sweep_logs"
TOPICS="/ekf/state /ekf/update /corridor_geometry /odom_ekf /odom_dr /imu /model/vehicle/pose /clock"
NOISE=0.03
TARGET_X=110.0
SPEED=2.0
TIMEOUT=420          # wall seconds before a run is given up on
POLL=5               # wall seconds between ground truth position checks
 
# Stop a whole process group and wait for it, but never forever: one wedged
# shutdown would otherwise stall the sweep. The group matters: killing only the
# launch leaves its nodes orphaned, and orphaned nodes keep publishing into the
# next run's bag, which shows up later as duplicate timestamps.
stop_group() {
  local PID=$1 LIMIT=$2 n=0
  kill -INT -"$PID" 2>/dev/null || kill -INT "$PID" 2>/dev/null || return 0
  while kill -0 "$PID" 2>/dev/null && [ $n -lt "$LIMIT" ]; do
    sleep 1; n=$((n + 1))
  done
  if kill -0 "$PID" 2>/dev/null; then
    echo "  pid $PID did not stop after ${LIMIT}s, terminating the group"
    kill -TERM -"$PID" 2>/dev/null; sleep 2
    kill -KILL -"$PID" 2>/dev/null
  fi
  wait "$PID" 2>/dev/null
}
 
# Anything left from an earlier run must go before a bag is recorded. A stale
# ekf or dead_reckoning node publishes on the same topics as the new one, and
# the bag then holds two nodes' messages interleaved.
clear_stale() {
  local FOUND=0
  # Patterns name the INSTALLED executables. A pattern like
  # "corridor_experiments" would also match this script's own command line,
  # since it lives in src/corridor_experiments/scripts, and the sweep would
  # kill itself on its first cleanup.
  local PATS=("gz sim" "parameter_bridge" "robot_state_publisher" \
              "install/corridor_perception/lib" "install/corridor_experiments/lib" \
              "ros2 bag record")
  for PAT in "${PATS[@]}"; do
    if pgrep -f "$PAT" >/dev/null 2>&1; then FOUND=1; pkill -f "$PAT" 2>/dev/null; fi
  done
  if [ $FOUND -eq 1 ]; then
    sleep 3
    for PAT in "${PATS[@]}"; do pkill -9 -f "$PAT" 2>/dev/null; done
    sleep 2
    echo "  cleared processes left over from an earlier run"
  fi
}
 
command -v ros2 >/dev/null || { echo "ros2 not found: source install/setup.bash first"; exit 1; }
mkdir -p "$LOGS"
cd "$WS" || exit 1
 
echo "sweeping ${#SEEDS[@]} seeds: ${SEEDS[*]}"
echo "noise $NOISE m, target_x $TARGET_X, speed $SPEED"
 
for SEED in "${SEEDS[@]}"; do
  BAG="$BAGS/ekf_sweep_s$SEED"
  LOG="$LOGS/s$SEED.log"
 
  if [ -d "$BAG" ]; then
    echo "seed $SEED: bag exists, skipping"
    continue
  fi
 
  echo "seed $SEED: starting"
  clear_stale
  # shellcheck disable=SC2086
  setsid bash -c "cd '$BAGS' && exec ros2 bag record -o 'ekf_sweep_s$SEED' $TOPICS" \
    >"$LOG.rec" 2>&1 &
  REC_PID=$!
  sleep 3
 
  # Both variables matter, for different processes. PYTHONUNBUFFERED unbuffers
  # the launch tool, which is Python. RCUTILS_LOGGING_BUFFERED_STREAM=0
  # unbuffers the C++ nodes it starts: C stdio block buffers when its output is
  # a pipe, so run_controller's termination line would otherwise sit in a 4 KB
  # buffer that never fills. It propagates to every node in the launch.
  # setsid puts the launch in its own process group so the whole tree, nodes
  # included, can be signalled together.
  PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0 \
  setsid ros2 launch tunnel_vehicle simulation.launch.py \
    y:=0.0 target_x:=$TARGET_X speed:=$SPEED lidar_noise:=$NOISE seed:="$SEED" \
    >"$LOG" 2>&1 &
  RUN_PID=$!
 
  # Completion is detected from ROS, not from the log. The launch writes to a
  # file rather than a terminal, and the C++ nodes block buffer their output in
  # that case, so the run controller's termination line can sit unseen in a
  # buffer for the whole run. Ground truth x is authoritative and needs no
  # cooperation from any node's output stream.
  REACHED=0
  LAST_X=""
  STILL=0
  for (( i = 0; i < TIMEOUT; i += POLL )); do
    if ! kill -0 $RUN_PID 2>/dev/null; then
      echo "  launch exited after ${i}s without reaching the target"
      break
    fi
    # Keep only a line that is entirely a number. Stripping stray characters out
    # of a warning message instead would leave something like "eeeeeeeee", and
    # awk compares a non numeric operand as a STRING, where "e" sorts above
    # "110.0" and the target looks reached one second into the run.
    X=$(timeout 8 ros2 topic echo --once --field pose.position.x \
          /model/vehicle/pose 2>/dev/null \
        | grep -Eom1 '^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$')
    if [ -z "$X" ]; then
      echo "  ${i}s: waiting for /model/vehicle/pose"
    else
      echo "  ${i}s: x = $X"
      # a+0 and b+0 force numeric comparison whatever the operands look like.
      if awk -v a="$X" -v b="$TARGET_X" 'BEGIN{exit !(a+0>=b+0)}'; then
        REACHED=1
        echo "  target x reached, letting it come to rest"
        sleep 4
        break
      fi
      # A vehicle that stops short has failed; do not wait out the timeout.
      if [ -n "$LAST_X" ] && awk -v a="$X" -v b="$LAST_X" \
           'BEGIN{d=a+0-(b+0); exit !(d<0.05 && d>-0.05)}'; then
        STILL=$((STILL + 1))
        if [ $STILL -ge 4 ]; then
          echo "  stationary at x = $X for $((STILL * POLL))s, giving up on this seed"
          break
        fi
      else
        STILL=0
      fi
      LAST_X="$X"
    fi
    sleep $POLL
  done
 
  sleep 2                      # let the last messages reach the recorder
  stop_group $REC_PID 20       # the recorder first, so the bag closes cleanly
  stop_group $RUN_PID 40
  clear_stale                  # belt and braces: nothing may survive into the next seed
  sleep 3                      # let Gazebo release the GPU before the next run
 
  if [ $REACHED -eq 1 ] && [ -d "$BAG" ]; then
    echo "seed $SEED: done, bag written"
  else
    echo "seed $SEED: RUN NOT USABLE. See $LOG"
    rm -rf "$BAG"
  fi
done
 
echo
echo "done. Analyse with:"
echo "  python3 src/corridor_experiments/scripts/aggregate_sweep.py bags/ekf_sweep_s*"
 