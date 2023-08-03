#!/usr/bin/env bash
# Clean terminal
clear; rosclean purge -y &>/dev/null
# Build workspace
catkin build; if [ "$?" -ne 0 ]; then echo -e '\033[0;31mBuild failed. Exit'; exit; fi
# Source workspace
source devel/setup.bash; if [ "$?" -ne 0 ]; then echo -e '\033[0;31mSource failed. Run this in catkin workspace.\e[0m'; exit; fi

# Nice timing function
print_time() {
  # Start end times
  cnt=$1
  n_total=$2
  run_start_t=$3
  all_start_t=$4
  run_end_t=$(date -u +%s)

  # Comptue elapsed time
  run_t=$((run_end_t - run_start_t))
  run_tt=$((run_end_t - all_start_t))
  remain_t=$((run_tt * (n_total - cnt) * 1000 / cnt / 1000))

  # Convert to HMS
  m_tt=$((run_tt/60))
  h_tt=$((m_tt/60))
  m_tt=$((m_tt%60))
  s_tt=$((run_tt%60))
  m_rt=$((remain_t/60))
  h_rt=$((m_rt/60))
  m_rt=$((m_rt%60))
  s_rt=$((remain_t%60))

  #Print
  printf "took $((run_t))s. Total $((h_tt))h $((m_tt))m $((s_tt))s. Remain $((h_rt))h $((m_rt))m $((s_rt))s\n";
  return $run_t
}

# This is for timing. Names should exactly match in use
cnt=1
st="$(date -u +%s)";

gt_path_H="$PWD/src/mins/mins_data/GroundTruths/holonomic"
gt_path_N="$PWD/src/mins/mins_data/GroundTruths/nonholonomic"
gt_path_P="$PWD/src/mins/mins_data/GroundTruths/nonholonomic_planar"