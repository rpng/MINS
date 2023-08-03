#!/usr/bin/env bash
source $( dirname -- "$0"; )/setup_env.sh
source $( dirname -- "$0"; )/setup_output.sh CalibErrorPlot

seeds=( "0" "1" #"2" "3" #"4" "5" "6" "7" "8" "9"
)

n_total=$((${#seeds[@]} * 5))
printf "Total run: $n_total\n"
printf "\n\n\x1B[33mroslaunch mins_eval plot_consistency.launch load_path:=\"$output_path\" save_path:=\"$output_path\" visualize:=\"true\"\e[0m\n\n"

dataset="UD_Warehouse"
# Monte Carlo runs for this dataset
for j in "${!seeds[@]}"; do
  printf " run ${seeds[j]}\n"

  name="MINS_IC"; printf "  ($cnt/$n_total)$name "; start_time="$(date -u +%s)"; cnt=$((cnt+1));
  roslaunch mins simulation.launch sim_seed:="${seeds[j]}" sys_save_state:="true" sys_path_state:="$output_path/${seeds[j]}/" dataset:=${dataset} \
  cam_enabled:="true" gps_enabled:="false" wheel_enabled:="false" vicon_enabled:="false" lidar_enabled:="false" \
  cam_do_calib_ext:="true" cam_do_calib_int:="true" cam_do_calib_dt:="true" cam_max_n:=1 &>/dev/null
  printf "took $(("$(date -u +%s)" - $start_time))s\n";

  name="MINS_ICG"; printf "  ($cnt/$n_total)$name "; start_time="$(date -u +%s)"; cnt=$((cnt+1));
  roslaunch mins simulation.launch sim_seed:="${seeds[j]}" sys_save_state:="true" sys_path_state:="$output_path/${seeds[j]}/" dataset:=${dataset}\
  cam_enabled:="true" gps_enabled:="true" wheel_enabled:="false" vicon_enabled:="false" lidar_enabled:="false" \
  gps_do_calib_ext:="true" gps_do_calib_dt:="true" gps_max_n:=1 &>/dev/null
  printf "took $(("$(date -u +%s)" - $start_time))s\n";

  name="MINS_IVW"; printf "  ($cnt/$n_total)$name "; start_time="$(date -u +%s)"; cnt=$((cnt+1));
  roslaunch mins simulation.launch sim_seed:="${seeds[j]}" sys_save_state:="true" sys_path_state:="$output_path/${seeds[j]}/" dataset:=${dataset}\
  cam_enabled:="false" gps_enabled:="false" wheel_enabled:="true" vicon_enabled:="true" lidar_enabled:="false" \
  wheel_do_calib_ext:="true" wheel_do_calib_int:="true" wheel_do_calib_dt:="true" vicon_max_n:=1 &>/dev/null
  printf "took $(("$(date -u +%s)" - $start_time))s\n";

  name="MINS_IL"; printf "  ($cnt/$n_total)$name "; start_time="$(date -u +%s)"; cnt=$((cnt+1));
  roslaunch mins simulation.launch sim_seed:="${seeds[j]}" sys_save_state:="true" sys_path_state:="$output_path/${seeds[j]}/" dataset:=${dataset}\
  cam_enabled:="false" gps_enabled:="false" wheel_enabled:="false" vicon_enabled:="false" lidar_enabled:="true" \
  lidar_do_calib_ext:="true" lidar_do_calib_dt:="true" lidar_max_n:=1 &>/dev/null
  printf "took $(("$(date -u +%s)" - $start_time))s\n";

  name="MINS_IV"; printf "  ($cnt/$n_total)$name "; start_time="$(date -u +%s)"; cnt=$((cnt+1));
  roslaunch mins simulation.launch sim_seed:="${seeds[j]}" sys_save_state:="true" sys_path_state:="$output_path/${seeds[j]}/" dataset:=${dataset}\
  cam_enabled:="false" gps_enabled:="false" wheel_enabled:="false" vicon_enabled:="true" lidar_enabled:="false" \
  vicon_do_calib_ext:="true" vicon_do_calib_dt:="true" vicon_max_n:=1 &>/dev/null
  printf "took $(("$(date -u +%s)" - $start_time))s\n";
done

# Plot
roslaunch mins_eval plot_consistency.launch load_path:="$output_path" save_path:="$output_path" visualize:="true" #&>/dev/null