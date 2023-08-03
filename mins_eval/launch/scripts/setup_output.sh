#!/usr/bin/env bash
# Location to save log files into
output_path="$PWD/../outputs/$(date +%Y-%m-%d-%H-%M-%S)-$1"
rm -r "$output_path" &>/dev/null
mkdir "$output_path" &>/dev/null
if [ "$?" -ne 0 ]; then echo -e "\033[0;31m Failed: mkdir $output_path\n Please create parent path or change directory.\e[0m"; exit; fi