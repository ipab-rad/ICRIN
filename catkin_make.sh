#!/bin/bash
if [ "$#" -eq 1 ]; then
  if [ "$1" == "msgs" ]; then
    packages=($(ls ./icrin/icrin_msgs/))
  else
    packages=("$1")
  fi
else
  packages=($(ls ./icrin/) $(ls ./icrin/icrin_msgs/) $(ls ./))
fi

white_list=$(printf "%s;" "${packages[@]}")

cd ./../..
catkin_make -DCATKIN_WHITELIST_PACKAGES=$white_list
