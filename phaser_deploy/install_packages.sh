#!/usr/bin/env zsh

apt -y install clang-format-6.0 ros-melodic-pcl-conversions \
  libpcl-dev libnlopt-dev ros-melodic-pcl-ros wget

  wstool init
  wstool merge phaser/dependencies.rosinstall
  wstool update