#!/bin/bash
sudo ~/bin/jetson_clocks.sh
rm ctimer_log.txt
gsettings set org.gnome.desktop.peripherals.keyboard delay 80
gsettings set org.gnome.desktop.peripherals.keyboard repeat-interval 80
# roslaunch launch/archive.launch
# roslaunch launch/imitate.launch
roslaunch launch/tf.launch
gsettings set org.gnome.desktop.peripherals.keyboard delay 500
gsettings set org.gnome.desktop.peripherals.keyboard repeat-interval 60
sudo ~/bin/jetson_clocks.sh  --restore
ds=`date +%y%m%d%H%M`
dir=/caffe/ros/src/implay/scripts
mkdir $dir/snar/$ds
cp $dir/snapshots/* $dir/snar/$ds
