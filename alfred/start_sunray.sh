#!/bin/bash

echo "EUID=$EUID"
echo "PWD=$PWD"

CMD=""



if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

# if [ ! -d "/home/pi/Sunray/alfred/build" ]; then
#  echo "install Sunray first!"
#  exit
# fi

if [[ `pidof sunray` != "" ]]; then
  echo "Sunray linux app already running! Exiting..."
  exit
fi


# -----------------------------------------
echo "setup CAN interface..."
echo "NOTE: you may have to edit boot config to enable CAN driver (see https://github.com/owlRobotics-GmbH/owlRobotPlatform)"
ip link set can0 up type can bitrate 1000000
# ----- CAN-USB-bridge (SLCAN) -----------------------------
# sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyACM0
# sudo ip link set up slcan0

# -----------------------------------------
echo "----bluetooth devices----"
systemctl enable bluetooth.service
hcitool dev
# configure bluetooth BLE module
echo "----BLE config----"
echo 12 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval  # 24   6
echo 20 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval  # 40   6
echo 1 > /sys/kernel/debug/bluetooth/hci0/conn_latency       # 0    1
echo 153 > /sys/kernel/debug/bluetooth/hci0/adv_min_interval  # 0.625 ms units
echo 153 > /sys/kernel/debug/bluetooth/hci0/adv_max_interval  # 0.625 ms units
btmgmt -i hci0 power off
btmgmt -i hci0 le on
btmgmt -i hci0 bredr off
btmgmt -i hci0 connectable on
#btmgmt -i hci0 name "bananapi $MAC"
#btmgmt -i hci0 name "owlMower"
BLE_NAME=`grep "BLE_NAME" ../sunray/config.h | cut -d'"' -f 2`
echo "BLE_NAME=$BLE_NAME"
btmgmt -i hci0 name "$BLE_NAME"
btmgmt -i hci0 advertising on
btmgmt -i hci0 power on
## btmgmt -i hci0 advinfo
# -----------------------------------------


# setup audio interface
# https://gavv.net/articles/pulseaudio-under-the-hood/
# https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/User/SystemWide/
# https://unix.stackexchange.com/questions/204522/how-does-pulseaudio-start
if ! command -v play &> /dev/null
then 
  echo "installing audio player..."
  apt install -y libsox-fmt-mp3 sox mplayer alsa-utils pulseaudio
fi
# show audio devices
#aplay -l
cat /proc/asound/cards
# restart pulseaudio daemon as root
#killall pulseaudio
#sleep 1
#pulseaudio -D --system --disallow-exit --disallow-module-loading --verbose
#sudo chmod 666 /var/run/pulse/native
# set default volume 
adduser root dialout audio pulse-access pulse
amixer -D pulse sset Master 100%


echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 20`; do 
  RES=$(netstat -ant | grep -w 80 | grep LISTEN)
  #RES=$(lsofs -i:80)
  #RES=$(fuser 80/tcp) 
  if [ -z "$RES" ]; then
    break
  fi
  echo $RES
  PID=`fuser 80/tcp | cut -d' ' -f 2`
  echo "PID:$PID"
  ls -l /proc/$PID/exe  
  # echo -n .  
  sleep 2.0     
done; 



echo "----starting sunray----"
echo "CMD=$CMD"

echo "working dir:$PWD"    

#if [ -d "/home/pi/Sunray/alfred/build" ]; then
  # state and map files will be written here 
#  cd /boot/sunray
  # pick sunray from here
  /usr/bin/stdbuf -oL -eL /home/pi/Sunray/alfred/build/sunray
#else
  # pick sunray from here  
#  /usr/bin/stdbuf -oL -eL $PWD/build/sunray
#fi 

# debug mode
# exec gdbserver :1234 /home/pi/sunray_install/sunray "$@"
# debug executable with:  gdb /home/pi/sunray_install/sunray

