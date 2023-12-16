#!/bin/sh
password="maker"
echo "$password" | sudo pkill pybricks-microp
sudo pkill brickman
bash /home/robot/Commands/brickman.sh