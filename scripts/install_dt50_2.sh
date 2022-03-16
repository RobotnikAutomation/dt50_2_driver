#!/bin/bash
sudo cp -v ../rules/60-dt50-r.rules /etc/udev/rules.d/

cp -v init_dt50_2.sh $HOME/.ros/

sudo cp -v init-dt50-2.service /etc/systemd/system/
sudo systemctl enable init-dt50-2.service
sudo systemctl start init-dt50-2.service
