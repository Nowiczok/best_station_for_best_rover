#!/bin/sh
# skrypt do łączenia się po donglu do kalmana

touch virtualcom0

sudo socat -d -d tcp:192.168.1.45:8899 pty,link=virtualcom0 &

sudo python main.py --port virtualcom0

sudo killall socat