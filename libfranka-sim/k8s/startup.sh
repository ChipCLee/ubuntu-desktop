#!/bin/bash

# Start the X window manager (e.g., XFCE4)
/usr/bin/startxfce4 &

# Start the VNC server in the background
x11vnc -display :1 -forever -noipv6 -usepw &

# Keep the container running in the foreground by running the passed command or a dummy process
# exec "$@"
# or
tail -f /dev/null