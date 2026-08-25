#!/bin/bash
# Launcher: fully detaches the experiment from the calling WSL session.
cd /home/mukund/streaming_baseline || exit 1
setsid nohup python3 -u run_tests.py > console.log 2>&1 < /dev/null &
echo $! > run_tests.pid
sleep 2
if kill -0 "$(cat run_tests.pid)" 2>/dev/null; then
    echo "RUNNING pid=$(cat run_tests.pid)"
else
    echo "FAILED TO STAY UP"
    tail -20 console.log
fi
