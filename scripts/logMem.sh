#!/bin/bash -e
rm -f sender_memory.log
echo "      date     time $(free -m | grep total | sed -E 's/^    (.*)/\1/g')" >> sender_memory.log
while true; do
    echo "$(date '+%Y-%m-%d %H:%M:%S') $(free -m | grep Mem: | sed 's/Mem://g')" >> sender_memory.log
    sleep 1
done
