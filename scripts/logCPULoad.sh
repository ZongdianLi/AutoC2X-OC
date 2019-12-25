rm -f sender_cpuLoad.log
while true; do uptime >> sender_cpuLoad.log; sleep 1; done
