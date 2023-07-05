#!/bin/bash


sitl_num=3
[ -n "$1" ] && sitl_num="$1"

declare -a pids

n=0
rec_port=2020
send_port=2019
while [ $n -lt $sitl_num ]; do
	
	echo "starting bridge $n"
	micrortps_agent -t UDP -r $rec_port -s $send_port -n vhcl$n &>/dev/null &
	pids[${i}]=$!

	rec_port=$(($rec_port + 2))
	send_port=$(($send_port + 2))
	n=$(($n + 1))
done

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# wait for all pids
for pid in ${pids[*]}; do
    wait $pid
done