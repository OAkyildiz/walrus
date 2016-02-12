#!/bin/bash
#while [ -z "$(rosnode list | grep -q'/base_mainboard')" ] 
#do
	#echo "hi"
#	sleep 0.0001
#done
sleep 10
putty -load back & putty -load front & 
sleep .5
#echo "meow"
pkill putty
#ps aux | grep -i putty | awk {'print $2'} | xargs pkill

