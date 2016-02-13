#!/bin/bash

while [ "$(rosnode list | grep /base_mainboard)" != "/base_mainboard" ] 
do
sleep 0.0001
done
putty -load front &  putty -load back &
sleep .05

ps aux | grep -i putty | awk {'print $2'} | xargs kill
echo "hi"
exit
