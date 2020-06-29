#!/bin/bash
if [ $# -eq 0 ]
then
	delay=5
else
	delay=$1
fi

figlet "${delay} seconds"
sleep ${delay}
exit 0
