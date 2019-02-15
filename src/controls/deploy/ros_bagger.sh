#!/bin/bash

set -e
cd "$(dirname "$0")/../"
mkdir -p logs
cd logs

OLD_LOGS=$(ls -t -d */ | awk 'NR>10' | awk '{ for (i=NF; i>1; i--) printf("%s ",$i); print $1; }' | tac)
OLD_LOGS="$OLD_LOGS"

# Free up space for logfiles.
while [ $(df --output=pcent / | tr -dc '0-9') -gt 80 ]
do
	if [ $(echo $OLD_LOGS | wc -w) -le 1 ]
	then
		break
	fi

	OLDEST_LOG_FOLDER=$(echo $OLD_LOGS | awk '{print $1;}')
	OLD_LOGS=$(echo $OLD_LOGS | cut -d' ' -f2- -)
	echo "Low on space; deleting logs in $OLDEST_LOG_FOLDER"
	rm -rf $OLDEST_LOG_FOLDER

	sleep 0.1
done

DATE=$(date)
DATE=${DATE// /_}
mkdir $DATE

cd $DATE

rosbag record -a --duration=15m --split --max-splits=16 --lz4

