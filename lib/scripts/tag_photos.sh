#!/bin/bash

while true;do
	sleep 5;

	declare -A JSON_TO_TIME
	declare -A JSON_TO_FILE

	GREP_PATTERN=""

	# Generate search patterns for image times.
	for FILENAME in ~/pictures/drone_code_dslr_mounted/store_00020001/DCIM/100CANON/*.JPG;do
		JSON_FILE="${FILENAME%.*}.json"
		BASENAME="$(basename $FILENAME)"

		if [ -f "${JSON_FILE}" ];then
			continue;
		fi

		TIME="$(date +%s -r $FILENAME)";
		#TODO(comran): Tune the offset between camera and RasPi.
		let TIME="$TIME+25199";

		JSON_TO_TIME[$JSON_FILE]=$TIME
		JSON_TO_FILE[$JSON_FILE]=$FILENAME

		GREP_PATTERN="$GREP_PATTERN^.\{1,15\}$TIME\.";
		if [ "$GREP_PATTERN" != "" ];then
			GREP_PATTERN="$GREP_PATTERN\|";
		fi
	done

	if [ "$GREP_PATTERN" = "" ]; then
		continue
	fi

	GREP_PATTERN=${GREP_PATTERN::-4}

	MATCHES="$(cat ~/logs/uas_at_ucla/drone_code*.csv | grep -a ${GREP_PATTERN} | grep SENSORS)"

	for JSON_FILE in ${!JSON_TO_TIME[@]}; do
		FILE=${JSON_TO_FILE[$JSON_FILE]}
		TIME=${JSON_TO_TIME[$JSON_FILE]}
		GREP_PATTERN="^.\{1,15\}$TIME\."
		LOG_LINE="$(echo "$MATCHES" | grep -a ${GREP_PATTERN} | grep SENSORS | tail -1)"

		if [ "$LOG_LINE" = "" ];then
			break
		fi


		LATITUDE=$(echo $LOG_LINE | sed -ne 's/.* Latitude: \(.*\) Longitude: .*/\1/p')
		LONGITUDE=$(echo $LOG_LINE | sed -ne 's/.* Longitude: \(.*\) Altitude: .*/\1/p')
		RELATIVE_ALTITUDE=$(echo $LOG_LINE | sed -ne 's/.* RelativeAltitude: \(.*\) Heading: .*/\1/p')
		HEADING=$(echo $LOG_LINE | sed -ne 's/.* Heading: \(.*\) AccelX: .*/\1/p')

		if [ "$LATITUDE" = "" ];then
			LATITUDE=0
		fi
		if [ "$LONGITUDE" = "" ];then
			LONGITUDE=0
		fi
		if [ "$RELATIVE_ALTITUDE" = "" ];then
			RELATIVE_ALTITUDE=0
		fi
		if [ "$HEADING" = "" ];then
			HEADING=0
		fi

		JSON="{time: $TIME, file: $FILE, latitude: $LATITUDE, longitude: $LONGITUDE, relative_altitude: $RELATIVE_ALTITUDE, heading: $HEADING}"

		echo $JSON > $JSON_FILE
	done
done

