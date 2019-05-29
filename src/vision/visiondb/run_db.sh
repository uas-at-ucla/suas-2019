#!/bin/bash

cd "$(dirname "$0")"

if [[ $# != 1 ]]
then
    echo 'Usage: ./run.sh TASK'
    echo 'possible TASKs are:'
    echo 'run - start the vision database backend'
    echo 'clean - PERMANENTLY remove persistent data'
    echo 'export - copy the data out of the docker volume'
fi

if [[ $1 == 'run' ]]
then
    docker-compose up --build
elif [[ $1 == 'clean' ]]
then
    printf '\033[1m\033[91mWARNING: This will PERMANENTLY delete the database!\033[0m\n'
    read -p "Confirm deletion? [y/N]: " deletion_ans
    if [[ "$deletion_ans" == 'y' || "$deletion_ans" == 'Y' ]]
    then
        docker-compose down --volumes
        docker volume rm visiondb
    else
        echo 'Database deletion aborted.'
    fi
elif [[ $1 == 'export' ]]
then
    if [[ ! -d 'export_data' ]]
    then
        mkdir 'export_data'
    fi

    export_prefix=0
    while [[ -e 'export_data/visiondb_'$export_prefix'.sqlite3' ]]
    do
        export_prefix=$((export_prefix + 1))
    done
    export_name='export_data/visiondb_'$export_prefix'.sqlite3'

    docker run --rm -v=visiondb:/visiondb -v=$(pwd)/export_data:/export_data busybox cp visiondb/db.sqlite3 $export_name
    echo 'Successfully exported to "'"$export_name"'"'
fi

