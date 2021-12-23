#!/bin/bash

# Shell script run by cti-serial.service.
# The cti_serial_core driver creates device files /dev/ttyCTI*
# We want to create backwards compatible device files /devttyS*

# For first_port=4 and num_ports=8 this will create the following
# new devices, that should work identically to the CTI devices:
#       CTI             new
#       /dev/ttyCTI0    /dev/ttyS4
#       ...
#       /dev/ttyCTI7    /dev/ttyS11

first_port=4
num_ports=8

# determine major number from driver entry in /proc/devices
drvname=cti_serial_core
procname=ttyCTI

fgrep -q $procname /proc/devices || modprobe $drvname && sleep 5

major=$(fgrep $procname /proc/devices | cut -f 1 -d \  )

if [ -z "$major" ]; then
    echo "$drvname not running, $procname not found in /proc/devices"
    exit 1
fi

for (( minor = 0; minor < $num_ports; minor++ )); do

    port=$(( $minor + $first_port ))

    devfile=/dev/ttyS$port
    echo "devfile=$devfile"

    if [ -L $devfile ]; then
        # symbolic link, leave alone
        :
    elif [ -c $devfile ]; then
        # check major, minor numbers
        devnums=( $(stat --printf "%t %T") )

        if [ ${devnums[0]} = $major && ${devnums[1]} == $minor ]; then
            :
        else
            echo "Recreating $devfile"
            rm -f $devfile
        fi
    elif [ -f $devfile ]; then
        echo "$devfile is not a character file, deleting"
        rm -f $devfile
    fi

    if ! [ -c $devfile -o -L $devfile ]; then
        mknod -m 0666 $devfile c $major $minor
    fi

done
