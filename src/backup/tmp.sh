#!/bin/bash
device="/dev/video0"  # Replace with your device path

function all_default() {
    while read this_train; do
        parameter=$( sed -rn 's/^ *([^ ]*) .*/\1/p;' <<< "$this_train")
        default=$( sed -rn "s/^.* default=([^ ]*) ?.*/\1/p;" <<< "$this_train")
        echo "Setting $parameter to default value: $default"
        v4l2-ctl -d $device --set-ctrl=$parameter=$default
    done <<< "$(v4l2-ctl -d $device --list-ctrls | sed -r '/:/!d;')"
}

all_default
