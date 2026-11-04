#!/bin/sh
# Map a host TCP port to the sensor forwarder on the RK3566 over adb.
set -eu

ADB_SERIAL=${ADB_SERIAL:-}
LOCAL_PORT=${LOCAL_PORT:-5000}
DEVICE_PORT=${DEVICE_PORT:-5000}

if [ -n "$ADB_SERIAL" ]; then
    adb -s "$ADB_SERIAL" forward "tcp:${LOCAL_PORT}" "tcp:${DEVICE_PORT}"
else
    adb forward "tcp:${LOCAL_PORT}" "tcp:${DEVICE_PORT}"
fi

echo "adb forward tcp:${LOCAL_PORT} -> device tcp:${DEVICE_PORT}"
