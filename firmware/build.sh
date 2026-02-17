#!/bin/bash

# Use the first argument if provided, otherwise find the first usbmodem
PORT="${1:-$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1)}"

if [ -z "$PORT" ]; then
    echo "Error: No USB modem found and no port specified."
    exit 1
fi

echo "Using port: $PORT"

idf.py -p "$PORT" build flash monitor