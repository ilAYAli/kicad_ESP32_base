#!/bin/bash
set -e

REMOTE_USER="petter"
REMOTE_HOST="31.97.149.235"
REMOTE_PATH="/var/www/wahlman.no/firmware/esp32base"

# Generate version and update ota.h
NEW_VER=$(date +"%y%m%d%H%M")
CURRENT_VER=$(grep "define OTA_CURRENT_VERSION" src/ota.hpp | grep -oE "[0-9]+")

echo "Deploying v$NEW_VER (was v$CURRENT_VER)"

# Update version
if [[ "$OSTYPE" == "darwin"* ]]; then
    sed -i '' "s/OTA_CURRENT_VERSION ${CURRENT_VER}LL/OTA_CURRENT_VERSION ${NEW_VER}LL/" src/ota.hpp
else
    sed -i "s/OTA_CURRENT_VERSION ${CURRENT_VER}LL/OTA_CURRENT_VERSION ${NEW_VER}LL/" src/ota.hpp
fi

# Build
echo "Building..."
idf.py build || { rm -rf build/ && idf.py build; }

# Find the app binary
BIN="build/firmware.bin"
[ ! -f "$BIN" ] && { echo "Error: $BIN not found"; exit 1; }

echo "Found binary: $(du -h "$BIN" | cut -f1)"

# Upload
echo "Uploading..."
ssh $REMOTE_USER@$REMOTE_HOST "rm -rf $REMOTE_PATH/firmware.bin && mkdir -p $REMOTE_PATH"
echo -n "$NEW_VER" > version.txt
scp version.txt "$BIN" $REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/
ssh $REMOTE_USER@$REMOTE_HOST "cd $REMOTE_PATH && chmod 644 version.txt firmware.bin"

echo "✓ Done: https://wahlman.no/firmware/esp32base/firmware.bin"
rm version.txt
