#!/bin/zsh
# Build and flash the McLaren ECU.
#   ./flash.sh classic usb      first flash over USB (auto-finds the port)
#   ./flash.sh classic ota      update over Wi-Fi (garage mode), no cable
#   ./flash.sh s3 usb | s3 ota  same for the ESP32-S3
# Optional: OTA_HOST=192.168.1.143 ./flash.sh classic ota   (default: mclaren.local)
set -euo pipefail
cd "${0:A:h}"
board=${1:-classic}; how=${2:-usb}
case $board in
  classic) fqbn=esp32:esp32:esp32 ;;
  s3)      fqbn=esp32:esp32:esp32s3 ;;
  *) echo "board must be classic or s3"; exit 1 ;;
esac
cli=$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")
[ -f mclaren_ecu/secrets.h ] || { echo "missing mclaren_ecu/secrets.h (copy secrets.example.h)"; exit 1; }
out=/tmp/mclaren-build-$board
"$cli" compile --fqbn $fqbn --output-dir $out mclaren_ecu | tail -2

if [ "$how" = usb ]; then
  port=${PORT:-$(ls /dev/cu.usbserial-* /dev/cu.usbmodem* /dev/cu.wchusbserial* 2>/dev/null | head -1)}
  [ -n "$port" ] || { echo "no USB board found"; exit 1; }
  echo "flashing $board on $port"
  "$cli" upload -p "$port" --fqbn $fqbn --input-dir $out mclaren_ecu
else
  host=${OTA_HOST:-mclaren.local}
  pw=$(python3 -c "import re;print(re.search(r'OTA_PASSWORD\s+\"([^\"]+)\"',open('mclaren_ecu/secrets.h').read()).group(1))")
  espota=$(ls ~/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/espota.py | tail -1)
  echo "updating $board at $host over Wi-Fi (motors are forced off during the update)"
  (cd /tmp && python3 -I "$espota" -i "$host" -p 3232 --auth="$pw" -f $out/mclaren_ecu.ino.bin)
  for i in {1..8}; do sleep 3; s=$(curl -s -m 3 "http://$host/status" || true); [ -n "$s" ] && { echo "running: $s"; exit 0; }; done
  echo "update sent; car not answering yet (check http://$host/status)"
fi
