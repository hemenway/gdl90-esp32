#!/bin/sh
cd "$(dirname "$0")" || exit 1
python3 tools/usb_panel.py "$@" || { echo "Press Return to close."; read -r answer; exit 1; }
