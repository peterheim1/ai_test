#!/bin/bash
SCRIPT_DIR="$(dirname "$0")"
cd "$SCRIPT_DIR/.."
source voice_control/.venv/bin/activate
python -m voice_control.robbie_voice_server "$@"
