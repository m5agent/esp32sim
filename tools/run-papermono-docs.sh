#!/usr/bin/env bash
set -euo pipefail
exec python3 "$(dirname "$0")/run-paper-docs.py" papermono "$@"
