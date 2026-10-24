#!/usr/bin/env bash
set -euo pipefail

ABORT_FILE="/tmp/shelf_abort"
touch "${ABORT_FILE}"
echo "Landing/abort requested through ${ABORT_FILE}."
