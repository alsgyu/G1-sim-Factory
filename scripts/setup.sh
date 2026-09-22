#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-python3}"
"$PYTHON_BIN" -c 'import sys; assert (3,10) <= sys.version_info[:2] < (3,13), "Use Python 3.10–3.12"'
"$PYTHON_BIN" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
# CPU wheel keeps CUDA out of the fixed-path simulation environment.
.venv/bin/python -m pip install torch==2.7.1 --index-url https://download.pytorch.org/whl/cpu
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python scripts/fetch_assets.py
echo 'Ready: source .venv/bin/activate; python -m g1_factory.run --lab lab_a'
