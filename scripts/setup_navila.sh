#!/usr/bin/env bash
# Optional GPU model environment. Run from any directory; never touches .venv.
set -euo pipefail

G1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
G1_NAVILA_PYTHON="${G1_NAVILA_PYTHON:-python3.10}"
G1_NAVILA_REV="76b98f233dd0fff05dfcd69435eec6740febff9d"
G1_MODEL_REV="b2294e96581454468d6b94f38201f4f965ef48b7"
G1_ENV="$G1_ROOT/.venv-navila"
G1_UPSTREAM="$G1_ROOT/third_party/NaVILA"

if [[ "${1:-}" != "" && "${1:-}" != "--download" ]]; then
    echo "Usage: bash scripts/setup_navila.sh [--download]" >&2
    exit 2
fi
"$G1_NAVILA_PYTHON" -c 'import sys,platform; assert sys.version_info[:2] == (3,10), "Requires Python 3.10"; assert platform.system() == "Linux" and platform.machine() == "x86_64", "Requires Ubuntu x86_64"'
command -v git >/dev/null
command -v nvidia-smi >/dev/null || { echo "Install an NVIDIA driver before NaVILA setup." >&2; exit 1; }
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

if [[ ! -d "$G1_ENV" ]]; then
    "$G1_NAVILA_PYTHON" -m venv "$G1_ENV"
fi
"$G1_ENV/bin/python" -c 'import sys; assert sys.version_info[:2] == (3,10), "Existing NaVILA venv is not Python 3.10"'
mkdir -p "$G1_ROOT/third_party"
if [[ ! -e "$G1_UPSTREAM" ]]; then
    git init "$G1_UPSTREAM"
    git -C "$G1_UPSTREAM" remote add origin https://github.com/AnjieCheng/NaVILA.git
    git -C "$G1_UPSTREAM" fetch --depth 1 origin "$G1_NAVILA_REV"
    git -C "$G1_UPSTREAM" checkout --detach FETCH_HEAD
fi
if [[ "$(git -C "$G1_UPSTREAM" rev-parse HEAD)" != "$G1_NAVILA_REV" ]]; then
    echo "third_party/NaVILA has another revision. Preserve it and choose a clean checkout." >&2
    exit 1
fi

"$G1_ENV/bin/python" -m pip install 'pip<25' 'setuptools==69.5.1' wheel packaging ninja 'numpy==1.26.4'
"$G1_ENV/bin/python" -m pip install torch==2.3.0 torchvision==0.18.0 --index-url https://download.pytorch.org/whl/cu121
# Match the wheel linked by the pinned upstream environment_setup.sh.
"$G1_ENV/bin/python" -m pip install 'https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.8/flash_attn-2.5.8+cu122torch2.3cxx11abiFALSE-cp310-cp310-linux_x86_64.whl'
DS_BUILD_OPS=0 "$G1_ENV/bin/python" -m pip install -e "$G1_UPSTREAM[train]"
# Direct inference needs no Habitat, Isaac, Matterport, or lmms-eval installation.
"$G1_ENV/bin/python" -m pip install 'huggingface_hub==0.23.4' 'Pillow>=10,<12'
G1_SITE="$("$G1_ENV/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
cp -r "$G1_UPSTREAM/llava/train/transformers_replace/." "$G1_SITE/transformers/"
cp -r "$G1_UPSTREAM/llava/train/deepspeed_replace/." "$G1_SITE/deepspeed/"
"$G1_ENV/bin/python" -c 'import torch, flash_attn; from llava.model.builder import load_pretrained_model; assert torch.cuda.is_available(); print("NaVILA imports and CUDA are available")'

if [[ "${1:-}" == "--download" ]]; then
    "$G1_ENV/bin/python" - "$G1_ROOT" "$G1_MODEL_REV" <<'PY'
import sys
from pathlib import Path
from huggingface_hub import snapshot_download
snapshot_download(
    "a8cheng/navila-llama3-8b-8f", revision=sys.argv[2],
    local_dir=str(Path(sys.argv[1]) / "models" / "navila-llama3-8b-8f"),
)
PY
fi
echo "NaVILA environment ready. See docs/VLN.md for the server and simulation commands."
