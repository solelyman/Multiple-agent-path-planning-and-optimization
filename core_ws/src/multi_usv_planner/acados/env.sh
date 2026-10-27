#!/usr/bin/env bash
set -euo pipefail

# Minimal environment helper for local acados integration.
# Edit ACADOS_SOURCE_DIR below after installing acados.

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/.local/share/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/build/acados:$ACADOS_SOURCE_DIR/build/external/hpipm:$ACADOS_SOURCE_DIR/build/external/blasfeo:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
export ACADOS_PYTHON="$ACADOS_SOURCE_DIR/venv/bin/python"

echo "ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR"
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
echo "PYTHONPATH=$PYTHONPATH"
python3 - <<'PY'
mods = ['casadi', 'acados_template']
for m in mods:
    try:
        __import__(m)
        print(m + ': OK')
    except Exception as e:
        print(m + ': MISSING -> ' + str(e))
PY
