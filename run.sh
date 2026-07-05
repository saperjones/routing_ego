#!/usr/bin/env bash
#
# Convenience runner for the parking route-projection project.
#
#   ./run.sh            generate cases and serve the viewer (default)
#   ./run.sh serve      same as above
#   ./run.sh gen        (re)generate out/*.json only
#   ./run.sh test       run the unit + acceptance suite
#   ./run.sh e2e        run the headless-browser end-to-end suite
#   ./run.sh setup      just create the venv and install deps
#
# Override the port with:  PORT=9000 ./run.sh
#
set -euo pipefail
cd "$(dirname "$0")"                 # always operate from the repo root

VENV=.venv
PY="$VENV/bin/python"
PORT="${PORT:-8000}"
URL="http://localhost:${PORT}/viewer/index.html"

ensure_venv() {
    if [ ! -x "$PY" ]; then
        echo ">> creating venv ($VENV) ..."
        python3.12 -m venv "$VENV" 2>/dev/null || python3 -m venv "$VENV"
        "$PY" -m pip install -q --upgrade pip
        "$PY" -m pip install -q -e ".[dev]"
    fi
}

gen() {
    ensure_venv
    echo ">> generating test cases -> out/ ..."
    PYTHONPATH=src "$PY" -m parking_proj.generate
}

gen_real() {
    ensure_venv
    echo ">> generating real-data cases -> out/real/ ..."
    PYTHONPATH=src "$PY" -m parking_proj.generate_real
}

free_port() {
    # kill any process already listening on $PORT so we never hit
    # "Address already in use" from a leftover server
    local pids
    pids=$(lsof -nP -tiTCP:"$PORT" -sTCP:LISTEN 2>/dev/null || true)
    if [ -n "$pids" ]; then
        echo ">> port ${PORT} busy (pids: ${pids//$'\n'/ }) — freeing it"
        kill $pids 2>/dev/null || true
        sleep 1
    fi
}

serve() {
    gen
    free_port
    echo ">> serving at ${URL}"
    echo ">> (Ctrl-C to stop)"
    # open the browser shortly after the server comes up (macOS/Linux best-effort)
    if command -v open >/dev/null 2>&1; then
        ( sleep 1; open "$URL" ) &
    elif command -v xdg-open >/dev/null 2>&1; then
        ( sleep 1; xdg-open "$URL" ) &
    fi
    exec "$PY" -m http.server "$PORT"   # served from the repo ROOT so ../out/ resolves
}

case "${1:-serve}" in
    setup)         ensure_venv; echo ">> venv ready." ;;
    gen|generate)  gen ;;
    gen-real)      gen_real ;;
    serve|"")      serve ;;
    test)          ensure_venv; "$PY" -m pytest -v ;;
    e2e)
        ensure_venv
        "$PY" -m pip install -q -e ".[e2e]"
        "$PY" -m playwright install chromium
        "$PY" -m pytest -m e2e -v
        ;;
    -h|--help|help)
        echo "usage: ./run.sh [serve|gen|gen-real|test|e2e|setup]   (PORT=8000 by default)"
        ;;
    *)
        echo "unknown command: $1" >&2
        echo "usage: ./run.sh [serve|gen|gen-real|test|e2e|setup]" >&2
        exit 1
        ;;
esac
