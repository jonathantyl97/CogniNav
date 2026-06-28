#!/usr/bin/env bash
# Run all CogniNav gates. Thin alias for CI / quick start.
#
# Usage:
#   ./benchmarks/run_all_gates.sh
#   ./benchmarks/run_all_gates.sh --skip-humble

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/benchmarks/run_gate.sh" --all "$@"
