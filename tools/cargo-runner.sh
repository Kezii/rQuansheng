#!/usr/bin/env bash
set -euo pipefail

ELF="${1:-}"
if [[ -z "${ELF}" || ! -f "${ELF}" ]]; then
  echo "usage: cargo-runner.sh <path-to-elf>" >&2
  exit 2
fi

# Paths / settings (hardcoded on purpose).
LOG_FORMAT="[{t:blue}] {f:>10} {L} {s}"
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN_OUT="${PROJECT_ROOT}/firmware.bin"
K5PROG="${PROJECT_ROOT}/tools/k5prog"
PORT="/dev/ttyUSB0"
BAUD="38400"

command -v defmt-print >/dev/null 2>&1 || { echo "defmt-print not found in PATH" >&2; exit 3; }
command -v rust-objcopy >/dev/null 2>&1 || { echo "rust-objcopy not found in PATH" >&2; exit 3; }
[[ -x "${K5PROG}" ]] || { echo "k5prog not found/executable at ${K5PROG}" >&2; exit 3; }
[[ -e "${PORT}" ]] || { echo "serial port not found: ${PORT}" >&2; exit 3; }

echo "[1/4] cargo size"
(
  cd -- "${PROJECT_ROOT}"
  if [[ "${ELF}" == *"/release/"* ]]; then
    cargo size --release --bin app
  else
    cargo size --bin app
  fi
) || true

echo "[2/4] ELF -> BIN (${BIN_OUT})"
rust-objcopy -O binary "${ELF}" "${BIN_OUT}"

echo "[3/4] Flash"
"${K5PROG}" -b "${BIN_OUT}" -F -YYYYY

echo "[4/4] defmt logs (Ctrl-C to stop)"
stty -F "${PORT}" "${BAUD}" raw -echo || true
cat "${PORT}" | defmt-print --log-format "${LOG_FORMAT}" -e "${ELF}"

