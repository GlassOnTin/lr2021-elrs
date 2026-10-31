#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-or-later
# Flash firmware ELF/BIN to a Seeed Studio XIAO nRF54L15 over CMSIS-DAPv2 SWD.
# Usage: ./scripts/flash.sh <serial_number> [path/to/firmware.elf]

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERIAL="${1:-}"
IMAGE="${2:-${REPO_ROOT}/build/lr2021_elrs.elf}"

if [[ -z "${SERIAL}" ]]; then
    echo "Usage: $0 <adapter_serial> [elf_or_bin_path]" >&2
    echo "Available XIAO nRF54L15 CMSIS-DAP probes:" >&2
    ls -1 /dev/serial/by-id/*XIAO_nrf54_CMSIS-DAP* 2>/dev/null || true
    exit 1
fi

if [[ ! -f "${IMAGE}" ]]; then
    echo "Error: firmware image not found at ${IMAGE}" >&2
    exit 1
fi

USB_LOC=""
for f in /sys/bus/usb/devices/*/serial; do
    if [[ -r "$f" ]] && [[ "$(cat "$f" 2>/dev/null || true)" == "${SERIAL}" ]]; then
        USB_LOC="$(basename "$(dirname "$f")")"
        break
    fi
done

LOC_ARGS=(-c "adapter serial ${SERIAL}")
if [[ -n "${USB_LOC}" ]]; then
    echo "Resolved ${SERIAL} at USB location ${USB_LOC}"
    LOC_ARGS+=(-c "adapter usb location ${USB_LOC}")
fi

openocd \
    "${LOC_ARGS[@]}" \
    -f "${REPO_ROOT}/openocd/xiao_nrf54l15.cfg" \
    -c "init; nrf54l_flash_image ${IMAGE}; shutdown"
