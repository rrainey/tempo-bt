#!/usr/bin/env bash
#
# dfu-upgrade.sh - Upgrade a Tempo-BT device over BLE OTA DFU and verify it.
#
# Performs the full validated dual-core (nRF5340) DFU flow for one device:
#   resolve by name -> baseline -> stage app+net images -> mark pending ->
#   reset -> re-discover by name -> verify the running image.
#
# It is version-agnostic: the "target" image hash is taken from whatever was just
# staged, so it works for any DFU package, not a hard-coded firmware version.
#
# Usage:
#   tools/dfu-upgrade.sh <NNNN> [dfu_application.zip]
#     <NNNN>              4-char device suffix, e.g. 0006 (device Tempo-BT-0006)
#     [dfu_application.zip]  DFU package; default: <repo>/build/dfu_application.zip
#
# A device already running the target image is a no-op. Re-applying the SAME firmware
# version over OTA is not possible (the app image hash equals the running image, so it
# cannot be marked pending in the secondary slot) — use SWD/J-Link to re-flash same-version.
#
# Requirements: smpmgr (native image/os groups only), bluetoothctl, python3, unzip.
#
# WARNING: the firmware is built overwrite-only (no rollback). A failed net-core
# update can leave the device unreachable over BLE; recover with a wired SWD/J-Link
# re-flash of build/merged.hex + build/merged_CPUNET.hex. Prefer an SWD-accessible
# unit for first-time or risky updates. Flight logs are not touched.
#
# See README.md "Firmware Update over BLE (OTA DFU)" and docs/dfu-ota-test-plan.md.
set -u

SUFFIX="${1:?usage: dfu-upgrade.sh <NNNN> [dfu_application.zip]}"
NAME="Tempo-BT-${SUFFIX}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
ZIP="${2:-$REPO/build/dfu_application.zip}"
[ -f "$ZIP" ] || { echo "ERROR: DFU package not found: $ZIP"; exit 1; }

WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
unzip -o -q "$ZIP" -d "$WORK" || { echo "ERROR: could not unzip $ZIP"; exit 1; }
APP_BIN="$WORK/tempo-bt-v1.signed.bin"
NET_BIN="$WORK/ipc_radio.bin"
[ -f "$APP_BIN" ] && [ -f "$NET_BIN" ] || { echo "ERROR: package missing app/net images"; exit 1; }

say(){ echo "── $* ──"; }

# Parse smpmgr `image state-read` -> "slot image HASH active confirmed pending" per state.
parse(){ python3 "$HERE/dfu_state_parse.py"; }

# One scan pass for $NAME -> address (empty if not seen). Used while polling for a swap.
scan1(){ timeout 9 bluetoothctl --timeout 7 scan on 2>/dev/null \
    | grep -oE "[0-9A-F:]{17} $NAME\b" | grep -oE "^[0-9A-F:]{17}" | head -1
  bluetoothctl scan off >/dev/null 2>&1; }

# Resolve $NAME to a BLE address (MAC re-randomizes per power-on, so always by name).
resolve(){ local a i
  for i in 1 2 3 4 5 6; do
    a=$(timeout 10 bluetoothctl --timeout 8 scan on 2>/dev/null \
        | grep -oE "[0-9A-F:]{17} $NAME\b" | grep -oE "^[0-9A-F:]{17}" | head -1)
    [ -n "$a" ] && { echo "$a"; return 0; }
    bluetoothctl scan off >/dev/null 2>&1; sleep 1
  done
  return 1
}

# Retry image state-read until it returns parsed lines.
read_states(){ local out i
  for i in 1 2 3 4 5; do
    out=$(smpmgr --ble "$1" image state-read 2>/dev/null | parse)
    [ -n "$out" ] && { echo "$out"; return 0; }
    sleep 2
  done
  return 1
}

say "$NAME : resolving address"
DEV=$(resolve) || { echo "FAIL: $NAME not found in scan"; exit 2; }
echo "addr=$DEV"

say "$NAME : baseline"
smpmgr --ble "$DEV" os echo hello >/dev/null 2>&1 || { echo "FAIL: os echo (baseline)"; exit 3; }
OLD=$(read_states "$DEV" | awk '$1==0&&$2==0{print $3}')
echo "running (old) app hash: ${OLD:-?}"

say "$NAME : upload app (image 0)"; t=$SECONDS
smpmgr --timeout 30 --ble "$DEV" image upload "$APP_BIN" --slot 0 >/dev/null 2>&1 || { echo "FAIL: app upload"; exit 4; }
echo "app upload: $((SECONDS-t))s"
say "$NAME : upload net (image 1)"; t=$SECONDS
smpmgr --timeout 30 --ble "$DEV" image upload "$NET_BIN" --slot 1 >/dev/null 2>&1 || { echo "FAIL: net upload"; exit 4; }
echo "net upload: $((SECONDS-t))s"

say "$NAME : verify staged"
STAGED=$(read_states "$DEV") || { echo "FAIL: state-read (verify staged) no response"; exit 5; }
echo "$STAGED"
TARGET=$(echo "$STAGED" | awk '$1==1&&$2==0{print $3}')   # newly-staged app hash = upgrade target
SN=$(echo "$STAGED" | awk '$1==1&&$2==1{print $3}')
[ -n "$TARGET" ] && [ -n "$SN" ] || { echo "FAIL: app/net image not staged"; exit 5; }
if [ "$TARGET" = "$OLD" ]; then echo "NOTE: device already running this image; nothing to do"; exit 0; fi

# nRF5340 simultaneous update: each image is pended by a state-write on ITS OWN hash
# (app hash pends the app image, net hash pends the net image). Both are required.
# Rapid BLE reconnects can drop a request, so write both each attempt and verify.
say "$NAME : mark pending + reset (COMMIT — point of no return)"
PEND=0
for attempt in 1 2 3 4; do
  smpmgr --ble "$DEV" image state-write "$TARGET" false >/dev/null 2>&1; sleep 1
  smpmgr --ble "$DEV" image state-write "$SN" false >/dev/null 2>&1; sleep 1
  PEND=$(read_states "$DEV" | awk '$6=="True"' | wc -l)
  echo "attempt $attempt: images pending=$PEND (expect 2)"
  [ "$PEND" -ge 2 ] && break
  sleep 2
done
[ "$PEND" -ge 2 ] || { echo "FAIL: pending flags not set"; exit 6; }

# The reboot performs the swap: MCUboot overwrites the app slot and reprograms the net
# core (~30-45 s, longer than a plain reboot). `os reset` can silently fail to trigger, so
# we confirm the swap by polling the running hash and re-issue the reset if it didn't take.
#
# IMPORTANT: poll by CONNECTING to the known address (bleak connect is reliable), not by
# bluetoothctl scanning (which is flaky, especially with multiple BT controllers present).
# These devices keep their BLE MAC across reboots, so the address stays valid; only if
# direct polling never succeeds do we fall back to re-scanning by name.
say "$NAME : apply (reset) + wait for swap"
DEV2="$DEV"; RUN=""
for reset_try in 1 2 3; do
  smpmgr --ble "$DEV2" os reset >/dev/null 2>&1
  echo "reset issued (try $reset_try) to $DEV2; waiting for swap (~30-45 s)…"; t=$SECONDS
  while [ $((SECONDS-t)) -lt 75 ]; do
    RUN=$(smpmgr --timeout 4 --ble "$DEV2" image state-read 2>/dev/null \
          | python3 "$HERE/dfu_state_parse.py" | awk '$1==0&&$2==0{print $3}')
    [ -n "$RUN" ] && echo "  t+$((SECONDS-t))s running=${RUN:0:16}…"
    [ "$RUN" = "$TARGET" ] && break
  done
  [ "$RUN" = "$TARGET" ] && break
  echo "swap not confirmed on $DEV2; re-resolving by name in case the MAC changed…"
  A=$(resolve) && DEV2="$A"
done
[ "$RUN" = "$TARGET" ] || { echo "FAIL: $NAME did not apply update after 3 resets — check via SWD"; exit 9; }

say "$NAME : verify"
FINAL=$(read_states "$DEV2") || { echo "FAIL: state-read post-DFU no response"; exit 8; }
echo "$FINAL"
AC=$(echo "$FINAL" | awk '$1==0&&$2==0{print "active="$4,"confirmed="$5}')
# Best-effort confirmation that the assigned name is advertising (scan is flaky → non-fatal;
# the device is connectable, which already implies it is advertising).
seen=""; for k in 1 2 3; do s=$(scan1); [ -n "$s" ] && { seen="$s"; break; }; done
if [ -n "$seen" ]; then echo "name '$NAME' broadcasting at $seen ✓"
else echo "name '$NAME' not caught in a scan pass, but device is connectable (advertising)"; fi
echo "RESULT $NAME: PASS — running=$RUN ($AC); firmware upgraded"
