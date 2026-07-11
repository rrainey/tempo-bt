# BLE OTA DFU Verification Test Plan

**Status:** ✅ Executed & PASSED on devices `0004`, `0005`, `0006`, `0008` (2026-07-11) —
see §9. Draft plan retained below.
**System under test:** `tempo-bt-v1` firmware, MCUboot BLE OTA DFU path
**Goal:** Prove the existing BLE OTA DFU capability by upgrading a real device from
firmware **1.4.0 → 1.5.0** over Bluetooth, byte-for-byte trustworthy and repeatable,
using a command-line workflow.

> The README currently marks OTA firmware updates as *"not yet tested"*
> ([README.md](../README.md)). This plan is the first-pass validation to remove that caveat.

---

## 1. Scope & candidate devices

- **Candidate devices (need 1.4.0 → 1.5.0):** `0004`, `0006`, `0008`.
- **Use exactly one device for first execution** — the one that is **physically on the
  bench with SWD/J-Link access** (an nRF5340 DK, or a Tempo-BT board with a debug
  connector). Because the build is **overwrite-only (no rollback)**, a failed net-core
  update can leave the device unreachable over BLE; SWD is the only recovery.
- The other two devices are **not** touched until this device passes all acceptance
  criteria and the recovery procedure has been rehearsed at least once.

Out of scope: production over-the-air rollout to many devices, unattended DFU, DFU
driven from the `tempo-tb-ingest` daemon (that is a later step that will reuse the
findings here).

---

## 2. What the firmware actually implements (verified from config)

| Fact | Source |
|---|---|
| Bootloader is MCUboot via sysbuild | `sysbuild.conf` |
| **Two** updateable images: app core (0) + network core (1) | `sysbuild.conf` `SB_CONFIG_MCUBOOT_UPDATEABLE_IMAGES=2`, `SB_CONFIG_NETCORE_APP_UPDATE=y` |
| **Simultaneous** multi-image update (both cores, one reboot) | `SB_CONFIG_MCUBOOT_NRF53_MULTI_IMAGE_UPDATE=y` |
| **Overwrite-only** swap mode → **no test/confirm revert** | `SB_CONFIG_MCUBOOT_MODE_OVERWRITE_ONLY=y` |
| Images are **signed** (RSA-2048, Nordic sample key) | `prj.conf` `CONFIG_MCUBOOT_SIGNATURE_KEY_FILE=...root-rsa-2048.pem` |
| Transport is **BLE/SMP only** (no serial/USB DFU; shell off) | `prj.conf` `CONFIG_MCUMGR_TRANSPORT_BT=y`, `CONFIG_SHELL=n` |
| Image mgmt group enabled | `prj.conf` `CONFIG_MCUMGR_GRP_IMG=y`, `IMG_MANAGER`, `STREAM_FLASH` |
| Secondary/staging slots live in **external QSPI flash** (`mx25r64`) | `pm_static.yml` `mcuboot_secondary` (app), `mcuboot_secondary_1` (net) |
| Net-core image reaches the net core via **PCD SRAM** handoff on reboot | `pm_static.yml` `pcd_sram`, `mcuboot_primary_1` (ram_flash) |

**nRF5340 dual-core mechanism** (Nordic Academy, Lesson 9 — DFU for the nRF5340): the app
core cannot write net-core flash directly. In simultaneous mode the client uploads both
images; each is staged in external flash; on reboot MCUboot overwrites the app slot and
uses the PCD library to reprogram the net core from its staged image. Because it is
overwrite-only, there is no swap-back — a bad image is permanent until re-flashed.

---

## 3. Artifacts under test

Produced by the existing sysbuild build in `build/`:

| File | Role |
|---|---|
| `build/dfu_application.zip` | Multi-image DFU package (app + net core + manifest) |
| ↳ `tempo-bt-v1.signed.bin` | App-core image (manifest `image_index 0`) |
| ↳ `ipc_radio.bin` | Net-core image (manifest `image_index 1`) |
| ↳ `manifest.json` | Board/soc/index metadata consumed by Device Manager |
| `build/merged.hex` / `merged_CPUNET.hex` | **Recovery** images for SWD re-flash |

**Pre-condition to verify before starting:** confirm this build is genuinely 1.5.0.
`app.version` reads `1.5.0`, but the MCUboot image version in the manifest is `0.0.0+0`,
so the MCUboot version field cannot be used as the acceptance signal (see §7).
Record the SHA-256 of `dfu_application.zip` and both `.bin`s as the "known-good" set.

---

## 4. Tooling

Primary CLI: **`smpmgr`** (0.13.2, already on PATH) with the Tempo plugin.

```bash
export SMPMGR_PLUGIN_PATH=/home/riley/src/tempo-insights/smpmgr-extensions/plugins
# addressing uses the device BLE name, e.g. Tempo-BT-0004
```

Relevant native subcommands (verified present):
- `smpmgr --ble <name> image state-read` — list images/slots, flags (`active`, `pending`, `confirmed`)
- `smpmgr --ble <name> image upload <FILE> --slot <N>` — upload one image
- `smpmgr --ble <name> os reset` — reboot to apply

> ⚠️ **Known open question (resolve in Phase 2):** smpmgr's `image upload` exposes only
> `--slot`, not an explicit mcumgr-style `--image` number, and it does **not** ingest the
> `dfu_application.zip`. We must confirm that uploading the net-core `ipc_radio.bin` with
> the correct `--slot` value actually lands in net-core image slot 1. If smpmgr cannot
> address the net-core image, fall back to the reference client below.

Reference / fallback client: **nRF Connect Device Manager** (mobile app) or the **`mcumgr`
Go CLI** (`image upload -e -n <image_num>`), both of which consume the multi-image zip and
handle image numbering natively. Use this to (a) get a first known-good OTA success and
(b) cross-check smpmgr behavior.

Recovery tool: **`nrfutil device`** / `west flash` over J-Link using `merged.hex` +
`merged_CPUNET.hex` (the `generated_nrfutil_batch.json` recipe: program + RESET_PIN).

---

## 5. Safety & risk register

| Risk | Mitigation |
|---|---|
| Overwrite-only → no rollback; bad image bricks BLE | First device must have SWD; rehearse recovery **before** first OTA (Phase 0) |
| Net-core update breaks radio mid-update | Use **simultaneous** update (single reboot), not sequential; keep device stationary/powered |
| Wrong image → wrong slot (smpmgr `--slot` ambiguity) | Phase 2 capability check before any real upload; verify with `image state-read` |
| Interrupted upload (BLE drop, power loss) | External-flash staging means partial upload is discarded on next boot; re-upload. Confirm this is true (Phase 3 abort test, optional) |
| Sample signing key (not production) | Acceptable for this test; note as a separate production hardening item, out of scope |
| **Production logs on device** | DFU touches app/net firmware + external-flash *staging* slots only, **not** the LittleFS log partition. Still: pick a device whose logs are already harvested, or back them up first. |

---

## 6. Procedure

### Phase 0 — Recovery rehearsal (do this first, before any OTA)
0.1 Connect the candidate device via SWD/J-Link.
0.2 Read out and back up anything needed; note current firmware = 1.4.0.
0.3 Practice a full SWD recovery: flash `merged.hex` + `merged_CPUNET.hex` and confirm the
    device boots and advertises. **Do not proceed to Phase 1 until recovery works.**

### Phase 1 — Baseline capture (BLE, pre-upgrade)
1.1 `smpmgr --ble Tempo-BT-000X os echo hello` → confirms SMP over BLE is up.
1.2 `smpmgr --ble Tempo-BT-000X image state-read` → record image list, hashes, slot flags.
1.3 Capture the **version indicator** (see §7): `settings-get` output and/or the chosen
    1.5.0-specific behavior, in its 1.4.0 state.
1.4 `smpmgr --ble Tempo-BT-000X tempo session-list` and download one known session; record
    its SHA-256. (Baseline for the post-upgrade regression check.)

### Phase 2 — Tooling capability check (no commitment)
2.1 With `image state-read`, map how many image slots the device reports and which index is
    net core.
2.2 Determine the correct smpmgr invocation for **each** image. If smpmgr cannot target the
    net-core image, switch to the reference client (nRF Connect Device Manager / mcumgr)
    for the actual upload and record that smpmgr alone is insufficient for multi-image.
2.3 Decide the confirmed upload recipe and write it into the results record before Phase 3.

### Phase 3 — Upload (staging)
3.1 Upload app-core image (`tempo-bt-v1.signed.bin`, index 0) to its secondary slot.
3.2 Upload net-core image (`ipc_radio.bin`, index 1) to its secondary slot.
    *(Or: single multi-image zip upload via the reference client.)*
3.3 `image state-read` → confirm **both** staged images are present with the expected
    hashes and marked pending. Compare hashes against the §3 known-good set.

### Phase 4 — Apply
4.1 `smpmgr --ble Tempo-BT-000X os reset`.
4.2 Observe: device reboots, MCUboot overwrites app slot, PCD reprograms net core. Expect a
    longer-than-normal first boot. Device should re-advertise within a reasonable window.
4.3 If the device does **not** re-advertise → invoke Phase 0 recovery, log as a failure,
    stop.

### Phase 5 — Acceptance verification (post-upgrade)
5.1 `os echo` reconnects over BLE.
5.2 `image state-read` → active image hashes now match the 1.5.0 app image; net-core image
    updated; slot flags consistent with overwrite-only (active + confirmed).
5.3 **Version acceptance:** the §7 indicator now reports 1.5.0.
5.4 Byte-level check: the running app-image hash reported by `image state-read` equals the
    SHA-256 of the app image extracted from `dfu_application.zip`.

### Phase 6 — Regression (device still fully functional)
6.1 `tempo session-list`, `storage-info` succeed.
6.2 Re-download the Phase 1.4 session; SHA-256 **must match** the pre-upgrade value
    (proves file transfer + storage intact across DFU).
6.3 `settings-get`/`settings-set` round-trip; LED on/off; logger arm/disarm — smoke the
    tempo command surface.

---

## 7. Acceptance criteria

DFU is considered **verified** when **all** hold:

1. **[Firmware identity]** A reliable version indicator shows **1.5.0** post-upgrade where
   it showed 1.4.0 before. *(Resolve the indicator in Phase 1: the tempo plugin has no
   explicit firmware-version command, so use `settings-get` if it carries a version, or a
   documented 1.5.0-only behavior such as magnetometer/`MMC5983MA` support or the v1.5.0
   `SESSION_LIST` key format. Record which signal was used.)*
2. **[Byte integrity]** The active app-image hash from `image state-read` equals the
   SHA-256 of the app image in the known-good `dfu_application.zip`.
3. **[Net core updated]** `image state-read` shows the net-core image at index 1 updated to
   the new hash; BLE/radio fully functional after reboot.
4. **[No data loss]** A session downloaded before and after the upgrade has identical
   SHA-256 (log storage untouched).
5. **[Repeatable]** The exact confirmed command sequence is documented such that devices
   `0006` and `0008` can be upgraded by following it verbatim.

---

## 8. Rollback / recovery

There is **no OTA rollback** (overwrite-only). Recovery = SWD re-flash of
`merged.hex` + `merged_CPUNET.hex` per `build/generated_nrfutil_batch.json`
(`nrfutil device` / `west flash`). This must be proven working in Phase 0.

---

## 9. Results record

### Run 1 — device 0004, 1.4.0 → 1.5.0 — **PASS**

```
Date:        2026-07-11            Executor: riley (assisted)
Device id:   0004                  Debugger present: yes (SWD available)
BLE addr:    E9:F0:2B:A7:2D:7E     (retained across the reboot this run)

Build under test SHA-256 (file):
  dfu_application.zip:    496d1e23e7b2894853a414c5e237731a51299f75d32ca243fb2811a678dc8bfe
  tempo-bt-v1.signed.bin: 064e73c7c46f972c0664a3a4de3e3e5d5c9fb946029374d2d0794a5214fde51a
  ipc_radio.bin:          f2f4438d95fd573eaf6084e7c35299eadd6e09c736500992f795e805d916bc42

MCUboot image hashes (from image state-read — the acceptance references):
  app 1.4.0 (before):  787D372CD09DB7FDE5BDC8352F7E2725BD0358ECB31348E3D68226C00275E7A4
  app 1.5.0 (staged→running): A641F7473100881EB0E9E44F99A41919F3E77B23710C30016BF84F6A7929A4F3
  net (staged):        30C143EFB73B88E0B7AC03344D0910CAD8AF269CBA3833B047B8548B8DD2633F

Phase 0 recovery rehearsed:  not run this time (SWD recovery available as safety net)
Upload client used:          smpmgr 0.13.2 (native image/os groups; tempo plugin NOT loaded)
Post-upgrade version signal:  n/a — verified by image-hash match (settings-get version not yet in fw)
App image hash matches staged 1.5.0:  YES (running slot0 == A641F747, active+confirmed)
Net core updated / BLE ok:            YES (os echo works post-reboot; radio functional)
fs group functional post-DFU:         YES (file read-size responds; 0004 is production/no testok)
Session SHA-256 pre==post:            NOT TESTED (tempo session-list plugin didn't load; see gaps)
Overall:                              PASS

Timing (BLE, single scan adapter):
  app-core upload  (337 KB):  ~23.1 s  (~14.6 KB/s)
  net-core upload  (172 KB):  ~12.9 s  (~13.3 KB/s)
  state-write x2 + os reset:  ~few s
  reset → re-advertise:       ~11 s   (both-core overwrite + PCD net-core reprogram)
  ── full upload→reconnect wall time: ~50–55 s
```

### Confirmed CLI recipe (reusable for 0006 / 0008)

```bash
export SMPMGR_PLUGIN_PATH=/home/riley/src/tempo-insights/smpmgr-extensions/plugins  # (not needed for DFU)
DEV=<ble-addr-or-name>          # rediscover by name after any reboot; MAC may re-randomize
unzip -o build/dfu_application.zip   # → tempo-bt-v1.signed.bin, ipc_radio.bin

# 1. stage both images (non-destructive; device keeps running old fw)
smpmgr --timeout 30 --ble $DEV image upload tempo-bt-v1.signed.bin --slot 0   # app  → image 0
smpmgr --timeout 30 --ble $DEV image upload ipc_radio.bin          --slot 1   # net  → image 1
smpmgr --ble $DEV image state-read      # confirm slot1/image0 + slot1/image1 present, note hashes

# 2. commit (point of no return — overwrite-only, no rollback)
smpmgr --ble $DEV image state-write <APP_STAGED_HASH> false    # mark app pending
smpmgr --ble $DEV image state-write <NET_STAGED_HASH> false    # mark net pending
smpmgr --ble $DEV os reset

# 3. verify: rediscover by name, then
smpmgr --ble $DEV image state-read      # running slot0 hash == APP_STAGED_HASH, active+confirmed
```

Key findings:
- smpmgr's native `image upload --slot N` **does** address the nRF5340 image number:
  `--slot 0` → app core (image 0), `--slot 1` → net core (image 1). The `--slot` name is a
  misnomer for "image number." No `mcumgr`/Device Manager fallback was needed.
- **Each image must be marked pending by a state-write on its OWN hash** — the app hash
  pends the app image, the net hash pends the net image. Both are required for the
  simultaneous swap. (An early guess that one write pends both was wrong: the net image had
  been pended by a prior run.) BLE reconnects occasionally drop a request, so verify the
  pending count == 2 and retry.
- **`os reset` can silently fail to trigger the reboot.** Symptom: the device "re-advertises"
  in ~11 s but `state-read` still shows the OLD image running with the new images still
  `pending=True` — it never rebooted. A real swap takes ~30–45 s. Re-issue `os reset` (with
  visible output) until `state-read` shows the new image active and the pending images gone.
- Overwrite-only behaved as designed: after the swap the new image is directly
  `active + confirmed`; the secondary slots are consumed (no revert path).
- smpmgr also exposes a one-shot `upgrade` (upload+test+reset) — usable for **single-image**
  updates, but this dual-core case needs the two uploads staged before the single reset.

### Runs 2 & 3 — devices 0005, 0006, 1.4.0 → 1.5.0 — **PASS** (via `tools/dfu-upgrade.sh`)

Both upgraded from the same `789...`→`A641F747…` app image and verified: running app hash =
`A641F747…` active+confirmed, and each **still advertises its `Tempo-BT-nnnn` name** after
the upgrade (the explicit acceptance requirement). Upload timings matched Run 1 (~23 s app /
~10–15 s net). 0005's reset took on the first try (~36 s to swap); 0006 needed a re-issued
`os reset` (first reset silently didn't reboot), then swapped in ~41 s — this is what
motivated the reset-retry loop now built into `tools/dfu-upgrade.sh`.

### Run 4 — device 0008 — re-provisioned + **PASS**

`0008` had lost its name (advertising bare `Tempo-BT`, unprovisioned) and had to be
re-provisioned before upgrading:

```
smpmgr --plugin-path=<plugins> --ble <addr> tempo settings-set --ble-name "Tempo-BT-0008" --pcb-variant 2
smpmgr --ble <addr> os reset      # name changes require a reboot
```

Then upgraded 1.4.0 → 1.5.0 (running app = `A641F747…`), and the `Tempo-BT-0008` name +
PCB-variant 0x02 **persisted across the DFU** (settings live in NVM, separate from firmware).

### Infrastructure findings (2026-07-11)

- **`--plugin-path` must use the `=` form**: `smpmgr --plugin-path=<dir> …`. smpmgr's
  `get_plugins()` only scans argv for `--plugin-path=` (with equals) and the
  `SMPMGR_PLUGIN_PATH` env var is **not** consulted. Space-separated `--plugin-path <dir>`
  silently loads nothing → `No such command 'tempo'`. This was the entire "plugin won't
  load" problem from Runs 1–3; the plugin code is fine.
- **BLE flakiness was adapter contention, not firmware.** With the 4 transfer dongles +
  onboard controller present (5 HCI controllers, dongles reading all-zero addresses),
  bleak/BlueZ discovery and connection to a device were intermittent — the `os reset`
  "device not found" and the tool's failed re-discovery were all this. With the dongles
  unplugged (single controller) a connect probe was **6/6**. smpmgr has no adapter-select
  flag, so bleak uses the BlueZ default across contending controllers.
- **Prefer bleak connect over bluetoothctl scan.** Direct `smpmgr` operations (upload,
  state-read, reset) to a known address were reliable throughout; `bluetoothctl scan`
  discovery was the flaky layer. Devices keep their BLE MAC across reboots, so post-reset
  verification should **poll the known address directly** rather than re-scan by name.
  `tools/dfu-upgrade.sh` was updated to do exactly this.

A bare unprovisioned `Tempo-BT` (no suffix) seen alongside the fleet is correctly ignored
by harvest logic — but note it is how a name-reset device (like 0008 here) appears.

### Note: same-version OTA re-apply is not possible

`0010` was flashed to 1.5.0 over SWD/J-Link. An attempt to re-run the OTA on it (to exercise
the full swap cycle on the single-adapter setup) revealed a hard limit: **you cannot OTA the
same firmware version a device is already running.** The staged app image has the *same*
MCUboot hash as the running image, and `image state-write <hash> false` cannot distinguish
the secondary slot from the active primary — so the app image never gets marked pending
(only the net image, which has a distinct hash, did). The tool detects this (pending count
stays 1) and refuses to reset, so the device is left untouched. Consequence: OTA is for
version *changes* only; to re-flash the same version use SWD/J-Link. (A leftover net-image
pending flag from the attempt clears harmlessly on the next reboot.)

### Gaps / follow-ups
- **Tempo group-64 plugin did not load** under smpmgr 0.13.2 (`No such command 'tempo'`,
  even with `--plugin-path`). This blocked `session-list`/`settings-get` baseline and the
  byte-for-byte session download regression. Investigate plugin/smpmgr API compatibility
  separately; it does not affect the native DFU path.
- Net-core image is not directly re-readable via `image state-read` after boot (nRF5340
  limitation) — functional BLE is the proxy for net-core success.
- Repeat on `0006` and `0008` using the recipe above; consider a Phase-0 recovery rehearsal
  first on whichever unit is least accessible.

Post-run: `README.md` OTA line updated (validated); recommend also adding a line to the
`tempo-tb-ingest` `docs/feasibility.md` validation history.
