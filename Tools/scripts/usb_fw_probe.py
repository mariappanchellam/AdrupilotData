#!/usr/bin/env python3
'''
Detect ArduPilot flight controllers on USB, identify the board ID and list
the firmware available for it on firmware.ardupilot.org.

By default lists the targets that can be built from this source tree
(hwdef boards whose APJ_BOARD_ID or name matches the connected board).
With --online it also lists prebuilt firmware from firmware.ardupilot.org.

usage:
  usb_fw_probe.py                       # scan, identify, list buildable targets
  usb_fw_probe.py --online              # also list prebuilt firmware
  usb_fw_probe.py --port /dev/ttyACM0 --vehicle Copter

requires: pyserial, pymavlink

AP_FLAKE8_CLEAN
'''

import argparse
import gzip
import json
import os
import struct
import sys
import time
import urllib.request

import serial
from serial.tools import list_ports

MANIFEST_URL = "https://firmware.ardupilot.org/manifest.json.gz"

# USB vendor IDs commonly used by ArduPilot-capable flight controllers
KNOWN_VIDS = {
    0x1209: "ArduPilot (pid.codes)",
    0x2DAE: "CubePilot/Hex",
    0x26AC: "3DR/PX4",
    0x3162: "Holybro",
    0x0483: "STMicroelectronics",
    0x35A7: "ARK Electronics",
    0x3185: "Various (CUAV etc.)",
    0x1FC9: "NXP",
}

# bootloader protocol (see Tools/scripts/uploader.py)
BL_INSYNC = 0x12
BL_OK = 0x10
BL_EOC = 0x20
BL_GET_SYNC = 0x21
BL_GET_DEVICE = 0x22
BL_INFO_BL_REV = 0x01
BL_INFO_BOARD_ID = 0x02


def load_board_names():
    '''map board_id -> name from Tools/AP_Bootloader/board_types.txt'''
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "AP_Bootloader", "board_types.txt")
    names = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.split('#')[0].split()
                if len(line) == 2 and line[1].isdigit():
                    names.setdefault(int(line[1]), []).append(line[0])
    except OSError:
        pass
    return names


def hwdef_board_id(path, name_to_id, seen=None):
    '''follow includes in a hwdef file; return (board_id or None, is_periph)'''
    seen = seen if seen is not None else set()
    path = os.path.normpath(path)
    if path in seen or not os.path.exists(path):
        return (None, False)
    seen.add(path)
    board_id = None
    periph = False
    with open(path) as f:
        for line in f:
            words = line.split('#')[0].split()
            if not words:
                continue
            if words[0] == 'include' and len(words) > 1:
                inc_id, inc_periph = hwdef_board_id(os.path.join(os.path.dirname(path), words[1]), name_to_id, seen)
                if inc_id is not None:
                    board_id = inc_id
                periph = periph or inc_periph
            elif words[0] == 'undef' and 'APJ_BOARD_ID' in words[1:]:
                board_id = None
            elif words[0] == 'APJ_BOARD_ID' and len(words) > 1:
                v = words[1]
                board_id = int(v, 0) if v[0].isdigit() else name_to_id.get(v)
            elif any(w.startswith('AP_PERIPH') for w in words[:2]) or 'AP_PERIPH' in words:
                periph = True
    return (board_id, periph)


def local_targets(board_names):
    '''scan libraries/AP_HAL_ChibiOS/hwdef; return list of (target, board_id, is_periph)'''
    name_to_id = {}
    for bid, names in board_names.items():
        for n in names:
            name_to_id[n] = bid
    here = os.path.dirname(os.path.abspath(__file__))
    hwdef_dir = os.path.normpath(os.path.join(here, "..", "..", "libraries", "AP_HAL_ChibiOS", "hwdef"))
    targets = []
    for d in sorted(os.listdir(hwdef_dir)):
        hwdef = os.path.join(hwdef_dir, d, "hwdef.dat")
        if not os.path.exists(hwdef):
            continue
        bid, periph = hwdef_board_id(hwdef, name_to_id)
        targets.append((d, bid, periph))
    return targets


def find_ports():
    ports = []
    for p in list_ports.comports():
        if p.vid is None:
            continue
        if p.vid in KNOWN_VIDS or "ardupilot" in (p.manufacturer or "").lower():
            ports.append(p)
    return ports


def board_id_from_mavlink(device, baud, timeout):
    '''ask a running autopilot for AUTOPILOT_VERSION; returns (board_id, info) or None'''
    from pymavlink import mavutil
    conn = mavutil.mavlink_connection(device, baud=baud, autoreconnect=False)
    try:
        hb = conn.wait_heartbeat(timeout=timeout)
        if hb is None:
            return None
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                   mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                                   0, 0, 0, 0, 0, 0)
        deadline = time.time() + timeout
        while time.time() < deadline:
            m = conn.recv_match(type=['AUTOPILOT_VERSION', 'STATUSTEXT'], blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == 'STATUSTEXT':
                print("   STATUSTEXT: %s" % m.text)
                continue
            sw = m.flight_sw_version
            info = {
                "fw_version": "%u.%u.%u" % ((sw >> 24) & 0xff, (sw >> 16) & 0xff, (sw >> 8) & 0xff),
                "git": bytes(m.flight_custom_version).rstrip(b'\0').decode(errors='replace'),
                "autopilot_type": hb.autopilot,
                "mav_type": hb.type,
            }
            return (m.board_version >> 16, info)
    finally:
        conn.close()
    return None


def board_id_from_bootloader(device, timeout):
    '''query the ArduPilot/PX4 bootloader directly; returns board_id or None'''
    try:
        s = serial.Serial(device, 115200, timeout=timeout)
    except serial.SerialException:
        return None
    try:
        s.reset_input_buffer()
        s.write(bytes([BL_GET_SYNC, BL_EOC]))
        r = s.read(2)
        if len(r) != 2 or r[0] != BL_INSYNC or r[1] != BL_OK:
            return None
        s.write(bytes([BL_GET_DEVICE, BL_INFO_BOARD_ID, BL_EOC]))
        r = s.read(6)
        if len(r) != 6 or r[4] != BL_INSYNC or r[5] != BL_OK:
            return None
        return struct.unpack('<I', r[:4])[0]
    finally:
        s.close()


def fetch_manifest(cache_path):
    if cache_path and os.path.exists(cache_path) and time.time() - os.path.getmtime(cache_path) < 3600:
        with open(cache_path, 'rb') as f:
            data = f.read()
    else:
        print("Downloading %s ..." % MANIFEST_URL)
        with urllib.request.urlopen(MANIFEST_URL, timeout=60) as r:
            data = r.read()
        if cache_path:
            with open(cache_path, 'wb') as f:
                f.write(data)
    return json.loads(gzip.decompress(data))


def list_firmware(manifest, board_id, vehicle=None, release=None):
    rows = []
    for fw in manifest.get("firmware", []):
        if fw.get("board_id") != board_id:
            continue
        if vehicle and fw.get("vehicletype", "").lower() != vehicle.lower():
            continue
        if release and fw.get("mav-firmware-version-type", "").lower() != release.lower():
            continue
        if fw.get("format") not in ("apj", "abin"):
            continue
        rows.append(fw)
    rows.sort(key=lambda f: (f.get("vehicletype", ""), f.get("mav-firmware-version-type", ""),
                             f.get("platform", ""), f.get("format", "")))
    return rows


VEHICLE_TARGETS = ["copter", "heli", "plane", "rover", "sub", "antennatracker", "blimp"]


def print_buildable(targets, board_id, product_name, vehicle):
    '''print waf commands for local hwdef targets matching the board'''
    by_name = {t[0]: t for t in targets}
    matches = [t for t in targets if board_id is not None and t[1] == board_id]
    if product_name in by_name and by_name[product_name] not in matches:
        matches.insert(0, by_name[product_name])
    if not matches:
        print("\nNo hwdef in libraries/AP_HAL_ChibiOS/hwdef matches this board.")
        return
    # the USB product string is the board name of the running firmware, so it is the best match
    matches.sort(key=lambda t: (t[0] != product_name, t[0].lower()))
    print("\nBuildable targets in this source tree (%u):" % len(matches))
    for name, bid, periph in matches:
        tag = "  <== matches USB board name" if name == product_name else ""
        print("  %-28s board_id=%s%s%s" % (name, bid, " (AP_Periph)" if periph else "", tag))
    name, _, periph = matches[0]
    if periph:
        vehicles = ["AP_Periph"]
    elif vehicle:
        vehicles = [vehicle.lower()]
    else:
        vehicles = VEHICLE_TARGETS
    print("\nTo build for %s:" % name)
    print("  ./waf configure --board %s" % name)
    for v in vehicles:
        print("  ./waf %s" % v)
    print("Output: build/%s/bin/*.apj  (flash with Tools/scripts/uploader.py)" % name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial device (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--vehicle", help="copter, heli, plane, rover, sub, antennatracker, blimp")
    parser.add_argument("--release", help="with --online: OFFICIAL, BETA, DEV ...")
    parser.add_argument("--board-id", type=int, help="skip USB detection and use this board id")
    parser.add_argument("--board", help="skip USB detection and use this hwdef board name")
    parser.add_argument("--online", action="store_true", help="also list prebuilt firmware from firmware.ardupilot.org")
    parser.add_argument("--cache", default=os.path.expanduser("~/.cache/ap_manifest.json.gz"))
    args = parser.parse_args()

    names = load_board_names()
    targets = local_targets(names)
    board_id = args.board_id
    product_name = args.board
    device = args.port

    if board_id is None and product_name is None:
        ports = [p for p in list_ports.comports() if p.device == device] if device else find_ports()
        if not ports and device:
            ports = [type("P", (), {"device": device, "vid": None, "pid": None,
                                    "manufacturer": "", "product": "", "serial_number": ""})]
        if not ports:
            print("No ArduPilot-like USB serial devices found. Is the board plugged in?")
            print("All serial ports:")
            for p in list_ports.comports():
                print("  %s %s %s" % (p.device, p.hwid, p.description))
            sys.exit(1)

        for p in ports:
            print("Found %s  VID:PID=%s  %s / %s  serial=%s" % (
                p.device,
                "%04x:%04x" % (p.vid, p.pid) if p.vid is not None else "?",
                p.manufacturer, p.product, p.serial_number))
            if p.vid in KNOWN_VIDS:
                print("   vendor: %s" % KNOWN_VIDS[p.vid])
            if p.product and p.product.endswith("-BL"):
                print("   board is in bootloader mode")
        if len(ports) > 1:
            print("Several ports found; using %s (pick another with --port)" % ports[0].device)

        device = ports[0].device
        product = ports[0].product or ""
        product_name = product[:-3] if product.endswith("-BL") else product
        print("Probing %s via MAVLink ..." % device)
        res = board_id_from_mavlink(device, args.baud, args.timeout)
        if res is not None:
            board_id, info = res
            print("   running firmware %s (git %s)" % (info["fw_version"], info["git"]))
        else:
            print("   no MAVLink; trying bootloader protocol ...")
            board_id = board_id_from_bootloader(device, 1.0)
        if board_id is None and product_name not in [t[0] for t in targets]:
            print("Could not read board id. Replug the board and run again within a few "
                  "seconds (bootloader window), or pass --board-id / --board.")
            sys.exit(1)

    if board_id is None and product_name:
        board_id = next((t[1] for t in targets if t[0] == product_name), None)
    if board_id is not None:
        print("Board ID: %u (%s)" % (board_id, ", ".join(names.get(board_id, ["unknown in board_types.txt"]))))

    print_buildable(targets, board_id, product_name, args.vehicle)

    if args.online and board_id is not None:
        manifest = fetch_manifest(args.cache)
        rows = list_firmware(manifest, board_id, args.vehicle, args.release)
        if not rows:
            print("\nNo prebuilt firmware found in manifest for board id %u" % board_id)
        else:
            print("\n%-10s %-9s %-10s %-22s %-5s %s" % ("vehicle", "release", "version", "platform", "fmt", "url"))
            for fw in rows:
                print("%-10s %-9s %-10s %-22s %-5s %s" % (
                    fw.get("vehicletype", ""), fw.get("mav-firmware-version-type", ""),
                    fw.get("mav-firmware-version", ""), fw.get("platform", ""),
                    fw.get("format", ""), fw.get("url", "")))


if __name__ == "__main__":
    main()
