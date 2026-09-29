#!/usr/bin/env python3
'''
Detect ArduPilot flight controllers on USB, identify the board ID and list
the firmware available for it on firmware.ardupilot.org.

Optionally monitor the board for I2C interrupt storms (internal error
i2c_isr, bit 19) while you try to reproduce them.

usage:
  usb_fw_probe.py                       # scan, identify, list firmware
  usb_fw_probe.py --vehicle Copter      # filter the firmware list
  usb_fw_probe.py --port /dev/ttyACM0 --monitor-i2c

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

I2C_ISR_ERROR_BIT = 1 << 19


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


def monitor_i2c(device, baud, duration):
    '''watch SYS_STATUS/HEARTBEAT/STATUSTEXT for the i2c_isr internal error'''
    from pymavlink import mavutil
    conn = mavutil.mavlink_connection(device, baud=baud, autoreconnect=True)
    conn.wait_heartbeat(timeout=10)
    # ask for SYS_STATUS at 2Hz
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000, 0, 0, 0, 0, 0)
    print("Monitoring for I2C ISR storm (internal error bit 19) - Ctrl-C to stop")
    print("Now disturb the I2C bus (see notes printed at the end).")
    last_errors = None
    last_count = None
    end = time.time() + duration if duration else None
    try:
        while end is None or time.time() < end:
            m = conn.recv_match(type=['SYS_STATUS', 'HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=2)
            if m is None:
                continue
            t = m.get_type()
            if t == 'STATUSTEXT':
                print("%s STATUSTEXT: %s" % (time.strftime("%H:%M:%S"), m.text))
            elif t == 'HEARTBEAT' and m.system_status == mavutil.mavlink.MAV_STATE_CRITICAL:
                print("%s HEARTBEAT system_status=CRITICAL (internal error set)" % time.strftime("%H:%M:%S"))
            elif t == 'SYS_STATUS':
                errors = m.errors_count1 | (m.errors_count2 << 16)
                count = m.errors_count4
                if errors != last_errors or count != last_count:
                    flag = " <== I2C ISR STORM DETECTED" if errors & I2C_ISR_ERROR_BIT else ""
                    print("%s internal_errors=0x%08x count=%u%s" %
                          (time.strftime("%H:%M:%S"), errors, count, flag))
                    last_errors, last_count = errors, count
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial device (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--vehicle", help="Copter, Plane, Rover, Sub, Heli, AntennaTracker, Blimp, AP_Periph")
    parser.add_argument("--release", help="OFFICIAL, BETA, DEV, STABLE ...")
    parser.add_argument("--board-id", type=int, help="skip detection and use this board id")
    parser.add_argument("--monitor-i2c", action="store_true", help="monitor for I2C ISR storms after probing")
    parser.add_argument("--duration", type=float, default=0, help="monitor duration in seconds (0=forever)")
    parser.add_argument("--cache", default=os.path.expanduser("~/.cache/ap_manifest.json.gz"))
    args = parser.parse_args()

    names = load_board_names()
    board_id = args.board_id
    device = args.port

    if board_id is None:
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

        device = ports[0].device
        print("Probing %s via MAVLink ..." % device)
        res = board_id_from_mavlink(device, args.baud, args.timeout)
        if res is not None:
            board_id, info = res
            print("   running firmware %s (git %s)" % (info["fw_version"], info["git"]))
        else:
            print("   no MAVLink; trying bootloader protocol ...")
            board_id = board_id_from_bootloader(device, 1.0)
        if board_id is None:
            print("Could not read board id. Replug the board and run again within a few "
                  "seconds (bootloader window), or pass --board-id.")
            sys.exit(1)

    print("Board ID: %u (%s)" % (board_id, ", ".join(names.get(board_id, ["unknown in board_types.txt"]))))

    manifest = fetch_manifest(args.cache)
    rows = list_firmware(manifest, board_id, args.vehicle, args.release)
    if not rows:
        print("No firmware found in manifest for board id %u" % board_id)
    else:
        print("\n%-10s %-9s %-10s %-22s %-5s %s" % ("vehicle", "release", "version", "platform", "fmt", "url"))
        for fw in rows:
            print("%-10s %-9s %-10s %-22s %-5s %s" % (
                fw.get("vehicletype", ""), fw.get("mav-firmware-version-type", ""),
                fw.get("mav-firmware-version", ""), fw.get("platform", ""),
                fw.get("format", ""), fw.get("url", "")))
        print("\nFlash a .apj with:  Tools/scripts/uploader.py --port %s <file.apj>" % (device or "<port>"))

    if args.monitor_i2c:
        if device is None:
            print("--monitor-i2c needs --port")
            sys.exit(1)
        monitor_i2c(device, args.baud, args.duration)


if __name__ == "__main__":
    main()
