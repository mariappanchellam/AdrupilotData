#!/usr/bin/env python3
'''
Repeatedly reboot an ArduPilot board over MAVLink and record whether the
i2c_isr internal error (0x80000) is raised after each boot.

Bench use only: props off. The reboot command is refused while armed.

usage:
  reboot_i2c_check.py --port COM11 --cycles 50
  reboot_i2c_check.py --port /dev/ttyACM0 --cycles 0 --stop-on-error

requires: pymavlink

AP_FLAKE8_CLEAN
'''

import argparse
import csv
import time

from pymavlink import mavutil

I2C_ISR_ERROR_BIT = 1 << 19


def connect(port, baud, timeout):
    '''open the port and wait for a heartbeat; retries while the USB port re-enumerates'''
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            conn = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
        except Exception:
            time.sleep(1)
            continue
        hb = conn.wait_heartbeat(timeout=5)
        if hb is not None:
            return conn
        conn.close()
        time.sleep(1)
    return None


def sample_errors(conn, duration):
    '''watch SYS_STATUS and STATUSTEXT; returns (errors, error_count, messages)'''
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                               mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000, 0, 0, 0, 0, 0)
    errors = 0
    count = 0
    texts = []
    end = time.time() + duration
    while time.time() < end:
        m = conn.recv_match(type=['SYS_STATUS', 'STATUSTEXT'], blocking=True, timeout=1)
        if m is None:
            continue
        if m.get_type() == 'SYS_STATUS':
            errors |= m.errors_count1 | (m.errors_count2 << 16)
            count = max(count, m.errors_count4)
        else:
            text = m.text
            if "error" in text.lower() or "i2c" in text.lower():
                texts.append(text)
    return errors, count, texts


def reboot(conn):
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                               1, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    conn.close()
    if ack is not None and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="MAVLink port, e.g. COM11 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--cycles", type=int, default=20, help="number of reboots (0=until Ctrl-C)")
    parser.add_argument("--settle", type=float, default=20, help="seconds to watch after each boot")
    parser.add_argument("--boot-timeout", type=float, default=60, help="seconds to wait for the board to return")
    parser.add_argument("--stop-on-error", action="store_true", help="stop when i2c_isr is seen")
    parser.add_argument("--csv", default="reboot_i2c_check.csv")
    args = parser.parse_args()

    hits = 0
    cycle = 0
    with open(args.csv, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["cycle", "time", "connected", "internal_errors", "error_count", "i2c_isr", "messages"])
        try:
            while args.cycles == 0 or cycle < args.cycles:
                cycle += 1
                conn = connect(args.port, args.baud, args.boot_timeout)
                stamp = time.strftime("%Y-%m-%d %H:%M:%S")
                if conn is None:
                    print("[%u] board did not come back on %s within %.0fs" % (cycle, args.port, args.boot_timeout))
                    out.writerow([cycle, stamp, 0, "", "", "", "no heartbeat"])
                    f.flush()
                    break
                errors, count, texts = sample_errors(conn, args.settle)
                i2c = bool(errors & I2C_ISR_ERROR_BIT)
                hits += i2c
                print("[%u] internal_errors=0x%08x count=%u%s" % (
                    cycle, errors, count, "  <== I2C ISR STORM" if i2c else ""))
                for t in texts:
                    print("     %s" % t)
                out.writerow([cycle, stamp, 1, "0x%08x" % errors, count, int(i2c), " | ".join(texts)])
                f.flush()
                if i2c and args.stop_on_error:
                    conn.close()
                    print("Stopping with the error latched; connect Mission Planner to inspect it.")
                    break
                if args.cycles and cycle >= args.cycles:
                    conn.close()
                    break
                if not reboot(conn):
                    print("[%u] reboot refused (armed?) - stopping" % cycle)
                    break
                # let the USB port drop before reconnecting
                time.sleep(3)
        except KeyboardInterrupt:
            pass
    print("\n%u boot(s) checked, i2c_isr seen on %u. Results in %s" % (cycle, hits, args.csv))


if __name__ == "__main__":
    main()
