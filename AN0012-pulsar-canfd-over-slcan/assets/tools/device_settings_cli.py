"""
CLI tool for viewing and adjusting Binho device settings.

Supports: Binho Supernova and Binho Pulsar.
Requires: pip install hidapi
"""

import argparse
import struct
import sys
import time
import hid

# ── Device constants ────────────────────────────────────────────────
USB_VID = 0x1FC9
DEVICES = {
    0x82FC: "Binho Supernova",
    0x82FD: "Binho Pulsar",
}
HID_EP_BUF = 64

# ── Command codes (must match system_definitions.h) ─────────────────
GROUP_SYS = 1
ROLE_GENERIC = 0
TYPE_REQ_RESP = 0

def make_cmd(group, role, typ, val):
    return (group << 10) | (role << 8) | (typ << 7) | val

SYS_RESET_DEVICE   = make_cmd(GROUP_SYS, ROLE_GENERIC, TYPE_REQ_RESP, 1)
SYS_SET_USB_CONFIG  = make_cmd(GROUP_SYS, ROLE_GENERIC, TYPE_REQ_RESP, 13)
SYS_GET_USB_CONFIG  = make_cmd(GROUP_SYS, ROLE_GENERIC, TYPE_REQ_RESP, 14)

SUCCESS = 0x0000

CDC_MODE_NAMES = {
    0: "uart_passthrough",
    1: "terminal",
    2: "nova_compat",
    3: "rs485",
    4: "canfd",
}
CDC_MODE_BY_NAME = {v: k for k, v in CDC_MODE_NAMES.items()}

# ── USB transfer helpers ────────────────────────────────────────────

def build_packet(payload: bytes) -> bytes:
    length = len(payload)
    header = (1 << 0) | (1 << 1) | (length << 2)
    pkt = struct.pack('<H', header) + payload
    return b'\x00' + pkt.ljust(HID_EP_BUF, b'\x00')

def parse_response(data: bytes):
    header_raw = struct.unpack('<H', data[0:2])[0]
    payload_len = (header_raw >> 2) & 0x3FF
    payload = data[2:2+payload_len]
    resp_id, resp_code, result = struct.unpack('<HHH', payload[0:6])
    params = payload[6:]
    return resp_id, resp_code, result, params

# ── HID helpers ─────────────────────────────────────────────────────

def open_device(device_filter=None):
    """Open a Binho device. device_filter can be 'supernova' or 'pulsar'."""
    if device_filter:
        key = device_filter.lower()
        filtered = {pid: name for pid, name in DEVICES.items() if key in name.lower()}
        if not filtered:
            print(f"Error: Unknown device filter '{device_filter}'.")
            print(f"  Valid: supernova, pulsar")
            sys.exit(1)
    else:
        filtered = DEVICES

    dev = hid.device()
    for pid, name in filtered.items():
        try:
            dev.open(USB_VID, pid)
            print(f"Connected to {name}")
            return dev
        except Exception:
            continue

    if device_filter:
        print(f"Error: Could not find Binho {device_filter.title()}.")
    else:
        print("Error: No Binho device found.")
        print(f"  Looked for: {', '.join(DEVICES.values())}")
    print("  Make sure the device is connected and in app mode.")
    print("  Tip: Use --device supernova/pulsar if both are connected.")
    sys.exit(1)

def send_cmd(dev, cmd_code, params=b'', req_id=1):
    payload = struct.pack('<HH', req_id, cmd_code) + params
    dev.write(build_packet(payload))
    raw = dev.read(HID_EP_BUF, 3000)
    if not raw:
        raise TimeoutError("No response from device")
    return parse_response(bytes(raw))

def get_settings(dev):
    _, _, result, params = send_cmd(dev, SYS_GET_USB_CONFIG)
    if result != SUCCESS:
        print(f"Error: GET_USB_CONFIG failed (0x{result:04X})")
        sys.exit(1)
    webusb, cdc, mode = struct.unpack('BBB', params[:3])
    return webusb, cdc, mode

def set_settings(dev, webusb=0xFF, cdc=0xFF, cdc_mode=0xFF):
    params = struct.pack('BBB', webusb, cdc, cdc_mode)
    _, _, result, resp_params = send_cmd(dev, SYS_SET_USB_CONFIG, params)
    if result != SUCCESS and len(resp_params) >= 4:
        diag = struct.unpack('<I', resp_params[:4])[0]
        step = (diag >> 24) & 0xFF
        raw_status = diag & 0x00FFFFFF
        step_names = {1: "FLASH_Init", 2: "FLASH_Read", 3: "FLASH_Erase", 4: "FLASH_Program"}
        print(f"  Flash diagnostic: step={step} ({step_names.get(step, '?')}), "
              f"status=0x{raw_status:06X} (raw=0x{diag:08X})")
    return result

def reset_device(dev):
    payload = struct.pack('<HH', 99, SYS_RESET_DEVICE)
    dev.write(build_packet(payload))

# ── Commands ────────────────────────────────────────────────────────

def cmd_show(args):
    dev = open_device(args.device)
    webusb, cdc, mode = get_settings(dev)
    dev.close()

    mode_name = CDC_MODE_NAMES.get(mode, f"unknown({mode})")
    print(f"  webusb_enabled : {webusb}  {'(on)' if webusb else '(off)'}")
    print(f"  cdc_enabled    : {cdc}  {'(on)' if cdc else '(off)'}")
    print(f"  cdc_mode       : {mode}  ({mode_name})")

def cmd_set(args):
    dev = open_device(args.device)
    webusb_val = 0xFF
    cdc_val = 0xFF
    mode_val = 0xFF

    if args.webusb is not None:
        webusb_val = 1 if args.webusb in ("on", "1", "true") else 0
    if args.cdc is not None:
        cdc_val = 1 if args.cdc in ("on", "1", "true") else 0
    if args.cdc_mode is not None:
        if args.cdc_mode in CDC_MODE_BY_NAME:
            mode_val = CDC_MODE_BY_NAME[args.cdc_mode]
        else:
            try:
                mode_val = int(args.cdc_mode)
            except ValueError:
                print(f"Error: Invalid cdc_mode '{args.cdc_mode}'.")
                print(f"  Valid: {', '.join(CDC_MODE_BY_NAME.keys())} or 0/1/2/3")
                dev.close()
                sys.exit(1)

    if webusb_val == 0xFF and cdc_val == 0xFF and mode_val == 0xFF:
        print("Nothing to change. Use --webusb, --cdc, or --cdc-mode.")
        dev.close()
        return

    result = set_settings(dev, webusb_val, cdc_val, mode_val)
    if result != SUCCESS:
        print(f"Error: SET_USB_CONFIG failed (0x{result:04X})")
        dev.close()
        sys.exit(1)

    # Show updated settings
    webusb, cdc, mode = get_settings(dev)
    dev.close()

    mode_name = CDC_MODE_NAMES.get(mode, f"unknown({mode})")
    print("Settings updated:")
    print(f"  webusb_enabled : {webusb}  {'(on)' if webusb else '(off)'}")
    print(f"  cdc_enabled    : {cdc}  {'(on)' if cdc else '(off)'}")
    print(f"  cdc_mode       : {mode}  ({mode_name})")

    if not args.no_save:
        print("\nResetting device to persist settings to flash...")
        dev = open_device(args.device)
        reset_device(dev)
        dev.close()
        print("Device is resetting. Settings will be saved to flash.")

def cmd_reset(args):
    dev = open_device(args.device)

    # Restore defaults
    result = set_settings(dev, webusb=1, cdc=1, cdc_mode=0)
    if result != SUCCESS:
        print(f"Error: SET_USB_CONFIG failed (0x{result:04X})")
        dev.close()
        sys.exit(1)

    print("Settings restored to defaults:")
    print("  webusb_enabled : 1  (on)")
    print("  cdc_enabled    : 1  (on)")
    print("  cdc_mode       : 0  (uart_passthrough)")

    print("\nResetting device to persist settings to flash...")
    reset_device(dev)
    dev.close()
    print("Device is resetting. Defaults will be saved to flash.")

# ── Main ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="View and adjust Binho device settings.",
    )
    parser.add_argument("--device", choices=["supernova", "pulsar"],
                        help="Target a specific device (auto-detect if omitted)")
    sub = parser.add_subparsers(dest="command")

    # show
    sub.add_parser("show", help="Display current settings")

    # set
    p_set = sub.add_parser("set", help="Change one or more settings")
    p_set.add_argument("--webusb", choices=["on", "off", "0", "1", "true", "false"],
                       help="Enable/disable WebUSB")
    p_set.add_argument("--cdc", choices=["on", "off", "0", "1", "true", "false"],
                       help="Enable/disable CDC")
    p_set.add_argument("--cdc-mode", dest="cdc_mode",
                       help="CDC mode: uart_passthrough, terminal, nova_compat, rs485, canfd (or 0/1/2/3/4)")
    p_set.add_argument("--no-save", action="store_true",
                       help="Don't reset device after setting (changes stay in RAM only)")

    # reset
    sub.add_parser("reset", help="Restore all settings to factory defaults and save")

    args = parser.parse_args()

    if args.command == "show":
        cmd_show(args)
    elif args.command == "set":
        cmd_set(args)
    elif args.command == "reset":
        cmd_reset(args)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
