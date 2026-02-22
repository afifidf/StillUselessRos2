#!/usr/bin/env python3
"""
Tool untuk membaca, menampilkan, dan copy action page dari file motion_4095.bin
Format berdasarkan action_file_define.h ROBOTIS OP3

Usage:
  # Lihat info semua page yang terisi
  python3 action_bin_tool.py list motion_4095.bin

  # Lihat detail page tertentu (misal page 83 = right kick)
  python3 action_bin_tool.py info motion_4095.bin 83

  # Copy page dari robot3.bin ke robot2.bin
  python3 action_bin_tool.py copy robot3.bin robot2.bin 83

  # Copy multiple page sekaligus
  python3 action_bin_tool.py copy robot3.bin robot2.bin 83 84 120 121
"""

import struct
import sys
import os

MAXNUM_PAGE   = 256
MAXNUM_STEP   = 7
MAXNUM_JOINTS = 31
PAGE_SIZE     = 512   # bytes per page
HEADER_SIZE   = 64    # bytes
STEP_SIZE     = 64    # bytes per step


def read_page(f, page_num):
    """Baca satu page dari file."""
    f.seek(page_num * PAGE_SIZE)
    return f.read(PAGE_SIZE)


def write_page(f, page_num, data):
    """Tulis satu page ke file."""
    f.seek(page_num * PAGE_SIZE)
    f.write(data)


def parse_header(page_data):
    """Parse header dari raw page bytes."""
    name_raw = page_data[0:14]
    name = name_raw.split(b'\x00')[0].decode('ascii', errors='replace').strip()
    repeat   = page_data[15]
    schedule = page_data[16]
    stepnum  = page_data[20]
    speed    = page_data[22]
    accel    = page_data[24]
    next_p   = page_data[25]
    exit_p   = page_data[26]
    checksum = page_data[31]
    return {
        'name': name,
        'repeat': repeat,
        'schedule': schedule,
        'stepnum': stepnum,
        'speed': speed,
        'accel': accel,
        'next': next_p,
        'exit': exit_p,
        'checksum': checksum,
    }


def parse_steps(page_data):
    """Parse semua step dari raw page bytes."""
    steps = []
    for i in range(MAXNUM_STEP):
        offset = HEADER_SIZE + i * STEP_SIZE
        step_raw = page_data[offset:offset + STEP_SIZE]
        positions = struct.unpack_from('<' + 'H' * MAXNUM_JOINTS, step_raw, 0)
        pause = step_raw[62]
        time  = step_raw[63]
        steps.append({
            'positions': positions,
            'pause': pause,
            'time': time,
        })
    return steps


def is_page_empty(page_data):
    """Cek apakah page kosong (semua 0xFF atau stepnum == 0)."""
    header = parse_header(page_data)
    return header['stepnum'] == 0


def cmd_list(bin_file):
    """Tampilkan semua page yang terisi."""
    with open(bin_file, 'rb') as f:
        print(f"{'Page':>5} | {'Name':<14} | {'Steps':>5} | {'Speed':>5} | {'Repeat':>6} | {'Next':>4} | {'Exit':>4}")
        print('-' * 65)
        for page_num in range(MAXNUM_PAGE):
            page_data = read_page(f, page_num)
            if len(page_data) < PAGE_SIZE:
                break
            if not is_page_empty(page_data):
                h = parse_header(page_data)
                print(f"{page_num:>5} | {h['name']:<14} | {h['stepnum']:>5} | {h['speed']:>5} | {h['repeat']:>6} | {h['next']:>4} | {h['exit']:>4}")


def cmd_info(bin_file, page_num):
    """Tampilkan detail satu page."""
    with open(bin_file, 'rb') as f:
        page_data = read_page(f, page_num)

    if is_page_empty(page_data):
        print(f"Page {page_num} kosong.")
        return

    h = parse_header(page_data)
    print(f"\n=== Page {page_num} ===")
    print(f"  Name    : {h['name']}")
    print(f"  Steps   : {h['stepnum']}")
    print(f"  Speed   : {h['speed']}")
    print(f"  Accel   : {h['accel']}")
    print(f"  Repeat  : {h['repeat']}")
    print(f"  Next    : {h['next']}")
    print(f"  Exit    : {h['exit']}")
    print(f"  Schedule: {'time-based' if h['schedule'] == 0x0a else 'speed-based'}")

    steps = parse_steps(page_data)
    print(f"\n  Steps detail:")
    for i, step in enumerate(steps[:h['stepnum']]):
        print(f"    Step {i+1}: pause={step['pause']} time={step['time']}")
        print(f"      Positions: {step['positions']}")


def cmd_copy(src_file, dst_file, page_nums):
    """Copy page dari src ke dst."""
    # Pastikan dst_file ada dan ukurannya benar
    expected_size = MAXNUM_PAGE * PAGE_SIZE
    if not os.path.exists(dst_file):
        print(f"Error: file tujuan '{dst_file}' tidak ada.")
        return

    with open(src_file, 'rb') as src, open(dst_file, 'r+b') as dst:
        for page_num in page_nums:
            page_data = read_page(src, page_num)
            if is_page_empty(page_data):
                print(f"Page {page_num}: kosong di source, skip.")
                continue
            h = parse_header(page_data)
            write_page(dst, page_num, page_data)
            print(f"Page {page_num} '{h['name']}' ({h['stepnum']} steps) → berhasil dicopy ke '{dst_file}'")

    print("\nSelesai!")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return

    cmd = sys.argv[1]

    if cmd == 'list':
        cmd_list(sys.argv[2])

    elif cmd == 'info':
        if len(sys.argv) < 4:
            print("Usage: action_bin_tool.py info <file.bin> <page_num>")
            return
        cmd_info(sys.argv[2], int(sys.argv[3]))

    elif cmd == 'copy':
        if len(sys.argv) < 5:
            print("Usage: action_bin_tool.py copy <src.bin> <dst.bin> <page_num> [page_num ...]")
            return
        page_nums = [int(p) for p in sys.argv[4:]]
        cmd_copy(sys.argv[2], sys.argv[3], page_nums)

    else:
        print(f"Command tidak dikenal: {cmd}")
        print(__doc__)


if __name__ == '__main__':
    main()
