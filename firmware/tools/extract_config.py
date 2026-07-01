#!/usr/bin/env python3
"""
extract_config.py — Extract /config files from an ESP32 FFat partition image.

Handles FAT12 and FAT16.  No external dependencies.

Usage:
    python extract_config.py <image.bin> <output_dir>
"""

import os, struct, sys
from pathlib import Path

def fat_entry(fat: bytes, c: int, fat_type: int) -> int:
    """Read FAT entry for cluster c."""
    if fat_type == 12:
        byte_off = c * 3 // 2
        word = struct.unpack_from('<H', fat, byte_off)[0]
        return (word >> 4) if (c & 1) else (word & 0x0FFF)
    else:  # FAT16
        return struct.unpack_from('<H', fat, c * 2)[0]

def eoc(val: int, fat_type: int) -> bool:
    return val >= (0xFF8 if fat_type == 12 else 0xFFF8)

def read_chain(img: bytes, fat: bytes, first_c: int, clus: int,
               data_start_byte: int, fat_type: int) -> bytes:
    data = b''
    c = first_c
    while c >= 2 and not eoc(c, fat_type):
        off = data_start_byte + (c - 2) * clus
        data += img[off:off + clus]
        c = fat_entry(fat, c, fat_type)
    return data

def parse_entries(raw: bytes):
    """Yield (name, attr, first_cluster, size) for each valid entry."""
    lfn_parts = {}
    i = 0
    while i + 32 <= len(raw):
        e = raw[i:i + 32]
        i += 32
        if e[0] == 0x00:
            break
        if e[0] == 0xE5:
            lfn_parts.clear()
            continue
        attr = e[11]
        if attr == 0x0F:                        # LFN entry
            seq = e[0] & 0x3F
            chars_raw = e[1:11] + e[14:26] + e[28:32]
            chars = ''
            for j in range(0, len(chars_raw), 2):
                code = struct.unpack_from('<H', chars_raw, j)[0]
                if code == 0x0000:
                    break
                if code != 0xFFFF:
                    chars += chr(code)
            lfn_parts[seq] = chars
            continue
        # Short-name entry
        if lfn_parts:
            name = ''.join(lfn_parts[k] for k in sorted(lfn_parts))
            lfn_parts.clear()
        else:
            base = e[0:8].decode('ascii', 'replace').rstrip()
            ext  = e[8:11].decode('ascii', 'replace').rstrip()
            name = (base + '.' + ext) if ext else base
        if name in ('.', '..'):
            continue
        cluster = struct.unpack_from('<H', e, 26)[0]
        size    = struct.unpack_from('<L', e, 28)[0]
        yield name, attr, cluster, size

def extract(image_path: str, out_dir: str):
    with open(image_path, 'rb') as f:
        img = f.read()

    # ── Parse BPB ─────────────────────────────────────────────────────────────
    sector      = struct.unpack_from('<H', img, 11)[0]
    clus_secs   = img[13]
    reserved    = struct.unpack_from('<H', img, 14)[0]
    nfats       = img[16]
    root_entries= struct.unpack_from('<H', img, 17)[0]
    fat_secs    = struct.unpack_from('<H', img, 22)[0]
    clus        = sector * clus_secs
    root_secs   = root_entries * 32 // sector
    data_start  = reserved + nfats * fat_secs + root_secs
    total_secs  = struct.unpack_from('<H', img, 19)[0] or struct.unpack_from('<L', img, 32)[0]
    nclusters   = (total_secs - data_start) // clus_secs
    fat_type    = 12 if nclusters < 4086 else 16
    data_off    = data_start * sector

    fat_off = reserved * sector
    fat     = img[fat_off:fat_off + fat_secs * sector]

    print(f'FAT{fat_type}, {nclusters} clusters, {clus} B/cluster, data@sector {data_start}')

    # ── Root directory ─────────────────────────────────────────────────────────
    root_off = (reserved + nfats * fat_secs) * sector
    root_raw = img[root_off:root_off + root_secs * sector]

    def extract_dir(raw_bytes: bytes, dest: Path):
        dest.mkdir(parents=True, exist_ok=True)
        for name, attr, cluster, size in parse_entries(raw_bytes):
            if attr & 0x10:                     # directory
                if cluster >= 2:
                    sub_raw = read_chain(img, fat, cluster, clus, data_off, fat_type)
                    extract_dir(sub_raw, dest / name)
            else:                               # file
                if cluster >= 2:
                    data = read_chain(img, fat, cluster, clus, data_off, fat_type)[:size]
                else:
                    data = b''
                out_file = dest / name
                out_file.write_bytes(data)
                print(f'  extracted: {out_file}  ({size} B)')

    # Only extract the /config directory
    config_cluster = None
    for name, attr, cluster, _ in parse_entries(root_raw):
        if name.lower() == 'config' and (attr & 0x10):
            config_cluster = cluster
            break

    if config_cluster is None:
        print('No /config directory found in image — nothing to extract.')
        return False

    config_raw = read_chain(img, fat, config_cluster, clus, data_off, fat_type)
    extract_dir(config_raw, Path(out_dir))
    return True

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    ok = extract(sys.argv[1], sys.argv[2])
    sys.exit(0 if ok else 2)
