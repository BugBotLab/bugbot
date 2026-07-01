#!/usr/bin/env python3
"""
make_ffat.py — Build a FAT16 image for the ESP32 FFat partition.

Produces a binary that can be flashed with esptool.  No external
dependencies — only the Python standard library is needed.

Usage:
    python make_ffat.py <src_dir> <output.bin> [size_kb]

Default size_kb = 3776  (tinyuf2-partitions-8MB-noota.csv)
"""

import os, struct, time
from pathlib import Path

# ── FAT16 geometry ────────────────────────────────────────────────────────────
# ESP-IDF FATFS (ChaN FatFs) auto-selects cluster size = 1 sector (512 B) for
# the 3776 KB partition.  That yields ~7456 clusters → FAT16 range (≥4086).
# Using 8-sector clusters would give 938 clusters → driver treats image as
# FAT12 (entries are 1.5 B not 2 B) → unreadable.  Keep CLUS_SECS = 1.
SECTOR       = 512
CLUS_SECS    = 1          # 512-byte clusters → ~7456 clusters → FAT16
CLUS         = SECTOR * CLUS_SECS
RESERVED     = 4          # reserved sectors (boot + padding)
NFATS        = 2
ROOT_ENTRIES = 512        # standard for FAT16
ROOT_SECS    = ROOT_ENTRIES * 32 // SECTOR   # 32 sectors = 16 KB
EOC          = 0xFFFF     # end-of-chain marker

# ── LFN helpers ───────────────────────────────────────────────────────────────

def lfn_checksum(name83: bytes) -> int:
    """Microsoft LFN checksum over the 11-byte short name."""
    s = 0
    for b in name83:
        s = ((((s & 1) << 7) | (s >> 1)) + b) & 0xFF
    return s

def make_short_name(name: str, used: set) -> bytes:
    """Return a unique, uppercase 11-byte 8.3 name."""
    u = name.upper()
    dot = u.rfind('.')
    base, ext = (u[:dot], u[dot+1:]) if dot >= 0 else (u, '')
    bad = set(' "*/:<>?\\|+,.;=[]')
    base = ''.join('_' if c in bad else c for c in base)
    ext  = ''.join('_' if c in bad else c for c in ext)[:3].ljust(3)

    def pack(b):
        return (b.ljust(8) + ext).encode('ascii', 'replace')

    cand = pack(base[:8])
    if cand not in used:
        used.add(cand)
        return cand
    for n in range(1, 10000):
        tail = f'~{n}'
        cand = pack(base[:8 - len(tail)] + tail)
        if cand not in used:
            used.add(cand)
            return cand
    raise ValueError(f'Cannot make 8.3 name for {name!r}')

def lfn_entries(name: str, name83: bytes) -> list:
    """
    Return list of 32-byte LFN dir entries in directory order
    (highest sequence number first, i.e. the LAST chunk of the name comes first).
    """
    # UTF-16LE characters as 2-byte items
    chars = [name[i:i+1].encode('utf-16-le') for i in range(len(name))]
    chars.append(b'\x00\x00')          # null terminator
    while len(chars) % 13:
        chars.append(b'\xff\xff')      # padding

    chk = lfn_checksum(name83)
    n   = len(chars) // 13
    entries = []
    for i in range(n):
        seq   = i + 1
        order = seq | (0x40 if seq == n else 0)   # 0x40 = "last LFN entry" flag
        chunk = chars[i * 13:(i + 1) * 13]
        c1 = b''.join(chunk[0:5])     # characters 1-5  (10 bytes)
        c2 = b''.join(chunk[5:11])    # characters 6-11 (12 bytes)
        c3 = b''.join(chunk[11:13])   # characters 12-13 (4 bytes)
        e = bytearray(32)
        e[0]     = order
        e[1:11]  = c1
        e[11]    = 0x0F   # LFN attribute
        e[12]    = 0x00   # type (reserved)
        e[13]    = chk
        e[14:26] = c2
        e[26:28] = b'\x00\x00'   # cluster (must be 0 for LFN)
        e[28:32] = c3
        entries.append(bytes(e))
    # Directory order: LAST chunk first (highest seq, with 0x40), FIRST chunk last
    return list(reversed(entries))

# ── Directory entry ───────────────────────────────────────────────────────────

def dir_entry(name83: bytes, attr: int, cluster: int, size: int, mtime=None) -> bytes:
    t     = time.localtime(mtime) if mtime else time.localtime()
    fdate = ((max(0, t.tm_year - 1980)) << 9) | (t.tm_mon << 5) | t.tm_mday
    ftime = (t.tm_hour << 11) | (t.tm_min << 5) | (t.tm_sec // 2)
    e = bytearray(32)
    e[0:11] = name83
    e[11]   = attr
    struct.pack_into('<H', e, 14, ftime)           # create time
    struct.pack_into('<H', e, 16, fdate)           # create date
    struct.pack_into('<H', e, 18, fdate)           # last access date
    struct.pack_into('<H', e, 22, ftime)           # modified time
    struct.pack_into('<H', e, 24, fdate)           # modified date
    struct.pack_into('<H', e, 26, cluster & 0xFFFF)
    struct.pack_into('<L', e, 28, size)
    return bytes(e)

# ── Image builder ─────────────────────────────────────────────────────────────

def make_fat16(src: str, out: str, size_bytes: int):
    total_secs = size_bytes // SECTOR

    # Iteratively resolve fat_size (circular dependency)
    fat_size = 4
    for _ in range(4):
        data_start = RESERVED + NFATS * fat_size + ROOT_SECS
        nclusters  = (total_secs - data_start) // CLUS_SECS
        fat_size   = max(1, (nclusters * 2 + 4 + SECTOR - 1) // SECTOR)
    data_start = RESERVED + NFATS * fat_size + ROOT_SECS
    nclusters  = (total_secs - data_start) // CLUS_SECS

    img = bytearray(size_bytes)
    fat = bytearray(fat_size * SECTOR)
    # Reserved FAT entries
    struct.pack_into('<H', fat, 0, 0xFFF8)   # media descriptor copy
    struct.pack_into('<H', fat, 2, 0xFFFF)   # end-of-chain

    next_c = [2]

    def alloc():
        c = next_c[0]
        if c >= nclusters + 2:
            raise RuntimeError('FAT image is full')
        next_c[0] += 1
        return c

    def c_off(c):
        return (data_start + (c - 2) * CLUS_SECS) * SECTOR

    def set_fat(c, v):
        struct.pack_into('<H', fat, c * 2, v)

    def write_data(data: bytes) -> int:
        """Write file data into cluster chain; return first cluster (0 if empty)."""
        if not data:
            return 0
        cs = []
        for i in range(0, len(data), CLUS):
            c = alloc()
            cs.append(c)
            chunk = data[i:i + CLUS]
            off = c_off(c)
            img[off:off + len(chunk)] = chunk
        for i in range(len(cs) - 1):
            set_fat(cs[i], cs[i + 1])
        set_fat(cs[-1], EOC)
        return cs[0]

    def write_dir(path: Path, parent_c: int) -> int:
        """
        Recursively populate a directory.  Returns cluster number of this
        directory (0 for root, which occupies the fixed root area).
        parent_c=-1 signals this is the root.
        """
        is_root = (parent_c < 0)
        if is_root:
            buf  = bytearray(ROOT_SECS * SECTOR)
            my_c = 0
        else:
            my_c = alloc()
            set_fat(my_c, EOC)
            buf  = bytearray(CLUS)

        idx   = [0]
        used83: set = set()

        def append(e: bytes):
            off = idx[0] * 32
            if off + 32 > len(buf):
                raise RuntimeError(f'Directory full in {path}')
            buf[off:off + 32] = e
            idx[0] += 1

        # Dot entries (subdirectories only)
        if not is_root:
            append(dir_entry(b'.          ', 0x10, my_c,          0))
            append(dir_entry(b'..         ', 0x10, max(0, parent_c), 0))

        items = sorted(path.iterdir(), key=lambda p: (p.is_dir(), p.name.lower()))
        for item in items:
            if item.name.startswith('.'):
                continue
            n83   = make_short_name(item.name, used83)
            mtime = item.stat().st_mtime

            # LFN entries (always written — preserves case for ESP-IDF FATFS)
            for e in lfn_entries(item.name, n83):
                append(e)

            if item.is_file():
                data = item.read_bytes()
                fc   = write_data(data)
                append(dir_entry(n83, 0x20, fc, len(data), mtime))
                print(f'  {"FILE":4s}  {item.relative_to(path.parent)}  ({len(data):,} B)')
            elif item.is_dir():
                sub_c = write_dir(item, my_c)
                append(dir_entry(n83, 0x10, sub_c, 0, mtime))
                print(f'  {"DIR":4s}  {item.relative_to(path.parent)}/')

        # Write directory buffer into image
        if is_root:
            off = (RESERVED + NFATS * fat_size) * SECTOR
            img[off:off + ROOT_SECS * SECTOR] = buf
        else:
            img[c_off(my_c):c_off(my_c) + CLUS] = buf

        return my_c

    # ── Populate filesystem ────────────────────────────────────────────────────
    print(f'Building FAT16 image from {src} ...')
    write_dir(Path(src), -1)

    # ── Boot sector ───────────────────────────────────────────────────────────
    boot = bytearray(SECTOR)
    boot[0:3]   = b'\xEB\x58\x90'    # jmp + NOP (required by some implementations)
    boot[3:11]  = b'MSDOS5.0'        # OEM name
    struct.pack_into('<H', boot, 11, SECTOR)           # bytes per sector
    boot[13]    = CLUS_SECS                            # sectors per cluster
    struct.pack_into('<H', boot, 14, RESERVED)         # reserved sectors
    boot[16]    = NFATS                                # number of FATs
    struct.pack_into('<H', boot, 17, ROOT_ENTRIES)     # root entry count
    ts16 = total_secs if total_secs < 65536 else 0
    struct.pack_into('<H', boot, 19, ts16)             # total sectors (16-bit)
    boot[21]    = 0xF8                                 # media type (fixed disk)
    struct.pack_into('<H', boot, 22, fat_size)         # sectors per FAT
    struct.pack_into('<H', boot, 24, 63)               # sectors per track
    struct.pack_into('<H', boot, 26, 255)              # number of heads
    struct.pack_into('<L', boot, 28, 0)                # hidden sectors
    ts32 = total_secs if total_secs >= 65536 else 0
    struct.pack_into('<L', boot, 32, ts32)             # total sectors (32-bit)
    boot[36]    = 0x80                                 # drive number
    boot[37]    = 0x00                                 # reserved1
    boot[38]    = 0x29                                 # boot signature
    struct.pack_into('<L', boot, 39, 0xBEEFDEAD)      # volume serial number
    boot[43:54] = b'BUGBOT     '                       # volume label (11 bytes)
    boot[54:62] = b'FAT16   '                          # FS type (8 bytes)
    boot[510]   = 0x55
    boot[511]   = 0xAA
    img[0:SECTOR] = boot

    # ── FAT copies ────────────────────────────────────────────────────────────
    fat_off = RESERVED * SECTOR
    img[fat_off:fat_off + fat_size * SECTOR]                       = fat
    img[fat_off + fat_size * SECTOR:fat_off + 2 * fat_size * SECTOR] = fat

    # ── Write ─────────────────────────────────────────────────────────────────
    with open(out, 'wb') as f:
        f.write(img)

    used   = next_c[0] - 2
    kb_cap = size_bytes // 1024
    kb_use = used * CLUS // 1024
    print(f'\nImage: {out}')
    print(f'  {kb_cap} KB partition  |  {nclusters} clusters  |  {used} used ({kb_use} KB)')


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    make_fat16(sys.argv[1], sys.argv[2],
               int(sys.argv[3]) * 1024 if len(sys.argv) > 3 else 3776 * 1024)
