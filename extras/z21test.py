#!/usr/bin/env python3
"""Z21 LAN test client for MobaStation (only the Python standard library).

  z21test.py IP selftest              run all checks, exit code 0 = everything passed
  z21test.py IP info                  serial number, hardware info, central state
  z21test.py IP power on|off          track power
  z21test.py IP loco ADDR SPEED [r]   drive a loco with 128 steps (0-126, r = reverse)
  z21test.py IP turnout ADDR 0|1      switch a turnout (ADDR from 1)
  z21test.py IP listen [SECONDS]      subscribe to all broadcasts and print them

The selftest switches the track power on for a moment and uses loco 9999 and
turnout 512 (the last one the station keeps with TURNOUT_STATE_COUNT 512).
"""
import socket
import struct
import sys
import time

PORT = 21105


def x_record(*body):
    """LAN_X record with length, header 0x40 and XOR"""
    xor = 0
    for b in body:
        xor ^= b
    data = bytes(body) + bytes([xor])
    return struct.pack('<HH', len(data) + 4, 0x40) + data


def record(header, payload=b''):
    return struct.pack('<HH', len(payload) + 4, header) + payload


def loco_addr(addr):
    msb = (addr >> 8) & 0x3F
    if addr >= 128:
        msb |= 0xC0
    return msb, addr & 0xFF


def split_records(data):
    out = []
    while len(data) >= 4:
        length = struct.unpack_from('<H', data)[0]
        if length < 4 or length > len(data):
            break
        out.append(data[:length])
        data = data[length:]
    return out


def speed_step(value, steps):
    """speed byte of LAN_X_LOCO_INFO -> drive step (0 = stop, 'estop' = emergency stop)"""
    if steps == 28:
        code = ((value & 0x0F) << 1) | ((value >> 4) & 1)    # bit 4 is the lowest speed bit
        return 0 if code <= 1 else 'estop' if code <= 3 else code - 3
    v = value & (0x0F if steps == 14 else 0x7F)
    return 0 if v == 0 else 'estop' if v == 1 else v - 1


def describe(rec):
    header = struct.unpack_from('<H', rec, 2)[0]
    p = rec[4:]
    if header == 0x10:
        return 'SERIAL_NUMBER %d' % struct.unpack_from('<I', p)[0]
    if header == 0x1A:
        hw, fw = struct.unpack_from('<II', p)
        return 'HWINFO hw=0x%08X fw=0x%08X' % (hw, fw)
    if header == 0x51:
        return 'BROADCASTFLAGS 0x%08X' % struct.unpack_from('<I', p)[0]
    if header == 0x84:
        main, prog, filt, temp, supply, vcc, cs, cse = struct.unpack_from('<hhhhHHBB', p)
        return 'SYSTEMSTATE main=%dmA prog=%dmA temp=%d supply=%dmV state=0x%02X' % (main, prog, temp, supply, cs)
    if header == 0x80:
        return 'RMBUS group %d: %s' % (p[0], p[1:11].hex(' '))
    if header == 0x40:
        xh = p[0]
        if xh == 0x61:
            names = {0x00: 'TRACK_POWER_OFF', 0x01: 'TRACK_POWER_ON', 0x02: 'PROGRAMMING_MODE',
                     0x08: 'TRACK_SHORT_CIRCUIT', 0x12: 'CV_NACK_SC', 0x13: 'CV_NACK', 0x82: 'UNKNOWN_COMMAND'}
            return 'X ' + names.get(p[1], 'BC 0x61 0x%02X' % p[1])
        if xh == 0x81:
            return 'X STOPPED'
        if xh == 0x62:
            return 'X STATUS_CHANGED state=0x%02X' % p[2]
        if xh == 0x63:
            return 'X VERSION xbus=0x%02X id=0x%02X' % (p[2], p[3])
        if xh == 0xF3:
            return 'X FIRMWARE %d.%02X' % (p[2], p[3])
        if xh == 0x43:
            return 'X TURNOUT_INFO addr=%d zz=%d' % (((p[1] << 8) | p[2]) + 1, p[3] & 3)
        if xh == 0x64:
            return 'X CV_RESULT cv=%d value=%d' % (((p[2] << 8) | p[3]) + 1, p[4])
        if xh == 0xEF:
            addr = ((p[1] & 0x3F) << 8) | p[2]
            steps = {0: 14, 2: 28, 4: 128}.get(p[3] & 7, '?')
            return 'X LOCO_INFO addr=%d steps=%s %s step=%s F0=%d' % (
                addr, steps, 'fwd' if p[4] & 0x80 else 'rev', speed_step(p[4], steps), (p[5] >> 4) & 1)
        return 'X 0x%02X %s' % (xh, p.hex(' '))
    return 'header 0x%04X %s' % (header, p.hex(' '))


class Z21:
    def __init__(self, ip):
        self.addr = (ip, PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))

    def send(self, *records):
        self.sock.sendto(b''.join(records), self.addr)

    def receive(self, timeout=1.0):
        """all records that arrive until timeout"""
        recs = []
        end = time.time() + timeout
        while True:
            left = end - time.time()
            if left <= 0:
                return recs
            self.sock.settimeout(left)
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                return recs
            recs.extend(split_records(data))

    def ask(self, *records, timeout=1.0):
        self.send(*records)
        return self.receive(timeout)


def find(recs, header, xheader=None, db0=None):
    for r in recs:
        if struct.unpack_from('<H', r, 2)[0] != header:
            continue
        if xheader is not None and r[4] != xheader:
            continue
        if db0 is not None and r[5] != db0:
            continue
        return r
    return None


def selftest(z):
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print('%s  %s %s' % ('OK  ' if ok else 'FAIL', name, detail))

    r = z.ask(record(0x10))
    check('serial number', find(r, 0x10) is not None)
    r = z.ask(record(0x1A))
    hw = find(r, 0x1A)
    check('hardware info', hw is not None, describe(hw) if hw else '')
    # two records in one datagram, both must be answered
    r = z.ask(record(0x10) + record(0x1A))
    check('two records in one datagram', find(r, 0x10) is not None and find(r, 0x1A) is not None)
    # subscribe: all (0x01) + system state (0x100)
    z.ask(record(0x50, struct.pack('<I', 0x00000101)))
    r = z.ask(record(0x51))
    flags = find(r, 0x51)
    check('broadcast flags', flags is not None and struct.unpack_from('<I', flags, 4)[0] & 0x101 == 0x101)
    r = z.ask(x_record(0x21, 0x81))
    check('track power on', find(r, 0x40, 0x61, 0x01) is not None)
    r = z.ask(x_record(0x21, 0x24))
    st = find(r, 0x40, 0x62)
    check('central state normal', st is not None and st[6] == 0x00)
    msb, lsb = loco_addr(9999)
    r = z.ask(x_record(0xE4, 0x13, msb, lsb, 0x80 | 20))
    li = find(r, 0x40, 0xEF)
    check('loco drive 128 steps', li is not None and li[8] == 0x80 | 20, describe(li) if li else '')
    # 28 steps, step 10 forward: code 13 = 0b01101 -> R00V VVVV with V(LSB) in bit 4 = 0x96
    r = z.ask(x_record(0xE4, 0x12, msb, lsb, 0x96))
    li = find(r, 0x40, 0xEF)
    check('loco drive 28 steps', li is not None and li[7] & 7 == 2 and li[8] == 0x96, describe(li) if li else '')
    z.ask(x_record(0xE4, 0x13, msb, lsb, 0x80))
    fadr = 512 - 1     # inside TURNOUT_STATE_COUNT, higher turnouts report 'unknown'
    z.ask(x_record(0x53, fadr >> 8, fadr & 0xFF, 0x89))
    z.ask(x_record(0x53, fadr >> 8, fadr & 0xFF, 0x81))
    r = z.ask(x_record(0x43, fadr >> 8, fadr & 0xFF))
    ti = find(r, 0x40, 0x43)
    check('turnout info', ti is not None and ti[7] & 3 == 2, describe(ti) if ti else '')
    r = z.ask(record(0x85), timeout=1.0)
    check('system state', find(r, 0x84) is not None, describe(find(r, 0x84)) if find(r, 0x84) else '')
    r = z.ask(x_record(0x21, 0x80))
    check('track power off', find(r, 0x40, 0x61, 0x00) is not None)
    z.send(record(0x30))
    print('%d/%d checks passed' % (sum(results), len(results)))
    return all(results)


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    z = Z21(argv[1])
    cmd = argv[2]
    if cmd == 'selftest':
        return 0 if selftest(z) else 1
    if cmd == 'info':
        recs = z.ask(record(0x10), record(0x1A), x_record(0x21, 0x21), x_record(0x21, 0x24), x_record(0xF1, 0x0A))
    elif cmd == 'power':
        recs = z.ask(x_record(0x21, 0x81 if argv[3] == 'on' else 0x80))
    elif cmd == 'loco':
        msb, lsb = loco_addr(int(argv[3]))
        speed = min(int(argv[4]), 126)
        speed = speed + 1 if speed else 0    # 1 is the emergency stop
        rev = len(argv) > 5 and argv[5] == 'r'
        recs = z.ask(x_record(0xE4, 0x13, msb, lsb, (0 if rev else 0x80) | speed))
    elif cmd == 'turnout':
        fadr = int(argv[3]) - 1
        pos = int(argv[4]) & 1
        # subscribe first: the answer of a MobaBus module comes as broadcast
        z.ask(record(0x50, struct.pack('<I', 0x00000001)), timeout=0.2)
        z.send(x_record(0x53, fadr >> 8, fadr & 0xFF, 0x88 | pos))
        time.sleep(0.2)
        recs = z.ask(x_record(0x53, fadr >> 8, fadr & 0xFF, 0x80 | pos))
    elif cmd == 'listen':
        z.send(record(0x50, struct.pack('<I', 0x00010101)))
        end = time.time() + (float(argv[3]) if len(argv) > 3 else 30)
        while time.time() < end:
            for r in z.receive(1.0):
                print(time.strftime('%H:%M:%S'), describe(r))
        z.send(record(0x30))
        return 0
    else:
        print(__doc__)
        return 2
    if not recs:
        print('no answer from', argv[1])
        return 1
    for r in recs:
        print(describe(r))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
