#!/usr/bin/env python3
"""Print the serial output of the board for some seconds (only the Python standard library).

  serial_log.py [PORT] [SECONDS] [BAUD] [noreset]     defaults: /dev/ttyUSB0 20 115200

The board is reset with a DTR pulse (like the auto reset of the IDE), so the log starts with
the boot messages. "noreset" (anywhere in the arguments) listens to a running board: it does
not pulse DTR and leaves DTR on when it ends. Linux sets DTR when a port is opened, so the
board is only left running if the port was closed with DTR on before (a previous noreset run).
For an interactive console use "make monitor" (arduino-cli monitor).
"""
import fcntl
import os
import struct
import select
import sys
import termios
import time

BAUDS = {9600: termios.B9600, 57600: termios.B57600, 115200: termios.B115200}


def main():
    args = [a for a in sys.argv[1:] if a != 'noreset']
    reset = 'noreset' not in sys.argv[1:]
    port = args[0] if len(args) > 0 else '/dev/ttyUSB0'
    seconds = float(args[1]) if len(args) > 1 else 20
    baud = BAUDS[int(args[2]) if len(args) > 2 else 115200]

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                                        # iflag: raw
        attrs[1] = 0                                        # oflag
        #HUPCL: DTR drops when the port is closed, the next open resets the board again
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | (termios.HUPCL if reset else 0)
        attrs[3] = 0                                        # lflag: no echo, not canonical
        attrs[4] = attrs[5] = baud
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        if reset:
            #falling edge on the reset pin of the Arduino: DTR off, then on
            dtr = struct.pack('I', termios.TIOCM_DTR)
            fcntl.ioctl(fd, termios.TIOCMBIC, dtr)
            time.sleep(0.1)
            fcntl.ioctl(fd, termios.TIOCMBIS, dtr)
            termios.tcflush(fd, termios.TCIFLUSH)

        end = time.time() + seconds
        while time.time() < end:
            ready, _, _ = select.select([fd], [], [], 0.2)
            if ready:
                data = os.read(fd, 1024)
                sys.stdout.write(data.decode('latin-1').replace('\r', ''))
                sys.stdout.flush()
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
