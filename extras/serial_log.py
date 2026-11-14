#!/usr/bin/env python3
"""Print the serial output of the board for some seconds (only the Python standard library).

  serial_log.py [PORT] [SECONDS] [BAUD] [noreset]     defaults: /dev/ttyUSB0 20 115200

The board is reset with a DTR pulse (like the auto reset of the IDE), so the log starts with
the boot messages. "noreset" only listens to a running board.
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
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    baud = BAUDS[int(sys.argv[3]) if len(sys.argv) > 3 else 115200]
    reset = not (len(sys.argv) > 4 and sys.argv[4] == 'noreset')

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                                        # iflag: raw
        attrs[1] = 0                                        # oflag
        #HUPCL: DTR drops when the port is closed, the next open resets the board again
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.HUPCL
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
