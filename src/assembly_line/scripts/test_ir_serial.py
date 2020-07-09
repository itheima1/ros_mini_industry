import serial
import threading
import time


def do_recv():
    while True:
        buffer = ser.read(6)
        buffer = bytearray(buffer)

        ir_1 = buffer[3] & 0x04 != 0
        ir_2 = buffer[3] & 0x01 != 0

        print "{} {}".format(ir_1, ir_2)


if __name__ == '__main__':
    ser = serial.Serial(port="/dev/ttyUSB1", baudrate=9600)

    threading.Thread(target=do_recv).start()

    while True:
        ser.write(b'\xFE\x02\x00\x00\x00\x04\x6D\xC6')
        time.sleep(0.1)
