# coding: utf-8

import serial
import time
import threading


class AssemblyDevice:
    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.is_running = True

        self.line_states = [True, True, True, True]
        self.ir_states = [False, False]

        self._rate_time = 0.1
        self._wait_time = 0.25

        self.ser = serial.Serial(port=self.port, baudrate=9600)

    def sync_states(self):
        thread = threading.Thread(target=self._do_snd_states)
        thread.start()

    def _do_snd_states(self):
        try:
            thread = threading.Thread(target=self._do_recv_states, args=(self.ser,))
            thread.start()

            while self.is_running:
                # 发送获取流水线状态指令
                self.ser.write(b'\xFE\x01\x00\x00\x00\x04\x29\xC6')
                time.sleep(self._rate_time)

                # 发送获取红外状态指令
                self.ser.write(b'\xFE\x02\x00\x00\x00\x04\x6D\xC6')
                time.sleep(self._rate_time)
            self.ser.write(b'\xFE\x01\x00\x00\x00\x04\x29\xC6')
            time.sleep(self._rate_time)
            self.ser.write(b'\xFE\x02\x00\x00\x00\x04\x6D\xC6')
            time.sleep(self._rate_time)
        except Exception as e:
            print e

    def _do_recv_states(self, ser):
        try:
            while self.is_running:
                result = ser.read(6)

                result = bytearray(result)

                if result[1] == 0x01:
                    # line states
                    self.line_states = [(result[3] & 0x01) == 0, (result[3] & 0x02) == 0, (result[3] & 0x03) == 0,
                                        (result[3] & 0x04) == 0]
                    # print "{} {} {}".format(self.line_states, result[3], "{0:b}".format(result[3]))
                if result[1] == 0x02:
                    # ir states
                    self.ir_states = [result[3] & 0x04, result[3] & 0x02]
        except Exception as e:
            print e
        finally:
            pass
            # ser.close()

    def start(self, index):
        if index not in [1, 2, 3, 4]: return

        try:
            data = b'\xFE\x05\x00\x00\x00\x00\xD9\xC5'
            if index == 2:
                data = b'\xFE\x05\x00\x01\x00\x00\x88\x05'
            elif index == 3:
                data = b'\xFE\x05\x00\x02\x00\x00\x78\x05'
            elif index == 4:
                data = b'\xFE\x05\x00\x03\x00\x00\x29\xC5'

            self.ser.write(data)
        except Exception as e:
            print e

        time.sleep(self._wait_time)
        return self.line_states[index - 1] == True

    def stop(self, index):
        if index not in [1, 2, 3, 4]: return

        try:
            data = b'\xFE\x05\x00\x00\xFF\x00\x98\x35'
            if index == 2:
                data = b'\xFE\x05\x00\x01\xFF\x00\xC9\xF5'
            elif index == 3:
                data = b'\xFE\x05\x00\x02\xFF\x00\x39\xF5'
            elif index == 4:
                data = b'\xFE\x05\x00\x03\xFF\x00\x68\x35'

            self.ser.write(data)
        except Exception as e:
            print e

        time.sleep(self._wait_time)
        return self.line_states[index - 1] == False

    def start_all(self):
        try:
            data = b'\xFE\x0F\x00\x00\x00\x04\x01\x00\x71\x92'
            self.ser.write(data)
        except Exception as e:
            print e

        time.sleep(self._wait_time)
        result = True
        for s in self.line_states:
            if not s:
                result = False
                break
        return result

    def stop_all(self):
        try:
            data = b'\xFE\x0F\x00\x00\x00\x04\x01\xFF\x31\xD2'
            self.ser.write(data)
        except Exception as e:
            print e

        time.sleep(self._wait_time)
        result = True
        for s in self.line_states:
            if s:
                result = False
                break
        return result