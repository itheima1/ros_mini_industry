# coding: utf-8

import time
import threading
import serial


class AssemblyDevice:
    _config_start = {
        4: b'\xf3',
        3: b'\xf4',
        2: b'\xf5',
        1: b'\xf6'
    }

    _config_stop = {
        4: b'\x03',
        3: b'\x04',
        2: b'\x05',
        1: b'\x06'
    }

    _config_ir = [
        0xa7, 0xa8
    ]

    def __init__(self, port):
        self.port = port

        self.is_running = True

        self.line_states = [True, True, True, True]
        self.ir_states = [False, False]

        self._rate_time = 0.1
        self._wait_time = 0.25

        self.ser = serial.Serial(port=self.port, baudrate=9600)

        self.ir1_timer = threading.Timer(1, self._reset_ir1_state)
        self.ir2_timer = threading.Timer(1, self._reset_ir2_state)

    def connect(self):
        try:
            self._sync_states()
        except Exception as e:
            print e

    def disconnect(self):
        self.is_running = False
        self.stop_all()
        self.ser.close()

    def _sync_states(self):
        thread = threading.Thread(target=self._do_recv_states)
        thread.start()

    def _reset_ir1_state(self):
        self.ir_states[0] = False

    def _reset_ir2_state(self):
        self.ir_states[1] = False

    def _do_recv_states(self):
        try:
            while self.is_running:
                buffer = self.ser.read(1)

                if len(buffer) == 0:
                    break
                # print len(buffer)
                buffer = bytearray(buffer)
                print hex(buffer[0])
                if buffer[0] == AssemblyDevice._config_ir[0]:
                    print "1 on"
                    self.ir_states[0] = True
                    if self.ir1_timer.is_alive():
                        self.ir1_timer.cancel()
                        self.ir1_timer = threading.Timer(1, self._reset_ir1_state)
                    self.ir1_timer.start()
                elif buffer[0] == AssemblyDevice._config_ir[1]:
                    print "2 on"
                    self.ir_states[1] = True
                    if self.ir2_timer.is_alive():
                        self.ir2_timer.cancel()
                        self.ir2_timer = threading.Timer(1, self._reset_ir2_state)
                    self.ir2_timer.start()
        except Exception as e:
            pass

    def start(self, index):
        if index not in [1, 2, 3, 4]: return False

        try:
            data = AssemblyDevice._config_start[index]
            self.ser.write(data)

            self.line_states[index - 1] = True
            return True
        except Exception as e:
            print e

        return False
        # time.sleep(self._wait_time)
        # return self.line_states[index - 1] == True

    def stop(self, index):
        if index not in [1, 2, 3, 4]: return False

        try:
            data = AssemblyDevice._config_stop[index]
            self.ser.write(data)

            self.line_states[index - 1] = False
            return True
        except Exception as e:
            print e

        return False

    def start_all(self):
        try:
            data = b'\x00'
            self.ser.write(data)

            self.line_states = [True, True, True, True]
            return True
        except Exception as e:
            print e

        return False

    def stop_all(self):
        try:
            data = b'\xff'
            self.ser.write(data)

            self.line_states = [False, False, False, False]
            return True
        except Exception as e:
            print e

        return False
