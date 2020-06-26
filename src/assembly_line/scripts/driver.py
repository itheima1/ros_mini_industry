# coding: utf-8

import socket
import time
import threading


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

    _config_ir = {
        b'\xa7': 0,
        b'\xa8': 1
    }

    def __init__(self, host, port):
        self.host = host
        self.port = port

        self.is_running = True

        self.line_states = [True, True, True, True]
        self.ir_states = [False, False]

        self._rate_time = 0.1
        self._wait_time = 0.25

        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        try:
            self.client.connect((self.host, self.port))
            self._sync_states()
        except Exception as e:
            print e

    def disconnect(self):
        self.client.close()
        self.is_running = False

    def _sync_states(self):
        thread = threading.Thread(target=self._do_recv_states)
        thread.start()

    def _do_recv_states(self):
        while self.is_running:
            buffer = self.client.recv(1)
            buffer = bytearray(buffer)

            index = AssemblyDevice._config_ir[buffer[0]]
            self.ir_states[index] = True

    def start(self, index):
        if index not in [1, 2, 3, 4]: return False

        try:
            data = AssemblyDevice._config_start[index]
            self.client.send(data)

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
            self.client.send(data)

            self.line_states[index - 1] = False
            return True
        except Exception as e:
            print e

        return False

    def start_all(self):
        try:
            data = b'\xff'
            self.client.send(data)

            self.line_states = [True, True, True, True]
            return True
        except Exception as e:
            print e

        return False

    def stop_all(self):
        try:
            data = b'\x00'
            self.client.send(data)

            self.line_states = [False, False, False, False]
            return True
        except Exception as e:
            print e

        return False