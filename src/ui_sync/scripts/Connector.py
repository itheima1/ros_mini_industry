#! /usr/bin/env python
# coding=utf-8

import socket
import threading
import Queue as queue
import json
import time


class Connector:
    def __init__(self, ip="127.0.0.1", port=10008):
        self._ip = ip
        self._port = port

        # 连接状态
        self._connected = False
        # 连接状态回调
        self._connected_callback = None
        # socket client
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # send Queue
        self._queue = queue.Queue()

    def _set_connected(self, connected):
        self._connected = connected
        if self._connected_callback is not None:
            self._connected_callback(self._connected)

    def _do_connect(self, callback=None):
        try:
            # socket client
            self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._client.connect((self._ip, self._port))
            self._set_connected(True)
            if callback is not None:
                callback()

            # 接收
            recv_thread = threading.Thread(target=lambda: self._do_recv())
            recv_thread.start()

            # 发送
            send_thread = threading.Thread(target=lambda: self._do_send())
            send_thread.start()
        except Exception as e:
            print e

    def _do_recv(self):
        try:
            while self._connected:
                recv = self._client.recv(1024)
                if len(recv) == 0:
                    # 断开链接了
                    self.disconnect()
                    break
                print recv.decode("utf-8")
        except Exception as e:
            # 断开链接了
            self.disconnect()

    def _do_send(self):
        try:
            while self._connected:
                data = self._queue.get()
                if data is None:
                    continue

                self._client.send(data)
        except Exception as e:
            print e

    def connect(self, callback=None):
        if self._connected:
            return
        thread = threading.Thread(target=lambda: self._do_connect(callback))
        thread.start()

    def disconnect(self):
        if self._connected:
            self._client.close()
            self._set_connected(False)
            self._queue.put(None)

    def is_connected(self):
        return self._connected

    def on_connected_change(self, callback):
        """
        连接状态监听
        :param callback: 回调，包含一个参数，当前连接状态
        :return:
        """
        self._connected_callback = callback

    def _gen_msg(self, type):
        buffer = []
        # 标记头
        buffer.append(0xfa)
        buffer.append(0xa9)
        # 请求类型
        buffer.append(type)

        return buffer

    def send_aubo_joints(self, joints):
        # aubo关节角数据类型
        buffer = self._gen_msg(0x01)

        # 数据长度
        size = 3 * 6
        buffer.append((size & 0xff00) >> 8)
        buffer.append((size & 0x00ff) >> 0)

        for joint in joints:
            j = int((joint + 360) * 10000)
            buffer.append((j & 0xff0000) >> 16)
            buffer.append((j & 0x00ff00) >> 8)
            buffer.append((j & 0x0000ff) >> 0)

        # for b in data:
        #     print hex(b),
        # print
        self._queue.put(bytearray(buffer))

    def send_assembly_line(self, lines):
        # assembly line 类型
        buffer = self._gen_msg(0x02)

        # 数据长度
        size = 4
        buffer.append((size & 0xff00) >> 8)
        buffer.append((size & 0x00ff) >> 0)

        # 数据
        for line in lines:
            buffer.append(0x01 if line else 0x00)

        self._queue.put(bytearray(buffer))

    def send_assembly_ir(self, irs):
        # assembly ir 类型
        buffer = self._gen_msg(0x03)

        # 数据长度
        size = len(irs)
        buffer.append((size & 0xff00) >> 8)
        buffer.append((size & 0x00ff) >> 0)

        # 数据
        for ir in irs:
            buffer.append(0x01 if ir else 0x00)

        self._queue.put(bytearray(buffer))