#!/usr/bin/env python
# coding:utf-8

# 1、导入socket模块
import socket
import threading
import Queue
import json


class LaserDriver:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.is_running = False

        self.task_map = {}

    def connect(self):
        try:
            self.client.connect((self.host, self.port))
            self.is_running = True
            self._sync_states()
        except Exception as e:
            print "1"
            print e

    def _sync_states(self):
        thread = threading.Thread(target=self._do_recv_states)
        thread.start()

    def _do_recv_states(self):
        try:

            while self.is_running:
                buffer = self.client.recv(1024 * 4)
                if len(buffer) == 0:
                    break
                value = buffer.decode("utf-8")
                data = json.loads(value)
                id = data["id"]

                if self.task_map.has_key(id):
                    self.task_map[id].put(data)

        except Exception as e:
            print e
            self._clear_task_map()

    def _clear_task_map(self):
        for key in self.task_map.keys():
            self.task_map[key].put(None)

    def disconnect(self):
        self.client.close()

    def send(self, id, type, name, offset_x, offset_y, degree):
        queue = Queue.Queue()
        self.task_map[id] = queue

        # 发送请求
        self.client.send(json.dumps({
            "id": id,
            "type": type,
            "name": name,
            "offset": [
                offset_x, offset_y
            ],
            "degree": degree
        }))

        # blocking wait
        result = queue.get()
        # 移除
        del self.task_map[id]

        return result
