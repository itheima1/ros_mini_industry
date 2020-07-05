#! /usr/bin/env python
# coding: utf-8

import rospy
import httplib
import json
import threading
import Queue
from itheima_msgs.msg import AssemblyIR, Order
from itheima_msgs.srv import AssemblyLineCtrl, AssemblyLineCtrlRequest


def update_status_audit(id):
    order_update_url = "/order/update"
    order_update_body = "status=1&id={}".format(id)
    conn = httplib.HTTPConnection(host, port)
    conn.request("POST", url=order_update_url, body=order_update_body,
                 headers={"Content-Type": "application/x-www-form-urlencoded"})
    conn.getresponse()
    conn.close()


def do_work():
    rate = rospy.Rate(interval)
    while not rospy.is_shutdown():
        data = queue.get()
        if data is None:
            continue

        # 机械臂开始上料
        msg = Order()
        msg.name = data["name"]
        msg.type = data["type"]
        msg.id = data["id"]
        msg.state = Order.STATE_WAITING
        publisher.publish(msg)

        rate.sleep()


def stop_assembly_line(index):
    client = rospy.ServiceProxy("/assembly/line_ctrl", AssemblyLineCtrl)
    client.wait_for_service()
    request = AssemblyLineCtrlRequest()
    request.line = index
    request.state = AssemblyLineCtrlRequest.STATE_STOP
    client.call(request)
    client.close()


def ir_callback(msg):
    if not isinstance(msg, AssemblyIR): return

    if msg.ir_1:
        # 1. 停止传送带2号
        stop_assembly_line(2)
        # 激光打标机应该开始打标工作
        # todo:

    if msg.ir_2:
        # 任务完成，机械臂开始将产品下架
        # 1. 停止传送带4号
        stop_assembly_line(4)
        # 2. 下架商品
        pass


if __name__ == '__main__':
    # 创建node
    node_name = "order_manager_node"
    rospy.init_node(node_name)

    host = rospy.get_param("~service_host", "192.168.1.100")
    port = rospy.get_param("~service_port", 3000)

    interval = rospy.get_param("~product_interval", 10)

    queue = Queue.Queue()

    publisher = rospy.Publisher("/order", String, queue_size=1000)
    rospy.Subscriber("/assembly/ir_status", AssemblyIR, ir_callback)

    threading.Thread(target=do_work).start()

    order_list_url = "/order/list"
    order_list_body = 'status=2'

    # engine = pyttsx.init()
    # voice = engine.getProperty('voice')
    # print voice

    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():

        try:
            # 获取新建订单，提醒订单审核
            conn = httplib.HTTPConnection(host, port)
            conn.request("POST", url=order_list_url, body=order_list_body,
                         headers={"Content-Type": "application/x-www-form-urlencoded"})

            response = conn.getresponse()
            # print response.status
            # print response.read()
            data = response.read()
            data = json.loads(data)
            print data
            if len(data) == 0:
                rate.sleep()
                continue

            for item in data:
                queue.put(item)

        except Exception as e:
            print e

        rate.sleep()
