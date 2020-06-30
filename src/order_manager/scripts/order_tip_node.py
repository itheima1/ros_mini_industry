#! /usr/bin/env python
# coding: utf-8

import rospy
import httplib
import json
import os
from playsound import playsound

current_dir = os.path.abspath(os.path.dirname(__file__))


def update_status_audit(id):
    order_update_url = "/order/update"
    order_update_body = "status=1&id={}".format(id)
    conn = httplib.HTTPConnection(host, port)
    conn.request("POST", url=order_update_url, body=order_update_body,
                 headers={"Content-Type": "application/x-www-form-urlencoded"})
    conn.getresponse()
    conn.close()


if __name__ == '__main__':
    # 创建node
    node_name = "order_tip_node"
    rospy.init_node(node_name)

    host = rospy.get_param("~service_host", "192.168.1.100")
    port = rospy.get_param("~service_port", 3000)

    order_list_url = "/order/list"
    order_list_body = 'status=0'

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

            if len(data) == 1:
                playsound(current_dir + "/order_tip1.ogg")
            else:
                playsound(current_dir + "/order_tip2.ogg")

            for item in data:
                update_status_audit(item["id"])

            conn.close()
        except Exception as e:
            print e

        rate.sleep()
