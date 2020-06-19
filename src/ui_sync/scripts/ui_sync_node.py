#! /usr/bin/env python
# coding: utf-8

import rospy
from Connector import Connector

from std_msgs.msg import Float32MultiArray
from itheima_msgs.msg import AssemblyLine, AssemblyIR


def aubo_joints_callback(msg):
    if not isinstance(msg, Float32MultiArray):
        return

    connector.send_aubo_joints(msg.data)


def assembly_line_callback(msg):
    if not isinstance(msg, AssemblyLine):
        return
    lines = [msg.line_1, msg.line_2, msg.line_3, msg.line_4]
    connector.send_assembly_line(lines)


def assembly_ir_callback(msg):
    if not isinstance(msg, AssemblyIR):
        return
    irs = [msg.ir_1, msg.ir_2]
    connector.send_assembly_ir(irs)


if __name__ == '__main__':
    # 创建node
    node_name = "ui_sync_node"
    rospy.init_node(node_name)

    ui_host = rospy.get_param("ui_host", "192.168.0.137")
    ui_port = rospy.get_param("ui_port", 10008)

    connector = Connector(ip=ui_host, port=ui_port)
    connector.connect()

    # aubo joints
    rospy.Subscriber("/aubo/joints", Float32MultiArray, aubo_joints_callback)

    # assembly line
    rospy.Subscriber("/assembly/line_state", AssemblyLine, assembly_line_callback)

    # assembly ir
    rospy.Subscriber("/assembly/ir_state", AssemblyIR, assembly_ir_callback)

    rospy.spin()
