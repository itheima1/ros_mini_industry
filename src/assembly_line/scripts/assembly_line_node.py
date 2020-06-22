#! /usr/bin/env python
# coding: utf-8

import rospy
from itheima_msgs.srv import AssemblyLineCtrl, AssemblyLineCtrlRequest, AssemblyLineCtrlResponse
from itheima_msgs.msg import AssemblyIR, AssemblyLine

from driver import AssemblyDevice

line_status = {
    1: False,
    2: False,
    3: False,
    4: False
}

ir_1 = False
ir_2 = False


def publish_line_state():
    line = AssemblyLine()
    line.line_1 = line_status[1]
    line.line_2 = line_status[2]
    line.line_3 = line_status[3]
    line.line_4 = line_status[4]
    line_publisher.publish(line)


def callback(request):
    if not isinstance(request, AssemblyLineCtrlRequest): return

    global line_status

    line = request.line
    state = request.state

    if state == AssemblyLineCtrlRequest.STATE_START:
        if line == AssemblyLineCtrlRequest.LINE_ALL:
            ad.start_all()
            line_status = {
                1: True,
                2: True,
                3: True,
                4: True
            }
        else:
            ad.start(line)
            line_status[line] = True
    else:
        if line == AssemblyLineCtrlRequest.LINE_ALL:
            ad.stop_all()
            line_status = {
                1: False,
                2: False,
                3: False,
                4: False
            }
        else:
            ad.stop(line)
            line_status[line] = False

    # 推送流水线状态
    publish_line_state()

    return AssemblyLineCtrlResponse()


def ir_callback(status):
    global ir_1, ir_2
    if status[0] != ir_1 or status[1] != ir_2:
        ir_1 = status[0]
        ir_2 = status[1]

        msg = AssemblyIR()
        msg.ir_1 = ir_1
        msg.ir_2 = ir_2
        ir_publisher.publish(msg)


if __name__ == '__main__':
    # 创建node
    node_name = "assembly_line_node"
    rospy.init_node(node_name)

    # host = rospy.get_param("assembly_host", "10.10.100.254")
    host = rospy.get_param("~assembly_host", "192.168.1.104")
    port = rospy.get_param("~assembly_port", 5566)

    # 默认关闭所有的设备
    ad = AssemblyDevice(host, port)
    ad.stop_all()

    # 创建服务
    service = rospy.Service("/assembly/line_ctrl", AssemblyLineCtrl, callback)

    # 创建红外线的publisher
    ir_publisher = rospy.Publisher("/assembly/ir_state", AssemblyIR, queue_size=1000)

    # 创建流水线状态的publisher
    line_publisher = rospy.Publisher("/assembly/line_state", AssemblyLine, queue_size=1000)

    # 获取红外数据
    rate = rospy.Rate(10)
    ad.get_ir_status(lambda: not rospy.is_shutdown(), lambda: rate.sleep(), ir_callback)

    rospy.spin()
