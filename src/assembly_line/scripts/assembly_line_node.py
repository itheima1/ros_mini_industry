#! /usr/bin/env python
# coding: utf-8

import rospy
from itheima_msgs.srv import AssemblyLineCtrl, AssemblyLineCtrlRequest, AssemblyLineCtrlResponse
from itheima_msgs.msg import AssemblyIR

from driver import AssemblyLine as AL

line_status = {
    1: False,
    2: False,
    3: False,
    4: False
}

ir_1 = False
ir_2 = False


def callback(request):
    if not isinstance(request, AssemblyLineCtrlRequest): return

    global line_status

    line = request.line
    state = request.state

    if state == AssemblyLineCtrlRequest.STATE_START:
        if line == AssemblyLineCtrlRequest.LINE_ALL:
            al.start_all()
            line_status = {
                1: True,
                2: True,
                3: True,
                4: True
            }
        else:
            al.start(line)
            line_status[line] = True
    else:
        if line == AssemblyLineCtrlRequest.LINE_ALL:
            al.stop_all()
            line_status = {
                1: False,
                2: False,
                3: False,
                4: False
            }
        else:
            al.stop(line)
            line_status[line] = False

    return AssemblyLineCtrlResponse()


if __name__ == '__main__':
    # 创建node
    node_name = "assembly_line_node"
    rospy.init_node(node_name)

    host = rospy.get_param("assembly_host", "10.10.100.254")
    port = rospy.get_param("assembly_port", 5566)

    al = AL(host, port)
    al.stop_all()

    # 创建服务
    service = rospy.Service("/assembly/line_ctrl", AssemblyLineCtrl, callback)

    # 创建publisher
    publisher = rospy.Publisher("/assembly/ir", AssemblyIR, queue_size=1000)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        status = al.get_ir_status()
        if status is not None:
            rate.sleep()
            continue

        msg = AssemblyIR()
        msg.ir_1 = status[0]
        msg.ir_2 = status[1]
        publisher.publish(msg)

        rate.sleep()

    rospy.spin()
