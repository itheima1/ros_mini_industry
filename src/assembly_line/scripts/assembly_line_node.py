#! /usr/bin/env python
# coding: utf-8

import rospy
from itheima_msgs.srv import AssemblyLineCtrl, AssemblyLineCtrlRequest, AssemblyLineCtrlResponse
from itheima_msgs.msg import AssemblyIR, AssemblyLine

# from driver import AssemblyDevice
from driver_ser import AssemblyDevice
import sys


ir_1 = False
ir_2 = False


def publish_line_state(states):
    line = AssemblyLine()
    line.line_1 = states[0]
    line.line_2 = states[1]
    line.line_3 = states[2]
    line.line_4 = states[3]
    line_publisher.publish(line)


def line_ctrl(line, state):
    result = False

    if state == AssemblyLineCtrlRequest.STATE_START:
        if line == AssemblyLineCtrlRequest.LINE_ALL:
            result = ad.start_all()
        else:
            result = ad.start(line)
    else:
        if line == AssemblyLineCtrlRequest.LINE_ALL:
            result = ad.stop_all()
        else:
            result = ad.stop(line)
    if not result:
        return line_ctrl(line, state)
    return True


def callback(request):
    if not isinstance(request, AssemblyLineCtrlRequest): return

    line = request.line
    state = request.state

    result = line_ctrl(line, state)

    response = AssemblyLineCtrlResponse()
    response.result = result
    return response


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
    # port = rospy.get_param("~assembly_port", 5566)

    port = "/dev/ttyUSB0"

    # 创建服务
    service = rospy.Service("/assembly/line_ctrl", AssemblyLineCtrl, callback)

    # 创建红外线的publisher
    ir_publisher = rospy.Publisher("/assembly/ir_state", AssemblyIR, queue_size=1000)

    # 创建流水线状态的publisher
    line_publisher = rospy.Publisher("/assembly/line_state", AssemblyLine, queue_size=1000)


    # 默认关闭所有的设备
    # ad = AssemblyDevice(host, port)
    ad = AssemblyDevice(port)
    ad.connect()
    ad.start_all()

    # 获取红外数据
    rate = rospy.Rate(10)
    # ad.get_ir_status(lambda: not rospy.is_shutdown(), lambda: rate.sleep(), ir_callback)
    # global ir_1, ir_2, line_status
    while not rospy.is_shutdown():
        line_states = ad.line_states
        ir_states = ad.ir_states
        # print ir_states
        # if ir_states[0] != ir_1 or ir_states[1] != ir_2:
        #     ir_1 = ir_states[0]
        #     ir_2 = ir_states[1]
        #
        #     msg = AssemblyIR()
        #     msg.ir_1 = ir_1
        #     msg.ir_2 = ir_2
        #     ir_publisher.publish(msg)
        ir_1 = ir_states[0]
        ir_2 = ir_states[1]

        msg = AssemblyIR()
        msg.ir_1 = ir_1
        msg.ir_2 = ir_2
        ir_publisher.publish(msg)

        # if line_states[0] != line_status[1] or \
        #         line_states[1] != line_status[2] or \
        #         line_states[2] != line_status[3] or \
        #         line_states[3] != line_status[4]:
        publish_line_state(line_states)

        rate.sleep()

    ad.disconnect()
    sys.exit(0)
