#! /usr/bin/env python
# coding: utf-8

import rospy
import httplib
import json
import threading
import Queue
from itheima_msgs.msg import AssemblyIR, Order
from itheima_msgs.srv import AssemblyLineCtrl, AssemblyLineCtrlRequest
import actionlib
from actionlib import TerminalState, CommState
from actionlib import ClientGoalHandle
from itheima_msgs.msg import ArmWorkAction, ArmWorkGoal, ArmWorkFeedback, ArmWorkResult
from itheima_msgs.msg import LaserMarkAction, LaserMarkGoal, LaserMarkFeedback, LaserMarkResult


STATE_INITED = 0  # 初始化完成
STATE_IN_TRANSPORTED = 1  # 运送输入完成
STATE_FEEDED = 2  # 上料完成
STATE_MARKED = 3  # 打标完成
STATE_BLANDED = 4  # 下料完成
STATE_OUT_TRANSPORTED = 5  # 运送输出完成


def update_status(id, state):
    # 0: 新建订单
    # 1: 等待审核
    # 2: 审核通过，等待生产
    # 3: 正在生产
    # 4: 生产完成
    # 5: 生产出问题
    order_update_url = "/order/update"
    order_update_body = "status={}&id={}".format(state, id)
    conn = httplib.HTTPConnection(host, port)
    conn.request("POST", url=order_update_url, body=order_update_body,
                 headers={"Content-Type": "application/x-www-form-urlencoded"})
    conn.getresponse()
    conn.close()


def update_status_producting(id):
    update_status(3, id)


def update_status_completed(id):
    update_status(4, id)


def update_status_error(id):
    update_status(5, id)


def start_aubo_blanding():
    # 创建客户端
    client = actionlib.ActionClient("/aubo/ctrl", ArmWorkAction)
    # 等待连接服务器
    client.wait_for_server()

    # 发送请求
    goal = ArmWorkGoal()
    goal.type = 1
    goal_handle = client.send_goal(goal)
    result = client.wait_for_server(rospy.Duration(10))
    if result:
        state = goal_handle.get_terminal_state()
    return True


def start_aubo_feeding():
    # 创建客户端
    client = actionlib.ActionClient("/aubo/ctrl", ArmWorkAction)
    # 等待连接服务器
    client.wait_for_server()

    # 发送请求
    goal = ArmWorkGoal()
    goal.type = 0
    goal_handle = client.send_goal(goal)
    result = client.wait_for_server(rospy.Duration(10))
    if result:
        terminal_state = goal_handle.get_terminal_state()
        get_result = goal_handle.get_result()

    return True


def find_item(state):
    for item in work_list:
        if item["state"] == state:
            return item
    return None


def do_work():
    rate = rospy.Rate(interval)
    while not rospy.is_shutdown():
        data = queue.get()
        if data is None:
            continue

        # # 机械臂开始上料
        # msg = Order()
        # msg.name = data["name"]
        # msg.type = data["type"]
        # msg.id = data["id"]
        # msg.state = Order.STATE_WAITING
        # publisher.publish(msg)
        #
        # rate.sleep()
        name = data["name"]
        type = data["type"]
        id = data["id"]

        work_list.append({
            "id": id,
            "name": name,
            "type": type,
            "state": STATE_INITED
        })

        # 更新生产状态
        update_status_producting(id)

        # 操作流程
        # TODO: 有可能是送货
        item = find_item(STATE_INITED)
        item["state"] = STATE_IN_TRANSPORTED

        # 1. 机械臂上料
        feeding = start_aubo_feeding()
        if feeding:
            item = find_item(STATE_IN_TRANSPORTED)
            item["state"] = STATE_FEEDED
            # 成功
            start_assembly_line(4)
        else:
            # 失败 TODO
            rospy.loginfo("订单{}, 上料失败".format(id))


def start_assembly_line(index):
    client = rospy.ServiceProxy("/assembly/line_ctrl", AssemblyLineCtrl)
    client.wait_for_service()
    request = AssemblyLineCtrlRequest()
    request.line = index
    request.state = AssemblyLineCtrlRequest.STATE_START
    client.call(request)
    client.close()


def stop_assembly_line(index):
    client = rospy.ServiceProxy("/assembly/line_ctrl", AssemblyLineCtrl)
    client.wait_for_service()
    request = AssemblyLineCtrlRequest()
    request.line = index
    request.state = AssemblyLineCtrlRequest.STATE_STOP
    client.call(request)
    client.close()


def start_laser_mark(name, type, id):
    # 创建客户端
    client = actionlib.ActionClient("/laser/mark", LaserMarkAction)
    # 等待连接服务器
    client.wait_for_server()

    # 发送请求
    goal = LaserMarkGoal()
    goal.name = name
    goal.type = type
    goal.id = id
    goal_handle = client.send_goal(goal)
    result = client.wait_for_server(rospy.Duration(10))
    if result:
        state = goal_handle.get_terminal_state()
    return True


def ir_callback(msg):
    if not isinstance(msg, AssemblyIR): return

    if msg.ir_1:
        # 1. 停止传送带2号
        stop_assembly_line(2)
        # 激光打标机应该开始打标工作
        item = find_item(STATE_FEEDED)
        mark = start_laser_mark(name=item["name"], id=item["id"], type=item["type"])
        if mark:
            start_assembly_line(2)
            item["state"] = STATE_MARKED
        else:
            # TODO:
            rospy.loginfo("订单{}, 打标失败")

    if msg.ir_2:
        # 任务完成，机械臂开始将产品下架
        # 1. 停止传送带4号
        stop_assembly_line(4)
        # 2. 下架商品
        blanding = start_aubo_blanding()
        if blanding:
            item = find_item(STATE_MARKED)
            item["state"] = STATE_BLANDED
            start_assembly_line(4)

        else:
            # TODO:
            rospy.loginfo("订单{}, 下架失败")


if __name__ == '__main__':
    # 创建node
    node_name = "order_manager_node"
    rospy.init_node(node_name)

    host = rospy.get_param("~service_host", "192.168.1.100")
    port = rospy.get_param("~service_port", 3000)

    interval = rospy.get_param("~product_interval", 10)

    # 消息队列，用来存放订单的
    queue = Queue.Queue()

    # work list 工作列表，用来记录生产过程中的状态数据
    work_list = []

    publisher = rospy.Publisher("/order", Order, queue_size=1000)
    rospy.Subscriber("/assembly/ir_status", AssemblyIR, ir_callback)

    threading.Thread(target=do_work).start()

    order_list_url = "/order/list"
    order_list_body = 'status=2'

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
    queue.put(None)
