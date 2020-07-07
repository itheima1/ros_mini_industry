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
STATE_PRE_IN_TRANSPORTED = 1  # 准备运送输入
STATE_IN_TRANSPORTED = 2  # 运送输入完成
STATE_PRE_FEED = 3  # 准备上料
STATE_FEEDED = 4  # 上料完成
STATE_PRE_MARK = 5  # 准备打标
STATE_MARKED = 6  # 打标完成
STATE_PRE_BLAND = 7  # 准备下料
STATE_BLANDED = 8  # 下料完成
STATE_PRE_OUT_TRANSPORTED = 9  # 准备运送输出
STATE_OUT_TRANSPORTED = 10  # 运送输出完成


def reinit_order():
    order_update_url = "/order/check"
    conn = httplib.HTTPConnection(host, port)
    conn.request("POST", url=order_update_url,
                 headers={"Content-Type": "application/x-www-form-urlencoded"})
    conn.getresponse()
    conn.close()


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
    update_status(id, 3)


def update_status_completed(id):
    update_status(id, 4)


def update_status_error(id):
    update_status(5, id)


def wait_for_result(handle, timeout):
    if not isinstance(handle, ClientGoalHandle):
        return False

    timeout_time = rospy.Time().now() + timeout
    comm_state = handle.get_comm_state()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        comm_state = handle.get_comm_state()
        if comm_state == CommState.DONE:
            break

        if timeout_time < rospy.Time().now():
            break

        rate.sleep()
    return comm_state == CommState.DONE


def start_aubo_blanding():
    # 创建客户端
    client = actionlib.ActionClient("/aubo/ctrl", ArmWorkAction)
    # 等待连接服务器
    client.wait_for_server()

    # 发送请求
    goal = ArmWorkGoal()
    goal.type = 1
    goal_handle = client.send_goal(goal)
    result = wait_for_result(goal_handle, rospy.Duration(30))

    if result:
        terminal_state = goal_handle.get_terminal_state()
        get_result = goal_handle.get_result()
        return terminal_state == TerminalState.SUCCEEDED, get_result
    return False, None


def start_aubo_feeding():
    # 创建客户端
    client = actionlib.ActionClient("/aubo/ctrl", ArmWorkAction)
    # 等待连接服务器
    client.wait_for_server()

    # 发送请求
    goal = ArmWorkGoal()
    goal.type = 0
    goal_handle = client.send_goal(goal)
    result = wait_for_result(goal_handle, rospy.Duration(30))
    if result:
        terminal_state = goal_handle.get_terminal_state()
        get_result = goal_handle.get_result()
        return terminal_state == TerminalState.SUCCEEDED, get_result
    return False, None


def find_item(state):
    for item in work_list:
        if item["state"] == state:
            return item
    return None


def clear_item(id):
    for item in list(work_list):
        if item["id"] == id:
            work_list.remove(item)


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

        rospy.loginfo("############## New Order {} ####################".format(id))

        work_list.append({
            "id": id,
            "name": name,
            "type": type,
            "state": STATE_INITED
        })

        # 操作流程
        # TODO: 有可能是送货
        item = find_item(STATE_INITED)
        item["state"] = STATE_IN_TRANSPORTED

        # 1. 机械臂上料
        item["state"] = STATE_PRE_FEED
        feeding = start_aubo_feeding()
        if feeding[0]:
            item["state"] = STATE_FEEDED
            # 成功
            start_assembly_line(4)

            rospy.loginfo("######## 订单{}, 上料成功 ######## ".format(id))
        else:
            # 失败
            rospy.loginfo("######## 订单{}, 上料失败 ######## ".format(id))
            item["state"] = STATE_IN_TRANSPORTED
            if feeding[1] is not None:
                rospy.loginfo(feeding[1])
            # 移除工作队列
            clear_item(item["id"])


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
    result = wait_for_result(goal_handle, rospy.Duration(30))
    if result:
        terminal_state = goal_handle.get_terminal_state()
        get_result = goal_handle.get_result()
        return terminal_state == TerminalState.SUCCEEDED, get_result
    return False, None


def ir_callback(msg):
    if not isinstance(msg, AssemblyIR): return

    if msg.ir_1:
        item = find_item(STATE_FEEDED)
        rospy.loginfo("####### IR #### 1 #### True #######")
        if item is not None:
            # 更新生产状态
            update_status_producting(item["id"])
            # 1. 停止传送带2号
            stop_assembly_line(2)
            rospy.sleep(3)
            # 激光打标机应该开始打标工作
            item["state"] = STATE_PRE_MARK
            mark = start_laser_mark(name=item["name"], id=item["id"], type=item["type"])
            if mark[0]:
                start_assembly_line(2)
                item["state"] = STATE_MARKED
                start_assembly_line(2)

                rospy.loginfo("####### 订单{}, 打标成功 #######".format(item["id"]))
            else:
                item["state"] = STATE_FEEDED
                start_assembly_line(2)
                rospy.loginfo("####### 订单{}, 打标失败 #######".format(item["id"]))
                if mark[1] is not None:
                    rospy.loginfo(mark[1])

    if msg.ir_2:
        # 任务完成，机械臂开始将产品下架
        # 1. 停止传送带4号
        item = find_item(STATE_MARKED)
        if item is not None:
            rospy.loginfo("############# 停止4号机 ########")
            stop_assembly_line(4)
            rospy.sleep(3)
            # 2. 下架商品
            item["state"] = STATE_PRE_BLAND
            blanding = start_aubo_blanding()
            if blanding[0]:
                rospy.loginfo("####### 订单{}, 下架成功 #########".format(item["id"]))
                item["state"] = STATE_BLANDED
                start_assembly_line(4)
                rospy.loginfo("############# 打开4号机 ########")

                update_status_completed(item["id"])
                clear_item(item["id"])
            else:
                item["state"] = STATE_MARKED
                rospy.loginfo("####### 订单{}, 下架失败 #########".format(item["id"]))
                if blanding[1] is not None:
                    rospy.loginfo(blanding[1])


if __name__ == '__main__':
    # 创建node
    node_name = "order_manager_node"
    rospy.init_node(node_name)

    host = rospy.get_param("~service_host", "192.168.1.100")
    port = rospy.get_param("~service_port", 3000)

    interval = rospy.get_param("~product_interval", 10)

    reinit_order()

    # 消息队列，用来存放订单的
    queue = Queue.Queue()

    # work list 工作列表，用来记录生产过程中的状态数据
    work_list = []

    publisher = rospy.Publisher("/order", Order, queue_size=1000)
    rospy.Subscriber("/assembly/ir_state", AssemblyIR, ir_callback)

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
                item_id = item["id"]
                need_add = True
                for m in work_list:
                    if m["id"] == item_id:
                        need_add = False
                        break
                if need_add:
                    queue.put(item)

        except Exception as e:
            print e

        rate.sleep()
    queue.put(None)
