import Packet as Pkt
import Global_Par as Gp
import Routing_table as RT
import State_action_table as SAT
import traffic as rntf
import random
import Virtual_agents as VAs
import math

# 计算两节点之间的距离
def nndis(ax, ay, bx, by):
    temp_x = ax - bx
    temp_y = ay - by
    temp_x = temp_x * temp_x
    temp_y = temp_y * temp_y
    result = math.sqrt(temp_x+temp_y)
    return result

class SDVNController:
    def __init__(self, node_num, intersection):
        self.hello_list = []  # hello请求列表
        self.flow_request_list = []  # 路由请求列表
        self.flow_report_list = [] # 路由回复列表
        self.intersection = intersection # 所有路口 {it_0:[x,y], it_1:[x,y], ..., it_n:[x,y]}
        self.node_info_dict = {i: [] for i in range(node_num)}  # 所有节点信息
        self.all_node_neighbor = {i: [] for i in range(node_num)} # 所有节点的邻居
        self.it_cover = {i: -1 for i in range(node_num)} # 所有节点所属路口 {node:intersection, ...}
        self.routing_table = RT.Routing_Table() # 路由表实例
        self.state_action_table = SAT.State_Action_Table() # State Action 表实例
        self.virtual_agents = [] # 虚拟智能体
        self.virtual_agents_num = 4 # 虚拟智能体数
        self.road_veh_num = {} # 道路上车辆数 {jun1: {junc1: x,...},...}
        self.road_veh_num_ow = {} # 道路上单向行驶车辆数 {jun1: {junc1: x,...},...}
        self.junc_veh = {}  # 单个路口内车辆列表 {junc: [veh1,...],...}
        self.road_veh = {}  # 道路上车辆列表 {junc: {junc: [],...},...}

    # 统计每个车辆的邻居
    def cal_neib(self):
        for veh, info in self.node_info_dict.items():
            c_x = info[0][0]
            c_y = info[0][1]
            for nveh, ninfo in self.node_info_dict.items():
                if veh == nveh:
                    continue
                n_x = ninfo[0][0]
                n_y = ninfo[0][1]
                d_cn = nndis(c_x, c_y, n_x, n_y)
                if d_cn <= Gp.com_dis:
                    self.all_node_neighbor[veh].append(nveh)
        return

    # 根据hello列表中的条目更新控制器中的节点信息
    def predict_position(self):
        # 提取列表中的每一个hello包，将（id，地理位置，速度，加速度，当前剩余缓存）存储到node_info_dict字典中。
        # 记录并更新当前全部节点的信息
        for value in self.hello_list:
            self.node_info_dict[value.node_id] = [value.position, value.current_cache]
        self.hello_list.clear() # 清空hello列表，为下一时刻腾地方
        self.cal_neib() # 邻居
        return

    # 通知节点所属区域
    def send_area_info(self, node_list):
        # 向每个节点发送流包，告诉他们所属的交叉口
        for node in node_list:
            # 节点所属区域id 是一个列表[]
            area = self.it_cover[node.node_id]
            # 生成路由回复数据包
            flow_notify = Pkt.FlowNotify(area)
            node.receive_notify(flow_notify)
        # 时延处理
        return

    # 选择路由表
    def select_policy(self, node_id, des_id, pkt_type):
        Gp.tag = 4
        bas_path = self.calculate_area_path(node_id, des_id)
        # 计算BP路径负载情况
        veh_num = 0
        if len(bas_path) > 1:
            for i in range(len(bas_path)-1):
                veh_num += self.road_veh_num[bas_path[i]][bas_path[i+1]]
        else:
            veh_num = len(self.junc_veh[bas_path[0]])
        l = 0
        for j in range(veh_num):
            l += random.random()
        load = min(1, l / veh_num / random.uniform(0,3))
        # 选择合适的策略
        if load >= 0 and load < 0.3:
            L = 'A'
        elif load >= 0.3 and load <0.7:
            L = 'B'
        else:
            L = 'C'
        priority = self.state_action_table.SAtable[pkt_type][L]
        a = random.random()
        if a > Gp.prob:
            Gp.tag = priority.index(max(priority)) + 1
        else:
            Gp.tag = random.choice([1,2,3])
        return L

    # # 选出具有最大Q值的下一跳区域
    # candidates = Gp.q_table.table[des_area][current_area]
    # next = candidates[candidates == max(candidates)].index
    # next_area = random.choice(next)

    # 查表计算区域路径
    def calculate_area_path(self, node_id, des_id):
        if Gp.tag == 1:
            table = self.routing_table.table_HRF
        elif Gp.tag == 2:
            table = self.routing_table.table_LDF
        elif Gp.tag == 3:
            table = self.routing_table.table_LBF
        else:
            table = self.routing_table.table_BP
        area_path = []
        node_area = self.it_cover[node_id][0]
        des_area = self.it_cover[des_id][0]
        print("source area: %d  target area: %d " % (node_area, des_area))
        current_area = node_area
        area_path.append(current_area) # 把当前数据包所在的区域添加到路径中
        while current_area != des_area:
            #print("current area: ", current_area)
            # 如果目的区域为当前区域的邻居，直接选择其为下一跳
            if des_area in Gp.adjacents_comb[current_area]:
                area_path.append(des_area)
                break
            # 否则遍历Q表，选择具有最大Q值的邻居作为下一跳
            candidates = table[des_area][current_area]
            next = candidates[candidates == max(candidates)].index
            next_area = random.choice(next)
            # print(table[des_area][current_area])
            # print(next_area)
            if next_area not in area_path:
                area_path.append(next_area)
                current_area = next_area
            else:
                print("loop in area selection")
                Gp.loop_fail_time += 1
                area_path = []
                return area_path
        # 截取
        i = 0
        for area in area_path:
            if node_id in self.junc_veh[area] and area != node_area:
                i = area_path.index(area)
        area_path = area_path[i:]
        # 打印
        print("area path: ", area_path)
        return area_path

    # 发送路由回复
    # @staticmethod
    def send_reply(self, requester_id, area_path, node_list, L):
        # 生成路由回复数据包
        flow_reply = Pkt.FlowReply(area_path, Gp.tag, L)
        # 向请求节点发送流包，告诉它路由路径
        for node in node_list:
            if node.node_id == requester_id:
                node.receive_flow(flow_reply)
        # 时延处理
        return

    # 处理请求表中的每个请求，计算路由，发送回复
    def resolve_request(self, node_list):
        # 遍历路由请求列表
        for request in self.flow_request_list: # flow_request_list由Node中产生路由请求的generate_request()生成
            # 选择路由表
            L = self.select_policy(request.node_id, request.des_id, request.pkt_type)
            # 查表计算路径
            area_path = self.calculate_area_path(request.node_id, request.des_id)
            # 向发送请求节点回复
            self.send_reply(request.node_id, area_path, node_list, L)
        self.flow_request_list.clear() # 清空路由请求列表，下回还得使
        return

    # 计算两个路口间路由跳数，时延，开销
    # 输出为三种reward
    def get_metrics(self, area_path, veh_path, total_delay):
        ad_list = []
        hc_list = []
        rc_list = []
        if len(area_path) <= 2:
            return [], [], []
        # 初始化字典，记录道路上的中间车辆
        road_relay = {}
        for i in range(0, len(area_path)-1):
            c = area_path[i]
            road_relay[c] = []
        # 记录道路上的中间车辆
        j = 0 # 计数
        i = 0
        while 1:
            if i > len(veh_path) - 1:
                break
            veh = veh_path[i]
            # print(veh, self.it_cover[veh])
            veh_list = self.road_veh[area_path[j]][area_path[j+1]]
            # print(j)
            if veh in veh_list:
                road_relay[area_path[j]].append(veh)
                i = i + 1
                # print("on road")
            else:
                if area_path[j+1] in self.it_cover[veh]:
                    road_relay[area_path[j]].append(veh)
                    i = i + 1
                else:
                    if j == 0:
                        if area_path[j] in self.it_cover[veh]:
                            road_relay[area_path[j]].append(veh)
                            i = i + 1
                        else:
                            if j < len(area_path) - 2:
                                j = j + 1
                            else:
                                print("??")
                                exit(0)
                    else:
                        if j < len(area_path) - 2:
                            j = j + 1
                        else:
                            print("?")
                            exit(0)
        # print(area_path)
        # print(veh_path)
        # print(road_relay)
        # print("----------")
        # 计算每一条路上的三个特征奖励
        # hc
        for jun, vehs in road_relay.items():
            next_ = area_path[area_path.index(jun) + 1]
            dis = Gp.junction_dis[jun][next_]
            for adj in Gp.adjacents_comb[next_]:
                if adj != jun:
                    dis += (Gp.junction_dis[next_][adj] / 2)
            h = - ((Gp.com_dis * len(vehs)) / dis)
            # hc_list[jun] = math.exp(h)
            hc_list.append(math.exp(h))
        # ad
        for jun, vehs in road_relay.items():
            delay_= 0
            if len(vehs) == 1:
                delay_ += 0
            else:
                for i in range(len(vehs)-1):
                    c = vehs[i]
                    n = vehs[i+1]
                    dis = nndis(self.node_info_dict[c][0][0], self.node_info_dict[c][0][1], self.node_info_dict[n][0][0], self.node_info_dict[n][0][1])
                    delay_ += dis * (0.005 / 1000) + len(vehs) * 0.224
            d = (1 - (delay_ / total_delay)) * max(0.1, 1 - (delay_ / 11))
            # ad_list[jun] = d
            ad_list.append(d)
        # rc
        for jun, vehs in road_relay.items():
            next_ = area_path[area_path.index(jun) + 1]
            # veh_num = len(self.road_veh[jun][next_]) # 路上总车数
            hello_num = 0  # 探测包数
            flow_num = len(vehs) # 流包数
            jun_veh_l = self.road_veh[jun][next_]
            for v in self.junc_veh[next_]:
                if v not in jun_veh_l:
                    jun_veh_l.append(v)
            hello_num_total = 0  # 探测包总数
            flow_num_total = len(jun_veh_l) # 流包总数
            for node in jun_veh_l:#self.road_veh[jun][next_]:
                hello_num_total += len(self.all_node_neighbor[node])
            for node1 in vehs:
                hello_num += len(self.all_node_neighbor[node1])
            rc = hello_num + flow_num
            cc = max(1, hello_num_total + flow_num_total)
            r = (1 + (rc/cc)) ** (-1)
            # rc_list[jun] = r
            rc_list.append(r)
        return ad_list, hc_list, rc_list

    # 根据路由结束后的路由回复，解析，更新路由表
    def resolve_report(self):
        for report in self.flow_report_list:
            print(report.trans_type)
            ad_list, hc_list, rc_list = self.get_metrics(report.area_path, report.veh_path, report.ad)
            self.state_action_table.update(report.area_path, report.trans_type, report.L, report.type, report.ad, report.hc, report.rc, ad_list, hc_list, rc_list)
        self.flow_report_list = []
        return

    # 路由表融合
    # 先预处理，然后通过两种融合方式输出新的路由表
    def table_fusion(self):
        self.routing_table.preprocessing()
        self.routing_table.fusion_weight()
        self.routing_table.fusion_fuzzy()
        return

    # --------------------------------------------------------------------------------------- #
    #                                       路网分析
    # --------------------------------------------------------------------------------------- #
    def analyze(self):
        # 计算得到道路-车辆，路口-车辆 分布情况
        veh_detail, node_area = rntf.intra_vehicles_num(self.node_info_dict, Gp.it_pos, Gp.adjacents_comb)
        veh_num, veh_, veh_num_ow = rntf.inter_vehicles_num(self.node_info_dict, Gp.it_pos, Gp.adjacents_comb)
        # 记录到控制器中
        self.junc_veh = veh_detail # 区域所包含车辆 it:[node,...],...
        self.it_cover = node_area # 车辆所属路口 node:[it,...],...
        self.road_veh_num = veh_num # 道路上车辆数 {jun1: {junc1: x,...},...}
        self.road_veh = veh_ # 道路上的车辆信息 {junc: {junc: [],...},...}
        self.road_veh_num_ow = veh_num_ow # 道路上单向车辆数 {jun1: {junc1: x, ...}, ...}
        return

    # --------------------------------------------------------------------------------------- #
    #                                       虚拟智能体并行训练
    # --------------------------------------------------------------------------------------- #
    # 实例化虚拟智能体对象
    def instantiate_virtual_agent(self):
        for i in range(0, self.virtual_agents_num):
            self.virtual_agents.append(VAs.Virtual_agent(i))
        return

    # 虚拟智能体学习路由策略
    def virtual_training(self):
        for agent in self.virtual_agents:
            agent.learning()
        return


















