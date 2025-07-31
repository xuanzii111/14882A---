# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       MA                                                           #
# 	Created:      2024/12/11 19:17:53                                          #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

#17收
# Library imports
from vex import *
import math
brain = Brain()
controller1 = Controller()


m_in = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
m_in2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1)
m_out1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1)
m_out2 = Motor(Ports. PORT1, GearSetting.RATIO_6_1,True)
ml1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
ml2 = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
ml3 = Motor(Ports.PORT21, GearSetting.RATIO_6_1, False)
mr1 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)
mr2 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
mr3 = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
motor_left = MotorGroup(ml1, ml2, ml3)
motor_right = MotorGroup(mr1, mr2, mr3)
rota_left = Rotation(Ports.PORT12)  # 左侧旋转传感器
rota_right = Rotation(Ports.PORT14)  # 右侧旋转传感器
inertial = Inertial(Ports.PORT15)
optical = Optical(Ports.PORT13)
rota = Rotation(Ports.PORT16)

qidong_zhua = Pneumatics(brain.three_wire_port.g)
qidong_geban = Pneumatics(brain.three_wire_port.h)
rota_left_angle = 0 # 全局左侧编码器值
rota_right_angle = 0 # 全局右侧编码器值
theta = 0     # 全局方向
global_x = 0  # 全局x坐标
global_y = 0  # 全局y坐标
qi_zhua_state=True
take_in_state=True
take_out_state=True
filt_state = False
optical.set_light(LedStateType.ON)
optical.set_light_power(100)



# 函数

def odometry():
    global rota_left_angle, rota_right_angle, theta, global_x, global_y
    # 常量
    ODOMETRY_WHEEL_D = 2.75  # 定位轮的直径为2.75英寸
    inch_per_degree = math.pi * ODOMETRY_WHEEL_D / 360.0
    # 初始化上次的角度和定位轮角度
    prev_theta = math.radians(theta)
    prev_left_angle = rota_left.position()
    prev_right_angle = rota_right.position()

    cos_45 = math.cos(math.radians(45))
    sin_45 = math.sin(math.radians(45))

    while True:
        # 获取当前值        
        theta = inertial.rotation()  # 获取当前的角度
        curr_theta = math.radians(theta)  # 转换为弧度
        rota_left_angle = rota_left.position()
        rota_right_angle = rota_right.position()

        # 计算两次之间的变化量
        delta_left = rota_left_angle - prev_left_angle
        delta_right = rota_right_angle - prev_right_angle
        delta_theta = curr_theta - prev_theta

        # 计算定位轮移动距离
        dl = delta_left * inch_per_degree
        dr = delta_right * inch_per_degree

        # 坐标原点O2，在机器人本地坐标系下的坐标
        if abs(delta_theta) < 1e-5:
            # 如果没有旋转，即产生了平移
            xO = dl
            yO = dr
        else:
            # 如果有旋转，先计算旋转中心坐标
            xC = (dr / delta_theta)
            yC = -(dl / delta_theta)
            # 计算旋转后的坐标原点O2，在机器人本地坐标系下的坐标
            xO = xC + (xC * math.cos(-delta_theta) + yC * math.sin(-delta_theta))
            yO = yC + (yC * math.cos(-delta_theta) - xC * math.sin(-delta_theta))

        # # 因为定位轮并非与机器人xy同向，需要对xO和yO进行旋转
        xO_rotated = xO * cos_45 + yO * sin_45
        yO_rotated = yO * cos_45-xO * sin_45 

        # 更新全局坐标的坐标 theta, global_x, global_y

        global_x += xO_rotated * \
            math.cos(prev_theta) - yO_rotated * math.sin(prev_theta)
        global_y += xO_rotated * \
            math.sin(prev_theta) + yO_rotated * math.cos(prev_theta)
        
        
        # 更新上次的值
        prev_theta = curr_theta
        prev_left_angle = rota_left_angle
        prev_right_angle = rota_right_angle

        t2.sleep_for(20)  # 每20毫秒更新一次位置

def move(vl, vr, q=100):
    motor_left.set_max_torque(q, PERCENT)
    motor_right.set_max_torque(q, PERCENT)
    motor_left.set_velocity(vl, PERCENT)
    motor_right.set_velocity(vr, PERCENT)
    motor_left.spin(FORWARD)
    motor_right.spin(FORWARD)

def change_filt_state():
    global filt_state
    filt_state = not filt_state


def take_in():
    global take_in_state
    take_in_state=not take_in_state

    
    
    
    if take_in_state == True:
        m_in.set_max_torque(100, PERCENT)
        m_in.set_velocity(100, PERCENT)
        m_in2.set_max_torque(100, PERCENT)
        m_in2.set_velocity(100, PERCENT)
        m_in.spin(FORWARD)
        m_in2.spin(FORWARD)
    else:
        m_in.set_max_torque(100, PERCENT)
        m_in.set_velocity(0, PERCENT)
        m_in2.set_max_torque(100, PERCENT)
        m_in2.set_velocity(0, PERCENT)
        m_in.spin(FORWARD)
        m_in2.spin(FORWARD)
def take_out():
    
    global take_out_state
    take_out_state=not take_out_state

    
    
    
    if take_in_state == True:
        m_in.set_max_torque(100, PERCENT)
        m_in.set_velocity(100, PERCENT)
        m_in.spin(FORWARD)
        m_in2.set_max_torque(100, PERCENT)
        m_in2.set_velocity(-100, PERCENT)
        m_in2.spin(FORWARD)
    else:
        m_in.set_max_torque(100, PERCENT)
        m_in.set_velocity(0, PERCENT)
        m_in2.set_max_torque(100, PERCENT)
        m_in2.set_velocity(0, PERCENT)
        m_in.spin(FORWARD)
        m_in2.spin(FORWARD)
    
def takestop():
    m_in.set_max_torque(100, PERCENT)
    m_in.stop()
    m_in2.set_max_torque(100, PERCENT)
    m_in2.stop()

def tu_stop():
    m_out1.stop()
    m_out2.stop()

def tu_chu_shang():
    m_out1.set_max_torque(100, PERCENT)
    m_out1.set_velocity(100, PERCENT)
    m_out1.spin(FORWARD)
    m_out2.set_max_torque(100, PERCENT)
    m_out2.set_velocity(100, PERCENT)
    m_out2.spin(FORWARD)
def tu_chu_zhong():
    m_out1.set_max_torque(100, PERCENT)
    m_out1.set_velocity(-100, PERCENT)
    m_out1.spin(FORWARD)
    m_out2.set_max_torque(100, PERCENT)
    m_out2.set_velocity(100, PERCENT)
    m_out2.spin(FORWARD)
def tu_fan():
    m_out1.set_max_torque(100, PERCENT)
    m_out1.set_velocity(-100, PERCENT)
    m_out1.spin(FORWARD)
    m_out2.set_max_torque(100, PERCENT)
    m_out2.set_velocity(-100, PERCENT)
    m_out2.spin(FORWARD)

    


def takestop():
    m_out1.set_max_torque(100, PERCENT)
    m_out1.stop()
    m_out2.set_max_torque(100, PERCENT)
    m_out2.stop()


def qi_geban_fang():
    qidong_geban.open()

def qi_geban_tai():
    qidong_geban.close()

def qi_zhua_tai():
    global qi_zhua_state
    qi_zhua_state=not qi_zhua_state
    if qi_zhua_state:
        qidong_zhua.close()
    else:
        qidong_zhua.open()

    
def red_l():
    global autotime
   
    autotime = brain.timer.time()
    inertial.set_heading(0, DEGREES)
    
    pid_turn_to(343.8)
    
    change_filt_state()
    pid_move_to(10.5,343.8,kp = 3)
    
    
    wait(2000,MSEC)
    pid_turn_to(225)
    pid_move_to(-5.5,225)
    tu_chu_zhong()
def blue_l():
    global autotime
   
    autotime = brain.timer.time()
    inertial.set_heading(0, DEGREES)
    
    pid_turn_to(343.8)
    
    change_filt_state()
    pid_move_to(7,343.8,kp = 6,jump_time=0)
    pid_move_to(3.5,343.8,kp = 3,jump_time=0)
    
    wait(500,MSEC)
    pid_turn_to(225)
    tu_chu_zhong()
    pid_move_to(-5.5,225)
    wait(1000,MSEC)
    pid_turn_to(231)
    pid_move_to(18,231)
    pid_turn_to(180)
    pid_move_to(10,180,kp = 5,jump_time=0)
    move(-10,-10)
    wait(100,MSEC)
    move(20,20)
    wait(250,MSEC)
    tu_chu_shang()
    pid_move_to(-12,170)
    
def filt_block():
    # 0~30 red
    # 150~220 blue
    red_block = [0, 30]
    blue_block = [150, 220]

    block = red_block
    while True:
        if filt_state or controller1.buttonB.pressing():
            hue = optical.hue()
            print("Hue: ", hue)
            if controller1.buttonB.pressing():
                m_in.spin(FORWARD)
                m_in.set_velocity(-100, PERCENT)
            else:
                m_in.spin(FORWARD)
                m_in.set_velocity(100, PERCENT)


            if block[0] < hue < block[1]:
                m_in2.spin(FORWARD)
                m_in2.set_velocity(-100, PERCENT)
            else:
                m_in2.spin(FORWARD)
                m_in2.set_velocity(100, PERCENT)
        else:
            m_in.stop()
            m_in2.stop()

        wait(20)


  

    
  





def pid_turn_to(dir=0.0, kp=0.51,ki=0.015,kd=0,jump_time=0.1):
    """### pid 转向函数
    #### Arguments:
        dir : 转向的方向, 范围 0~359.99 °
        kp: 比例系数
        ki: 积分系数
        kd: 微分系数
        jump_time:校准时间

    #### Returns:
        None

    #### Examples:
        ### 转向 90° 方向
        `pid_turn_to(90)`
    """

    pre_error, int_error, dev_error, arrived_time = 0, 0, 0, 0 - 1
    v = 0
    arrived = False  # 判断到达变量
    start_time = brain.timer.value()
    pid_time = start_time
    while not arrived:
        error = dir - inertial.heading(DEGREES)
        error = (error + 180) % 360 - 180
        int_error += error * (brain.timer.value() - pid_time)
        pid_time = brain.timer.value()

        if int_error * error < 0.0:
            int_error = 0
        else:
            int_error = min(max(int_error, -100), 100)

        dev_error = error - pre_error
        pre_error = error

        # 时间判断，结束循环

        # 判断是否进入校准区间,校准区间为6°
        if abs(error) < 3:
            if arrived_time < 0:
                arrived_time = brain.timer.value()
            elif brain.timer.value() - arrived_time > jump_time:
                arrived = True
        else:
            if brain.timer.value() - start_time > 3:
                # 如果没有进入校准区间，而且转向总时间超过3秒，则退出循环
                arrived = True
            else:
                arrived_time = -1
                # 如果没有进入校准区间，不超过3秒，则继续转向
                arrived = False

        v = kp * error + ki * int_error + kd * dev_error
        v = min(max(v, -100), 100)
        v = 0 if arrived else v
        move(v, -v)  # 开始转向
        wait(20, MSEC)


def pid_move_to(
    dis, dir, kp=8, ki=0.1, kd=0.051, jump_time=0.2, t_kp= 0.7,MGear=36, WGear=48, WD=2.75
,zhundu=1.5):
    pre_error, int_error, dev_error, arrived_time = 0, 0, 0, -1
    arrived = False
    motor_left.set_position(0, DEGREES)
    motor_right.set_position(0, DEGREES)
    start_time = brain.timer.value()
    pid_time = start_time
    while not arrived:
        Turerror = dir - inertial.heading(DEGREES)
        Turerror = (Turerror + 180) % 360 - 180
        vtur = t_kp * Turerror

        pos = (motor_left.position() + motor_right.position()) / 2
        error = dis - pos / 360 * 3.14 * WD * MGear / WGear
        int_error += error * (brain.timer.value() - pid_time)
        if int_error * error < 0.0:
            int_error = 0
        pid_time = brain.timer.value()
        if int_error * error < 0:
            int_error = 0
        int_error =  min(max(int_error, -200), 200)
        dev_error = error - pre_error
        pre_error = error
        # 时间判断，结束循环

        # 判断是否进入校准区间,校准区间为6°
        if abs(error) < zhundu:
            if arrived_time < 0:
                arrived_time = brain.timer.value()
            elif brain.timer.value() - arrived_time > jump_time:
                arrived = True
        else:
            if brain.timer.value() - start_time > 1 + abs(dis) * 0.5:
                # 如果没有进入校准区间，而且转向总时间超过限定时间，则退出循环
                arrived = True
            else:
                arrived_time = -1
                # 如果没有进入校准区间，不超过限定时间，则继续转向
                arrived = False

        v = kp * error + ki * int_error + kd * dev_error
        v = min(max(v, -100), 100)
        v = 0 if arrived else v
        vtur = 0 if arrived else vtur
        move(v + vtur, v - vtur)
        wait(20, MSEC)

def is_take_in():
    while(m_in.torque()>5):
        wait(10,MSEC)

def blue_r():
    global autotime
   
    autotime = brain.timer.time()
    inertial.set_heading(0, DEGREES)
    
    pid_turn_to(16.2)
    
    change_filt_state()
    pid_move_to(8,16.2,kp = 7,jump_time=0)
    pid_move_to(4,16.2,kp = 4,jump_time=0)
    wait(300,MSEC)
    pid_turn_to(141,jump_time=0)
    qi_zhua_tai()
    pid_move_to(16,141,kp = 8,jump_time=0)
    pid_turn_to(175,jump_time=0)
    pid_move_to(10,175,kp = 6,jump_time=0)
    move(-10,-10)
    wait(100,MSEC)
    move(20,20)
    wait(250,MSEC)
    tu_chu_shang()
    pid_move_to(-12,190)
    
def red_r():
    # global autotime
   
    # autotime = brain.timer.time()
    # inertial.set_heading(0, DEGREES)
    
    # pid_turn_to(16.2)
    
    # change_filt_state()
    # pid_move_to(8,16.2,kp = 7,jump_time=0)
    # pid_move_to(5,16.2,kp = 4,jump_time=0)
    # wait(300,MSEC)
    # pid_turn_to(138,jump_time=0)
    # qi_zhua_tai()
    # pid_move_to(16,138,kp = 8,jump_time=0)
    # pid_turn_to(175,jump_time=0)
    # pid_move_to(12,175,kp = 6,jump_time=0)
    # move(-10,-10)
    # wait(100,MSEC)
    # move(20,20)
    # wait(250,MSEC)
    # tu_chu_shang()
    # pid_move_to(-12,190)
    global autotime
   
    autotime = brain.timer.time()
    inertial.set_heading(0, DEGREES)
    
    pid_turn_to(16.2)
    
    change_filt_state()
    pid_move_to(8,16.2,kp = 7,jump_time=0)
    pid_move_to(4,16.2,kp = 4,jump_time=0)
    wait(300,MSEC)
    pid_turn_to(141,jump_time=0)
    qi_zhua_tai()
    pid_move_to(16,141,kp = 8,jump_time=0)
    pid_turn_to(170,jump_time=0)
    pid_move_to(7,170,kp = 5,jump_time=0)
    move(-10,-10)
    wait(100,MSEC)
    move(20,20)
    wait(300,MSEC)
    tu_chu_shang()
    pid_move_to(-12,195)
def js_blue_r():
    global autotime
   
    autotime = brain.timer.time()
    inertial.set_heading(180, DEGREES)
    
    pid_turn_to(180-16.2)
    
    change_filt_state()
    pid_move_to(8,180-16.2,kp = 7,jump_time=0)
    pid_move_to(4,180-16.2,kp = 4,jump_time=0)
    wait(300,MSEC)
    pid_turn_to(180-141,jump_time=0)
    qi_zhua_tai()
    pid_move_to(16,180-141,kp = 8,jump_time=0)
    pid_turn_to(180-175,jump_time=0)
    pid_move_to(10,180-175,kp = 6,jump_time=0)
    move(-10,-10)
    wait(100,MSEC)
    move(20,20)
    wait(250,MSEC)
    tu_chu_shang()
    pid_move_to(-12,180-190)
    
    
    
    


# ---------------------------------------------------------------------------- #
#   以下是比赛函数                                                              #
#                                                                              #
# ---------------------------------------------------------------------------- #


def pre_auto():
    m_in.set_max_torque(100, PERCENT)
    m_in2.set_max_torque(100, PERCENT)
    m_out1.set_max_torque(100, PERCENT)
    m_out2.set_max_torque(100, PERCENT)
    optical.set_light(LedStateType.ON)
    optical.set_light_power(100)
    rota_left.reset_position()
    rota_right.reset_position()
    inertial.calibrate()
    wait(2500,MSEC)
    


   

def autonomous():
    #red_l()
    #blue_l()
    red_r()
    #blue_r()

def user_control():
    
    takestop()
   
    while True:
        if controller1.buttonDown.pressing():
            move(-50,-50)
        else:
            ax1 = controller1.axis1.position()
            ax1 = ax1 * abs(ax1) / 100   #100
            ax3 = controller1.axis3.position()
            move(ax3 + ax1, ax3 - ax1)
        
        
        
        wait(20, MSEC)



# create competition instance
comp = Competition(user_control, autonomous)
controller1.buttonR1.pressed(tu_chu_shang)
controller1.buttonR1.released(tu_stop)
controller1.buttonL1.pressed(tu_fan)
controller1.buttonL1.released(tu_fan)
controller1.buttonR2.pressed(tu_chu_zhong)
controller1.buttonR2.released(tu_stop)
controller1.buttonX.pressed(qi_zhua_tai)
# controller.buttonB.released(takestop)
controller1.buttonB.pressed(take_out)

controller1.buttonA.pressed(change_filt_state)
controller1.buttonY.pressed(qi_geban_fang)
controller1.buttonY.released(qi_geban_tai)
#controller.buttonUp.pressed(upto_h)
# controller.buttonY.pressed(sift_color)
#controller.buttonB.pressed(gaogua)
#optical.object_detected(sift_ring)


pre_auto()
t1 = Thread(filt_block)
t2 = Thread(odometry)

while True:
    brain.screen.print_at(global_x,x=10,y=20)
    brain.screen.print_at(global_y,x=110,y=20)
   
    wait(100, MSEC)
    
    
    
    # def red_r():
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(540 - 236, DEGREES)
#     wait(1000,MSEC)
#     m_up.set_position(0, DEGREES)
#     m_up.spin_to_position(500, DEGREES)
#     pid_move_to(8.5, 540 - 236, kp=5)
#     m_in.spin_for(FORWARD,90,DEGREES)
#     m_in.set_velocity(80, PERCENT)
#     m_up.spin_to_position(415, DEGREES)
#     wait(200, MSEC)
#     pid_move_to(-10, 540 - 236)
#     wait(200, MSEC)
#     m_up.spin_to_position(0, DEGREES, False)
#     qi_open()
#     pid_turn_to(540 - 250)
#     pid_move_to(-34, 540 - 240, 2)
#     qi_close()
#     take_in()
#     pid_turn_to(180 - 0)
#     pid_move_to(20, 180 - 0, kp=4)
#     wait(200, MSEC)
#     pid_move_to(2, 180 - 0, kp=4)

#     pid_turn_to(240)
#     m_up.spin_to_position(130, DEGREES, False)
#     pid_move_to(32, 240, kp=7 ,jump_time=0)
#     move(30, 30, 40)
#     wait(500, MSEC)
#     pid_move_to(-2, 225, jump_time=0)
#     wait(500, MSEC)
#     pid_turn_to(222, jump_time=0)
#     m_up.spin_to_position(550, DEGREES, False)
#     pid_move_to(-57, 225, jump_time=0)
#     m_in.stop()
#     move(-30, -30, 30)
#     autotime = brain.timer.time() - autotime


# def blue_l():
#               ############新车#############
#     #初始化
    
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(238, DEGREES)
#     #进联队环
#     pid_move_to(5,238,3)
#     upto_up()
#     pid_move_to(-5,238,5)
#     upto_down()
#     #夹第一个桩
#     qi_open()
#     pid_move_to(-32, 250, 2)
#     qi_close()
#     take_in()
#     #吸环
#     pid_turn_to(0)
#     pid_move_to(20,  0, kp=4)
#     wait(200, MSEC)
#     pid_move_to(2, 0, kp=4)
#     #吸加分角里的环
#     pid_turn_to(300)
#     pid_move_to(32, 300, kp=7 ,jump_time=0)
#     move(30, 30, 40)
#     wait(200, MSEC)
#     pid_move_to(-2, 540-225, jump_time=0)
#     wait(200, MSEC)
#     #撞杆
#     pid_turn_to(540-222, jump_time=0)
#     m_up.spin_to_position(550, DEGREES, False)
#     pid_move_to(-57, 540-225, jump_time=0)
#     m_in.stop()
#     move(-30, -30, 30)
#     autotime = brain.timer.time() - autotime
    
    
# def skill():
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(270, DEGREES)
#     take_in()
#     wait(1000,MSEC)
#     takestop()
#     pid_move_to(15,270,5)
#     pid_turn_to(0)
#     qi_open()
#     pid_move_to(-22,0,4)
#     pid_move_to(-3,0,3)
#     qi_close()
#     pid_turn_to(180)
#     wait(500,MSEC)
#     take_in
#     pid_move_to(30,180,3.5)
#     pid_move_to(2,180,3.5)
#     take_in()
#     wait(1500,MSEC)
#     pid_move_to(-12,70,4)
#     pid_move_to(23,70,3.5)
#     wait(500,MSEC)
#     #pid_turn_to(360-45)
#     pid_move_to(-5,313,5)
#     wait(500,MSEC)
#     move(-30,-30)
#     wait(2000,MSEC)
#     pid_move_to(2, 360 - 45)
#     wait(500, MSEC)
#     qi_open()
#     wait(500,MSEC)
#     pid_move_to(19, 360 - 45)
#     pid_turn_to(270)
#     move(-50, -50, 50)
#     wait(1500, MSEC)
#     move(0, 0)
#     wait(500, MSEC)
#     inertial.set_heading(270, DEGREES)
#     wait(500, MSEC)
#     pid_move_to(18, 270, 4)
#     pid_turn_to(180)
#     qi_open()
#     pid_move_to(-55, 180, 4)
#     pid_move_to(-7, 180, 4)
#     qi_close()
#     pid_turn_to(0)
#     pid_move_to(33, 0, 2)
#     wait(1500, MSEC)
#     pid_move_to(-15, 110, 2.7)
#     pid_move_to(22, 110, 3.5)
#     wait(1500, MSEC)
#     pid_turn_to(45 + 180)
#     move(-30, -30)
#     wait(2000, MSEC)
#     pid_move_to(2, 45 + 180)
#     qi_open()
#     wait(1000, MSEC)
#     pid_turn_to(45+180)
#     pid_move_to(55,42+180)
#     takestop()
#     m_up.spin_to_position(365, DEGREES)
    
#     move(100,100)
#     wait(500, MSEC)
#     move(0,0)
#     m_up.spin_to_position(0,DEGREES)
#     m_up.set_stopping(HOLD)


# def xin_blue_l():
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(296, DEGREES)#296
#     #pid_turn_to(180)
#     #wait(15000,MSEC)
    
#     m_up.set_position(0,DEGREES)
#     m_up.set_velocity(40, PERCENT)
#     m_up.spin(FORWARD)
#     m_up.set_timeout(800,MSEC)
#     m_up.spin_to_position(180,DEGREES)
#     pid_move_to(-12,303)
#     upto_up()
#     pid_turn_to(0)
#     qi_open()
#     upto_stop()
#     pid_move_to(-24,0,kp = 4,jump_time=0)
#     pid_move_to(-5,0,kp = 3)
#     qi_close()
#     pid_turn_to(90)
#     take_in()
#     pid_move_to(15,90,kp = 4)
#     pid_move_to(6,90,kp = 3)
#     pid_turn_to(21)
#     pid_move_to(36,21,kp = 5)
#     #pid_move_to(10,45,kp = 4)
#     move(100,100)
#     wait(600,MSEC)
#     move(0,0)
#     wait(300,MSEC)
#     move(100,100)
#     wait(300,MSEC)
#     move(0,0)
#     wait(300,MSEC)
#     # move(60,60)
#     # wait(450,MSEC)
#     pid_move_to(-2,45,kp = 4) 
#     pid_move_to(2,45,kp = 4)  
#     wait(1000,MSEC)
#     m_up.spin_to_position(17,DEGREES,False)
#     pid_move_to(-50,45,jump_time=0)
#     takestop()
#     move(-30,-30,40)
#     #pid_move_to(-13,303)
#     #pid_turn_to(270)
#     #qi_tai()
#     #pid_move_to(25,270)
#     #take_in()
#     #pid_move_to(5,270)
#     #takestop()
#     #qi_song()
#     #qi_open()
#     #pid_turn_to(55)
#     #pid_move_to(-20, 305)
#     #pid_move_to(-5, 305)
#     #qi_close()
#     #take_in()
#     #pid_turn_to(90)
#     #pid_move_to(25,90)
#     #pid_move_to(3,90)
#     #qi_close()
#     #pid_move_to(-3,45)
#     #pid_turn_to(90)
#     #take_in()
#     #wait(100,MSEC)
#     #pid_move_to(21, 90, 3.5)
#     #pid_move_to(2,90,2)

#     #pid_turn_to(0,jump_time=0.2)
#     #t_t = brain.timer.time()
#     #brain.timer.event(m_in.stop,1000)
#     #pid_move_to(27, 0, 3,jump_time=0)
#     #brain.screen.print_at("time", brain.timer.time()-t_t, x=1, y=110)
    
    
    
#     # pid_move_to(2,0,4,jump_time=0)
#     # t = brain.timer.time()
#     # while( True ):
#     #     if brain.timer.time()-t>500:
#     #         break
#     #     if m_in.torque()<0.45:
#     #         break
#     #     wait(5,MSEC)
    
#     # take_out()
#     # wait(200,MSEC)
#     # takestop()
#     #m_up.set_max_torque(100,PARTNER)
#     #m_up.set_velocity(100,PERCENT)
#     #m_up.spin_to_position(700,DEGREES,False)
#     #pid_turn_to(45)
#     #pid_move_to(18,45,3.5)
#     #m_in.set_max_torque(100,PERCENT)
#     #m_in.set_velocity(70,PERCENT)
    
#     #m_in.spin(FORWARD)
#     #wait(400,MSEC)
#     #m_in.stop()
#     #m_up.spin_to_position(140,DEGREES,False)
#     #pid_move_to(-19,45,4,jump_time=0)
#     #pid_turn_to(270+45,jump_time=0)
#     #take_in()
#     #pid_move_to(37, 270+45, 7,jump_time=0)
#     #move(30, 30, 40)
#     #wait(100, MSEC)
#     #pid_move_to(-2,270+45,3,jump_time=0)

# def xin_blue_r():
#      global autotime
#      autotime = brain.timer.time()
#      inertial.set_heading(0, DEGREES)#0
#      qi_open()
#      pid_move_to(-30,0,kp = 2.3,ki=0.5,jump_time = 0)
#      qi_close()
#      pid_turn_to(200)
#      take_in()
#      pid_move_to(15,222,kp = 3,ki=0.3,jump_time = 0.2)
#      #pid_move_to(5,220,kp = 3,jump_time = 0)
#      pid_turn_to(230,jump_time=0)
#      pid_move_to(17,270,kp = 3)
#      wait(700,MSEC)
#      pid_turn_to(3,jump_time = 0)
#      pid_move_to(14,3,kp = 5)
#      pid_turn_to(320)
#      pid_move_to(38,350,kp = 5)
#      pid_turn_to(315,jump_time=0)
#      move(100,100)
#      wait(400,MSEC)
#      move(0,0)
#      wait(700,MSEC)
#      pid_move_to(-2,315,kp = 3,jump_time = 0)
#      pid_move_to(3,315,kp = 3,jump_time = 0)
#      m_up.spin_to_position(45,DEGREES,False)
#      pid_move_to(-50,343,jump_time=0)
#      takestop()
#      move(-50,-50,40)
     
     
     
#     #  pid_move_to(-6,250,jump_time = 0)
#     #  pid_move_to(4,305)
#     #  pid_move_to(10,305,kp = 4)
#     #  wait(300,MSEC)
#     #  pid_turn_to(340,jump_time=0)
#     #  pid_move_to(44,300,kp=4)
#     #  move(100,100)
#     #  wait(600,MSEC)
#     #  pid_move_to(-2,300,kp = 3,jump_time = 0)
#     #  pid_move_to(3,300,kp = 3,jump_time = 0)
#     #  wait(1000,MSEC)
#     #  m_up.spin_to_position(45,DEGREES,False)
#     #  pid_move_to(-50,343,jump_time=0)
#     #  takestop()
#     #  move(-50,-50,40)

# def xin_red_l():
#     td = 530
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(180, DEGREES)#0
#     qi_open()
#     pid_move_to(-30,180,kp = 2.3,ki=0.5,jump_time = 0)
#     qi_close()
#     pid_turn_to(td-200)
#     take_in()
#     pid_move_to(15,td-222,kp = 4,ki=0.3,jump_time = 0)
#     #pid_move_to(5,220,kp = 3,jump_time = 0)
#     pid_turn_to(td-230,jump_time=0)
#     pid_move_to(17,td-270,kp = 3)
#     wait(700,MSEC)
#     pid_turn_to(td-3,jump_time = 0)
#     pid_move_to(14,td-3,kp = 5)
#     pid_turn_to(td-320)
#     pid_move_to(38,td-350,kp = 5)
#     pid_turn_to(td-315,jump_time=0)
#     pid_move_to(4,td-315,jump_time=0)

#     move(100,50)
#     wait(300,MSEC)
#     pid_move_to(4,td-315,jump_time=0)
#     move(-30,-30)
#     wait(100,MSEC)
#     move(100,100)
#     wait(300,MSEC)
#     move(-20,-20)
#     wait(400,MSEC)
#     move(0,0)
#     wait(900,MSEC)
#     # pid_move_to(-2,td-315,kp = 3,jump_time = 0)
#     # pid_move_to(3,td-315,kp = 3,jump_time = 0)
#     pid_turn_to(td-315,jump_time=0)
#     m_up.spin_to_position(45,DEGREES,False)
#     pid_move_to(-45,td-315+40,jump_time=0)
#     takestop()
#     move(-50,-50,40)


# def xin_red_r():
#     ty = 559
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(ty - 296, DEGREES)#296
#     m_up.set_position(0,DEGREES)
#     m_up.set_velocity(50, PERCENT)
#     m_up.spin(FORWARD)
#     m_up.set_timeout(800,MSEC)
#     m_up.spin_to_position(180,DEGREES)
#     pid_move_to(-16,ty - 303)
#     upto_up()
#     pid_turn_to(ty - 0)
#     qi_open()
#     upto_stop()
#     pid_move_to(-30,ty - 0,kp = 3)
#     #pid_move_to(-5,ty - 0,kp = 2,jump_time = 0)
#     qi_close()
#     pid_turn_to(ty - 92)
#     take_in()             
#     pid_move_to(15,ty - 92,kp = 4)
#     pid_move_to(6,ty - 92,kp = 3)
#     wait(500,MSEC)
#     pid_turn_to(ty - 21)
#     pid_move_to(33,ty - 23)
#     #pid_move_to(10,ty - 45,kp = 4)
#     move(100,100)
#     wait(600,MSEC)
#     move(0,0)
#     wait(450,MSEC)
#     move(100,100)
#     wait(600,MSEC)
#     move(0,0)
#     wait(450,MSEC)
#     pid_move_to(-3,ty - 45,kp = 3,jump_time = 0) 
#     pid_move_to(2,ty - 45,kp = 3,jump_time = 0)  
#     wait(1100,MSEC)
#     m_up.spin_to_position(17,DEGREES,False)
#     pid_move_to(-55,ty - 42,jump_time=0)
#     takestop()
#     move(-30,-30,40)

# def juesai_blue_r():
#      global autotime
#      autotime = brain.timer.time()
#      inertial.set_heading(0, DEGREES)#0
#      qi_open()
#      pid_move_to(-30,0,kp = 2.3,ki=0.5,jump_time = 0)
#      qi_close()
#      pid_turn_to(200)
#      take_in()
#      pid_move_to(15,222,kp = 6,ki=0.3,jump_time = 0.1)
#      wait(300,MSEC)
#      #pid_move_to(5,220,kp = 3,jump_time = 0)
#      pid_turn_to(242)
#      pid_move_to(14,260,kp = 4,jump_time = 0.1)
#      wait(700,MSEC)
#      pid_move_to(-8,250,jump_time = 0)
#      pid_move_to(15,315,kp = 4,jump_time = 0.1)
#      wait(300,MSEC)
#      pid_turn_to(10,jump_time=0)
#      pid_move_to(44,321,kp=5,jump_time=0.1)
#      move(100,100)
#      wait(600,MSEC)
#      pid_move_to(-2,310,kp = 3,jump_time = 0)
#      pid_move_to(3,310,kp = 3,jump_time = 0)
#      wait(600,MSEC)
#      pid_move_to(-16,360,jump_time=0)
#      pid_turn_to(90)
#      qi_tai()
#      pid_move_to(22,87,jump_time=0)
#      pid_move_to(5,87,kp = 4)
#      qi_song()
     
# def pid_tiao():
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(0, DEGREES)
#     pid_turn_to(90,kd=0.03,ki=0.013,kp=0.51)

# def skill():
#     huan = 540
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(180, DEGREES)
#     take_in()
#     wait(500,MSEC)
#     pid_move_to(9,180)
#     pid_turn_to(90)
#     qi_open()
#     pid_move_to(-25,90,kp = 3)
#     qi_close()
   
#     pid_turn_to(180)
#     pid_move_to(25,180,kp = 3)
#     pid_turn_to(180+23)
#     upto_m()
#     pid_move_to(50,180+23,kp = 4)
#     wait(600,MSEC)
#     takestop()
#     wait(100,MSEC)
#     take_in()
#     wait(500,MSEC)
#     takestop()
#     wait(100,MSEC)
#     take_in()
#     wait(500,MSEC)
#     takestop()
#     wait(100,MSEC)
#     take_in()
#     wait(500,MSEC)
#     takestop()
#     pid_move_to(-16,180+23)
#     pid_turn_to(180)
#     pid_move_to(-5.5,180)
#     pid_turn_to(270)
#     m_in2.set_max_torque(100, PERCENT)
#     m_in2.set_velocity(100, PERCENT)
#     m_in2.spin(FORWARD)
    
#     move(40,40)
#     wait(700,MSEC)
#     move(0,0)
#     m_in2.stop()
#     upto_down()
#     pid_turn_to(260,jump_time=0)
#     pid_turn_to(280,jump_time=0)
#     pid_turn_to(270)
    
#     take_in()
#     pid_move_to(-9.5,270)
#     upto_stop()
#     pid_turn_to(0)
#     pid_move_to(50,0,kp =1)
#     pid_move_to(-20,0)
#     pid_turn_to(270+55)
#     pid_move_to(14,270+55,kp = 2)
#     pid_turn_to(270+55-175)
#     pid_move_to(-27,270+55-175,kp = 3)
#     qi_open()
#     pid_move_to(8.5,270+55-175,kp = 2)
#     pid_turn_to(90)
#     move(-30,-30)
#     wait(500,MSEC)
#     move(0,0)
    
#     inertial.set_heading(90, DEGREES)
#     wait(500,MSEC)
#     pid_move_to(65,90)
#     pid_turn_to(270)
#     qi_open()
#     pid_move_to(-20,270,kp = 2)
#     qi_close()
#     pid_turn_to(180)
#     take_in()
#     pid_move_to(25,180,kp = 3)
#     pid_turn_to(180-28)
#     upto_m()
#     pid_move_to(50,180-28,kp = 4)
#     wait(600,MSEC)
#     takestop()
#     wait(100,MSEC)
#     take_in()
#     wait(500,MSEC)
#     takestop()
#     wait(100,MSEC)
#     take_in()
#     wait(500,MSEC)
#     takestop()
#     wait(100,MSEC)
#     take_in()
#     wait(500,MSEC)
#     takestop()
#     pid_move_to(-14,180-28)
#     pid_turn_to(180)
#     pid_move_to(-8,180)
#     pid_turn_to(90)
#     m_in2.set_max_torque(100, PERCENT)
#     m_in2.set_velocity(100, PERCENT)
#     m_in2.spin(FORWARD)
#     #pid_move_to(25,270,kp = 4)
#     move(40,40)
#     wait(700,MSEC)
#     move(0,0)
#     m_in2.stop()
#     upto_down()
#     pid_turn_to(100,jump_time=0)
#     pid_turn_to(80,jump_time=0)
#     pid_turn_to(90)
#     take_in()
#     pid_move_to(-10,90)
#     upto_stop()
#     pid_turn_to(0)
#     pid_move_to(50,0,kp =1)
#     pid_move_to(-20,0)
#     pid_turn_to(90-55)
#     pid_move_to(13,90-55,kp = 2)
#     pid_turn_to(90-55+175)
#     pid_move_to(-24,90-55+175,kp = 3)
#     qi_open()
#     pid_move_to(9,90-55+175,kp = 2)

# def js_red_l():
#      td = 530
#      global autotime
#      autotime = brain.timer.time()
#      inertial.set_heading(180, DEGREES)#0
#      qi_open()
#      pid_move_to(-30,180,kp = 3,ki=0.5,jump_time = 0)
#      qi_close()
#      pid_turn_to(td-200)
#      take_in()
#      pid_move_to(14,td-222,kp = 4,ki=0.3,jump_time = 0)
#      #pid_move_to(5,220,kp = 3,jump_time = 0)
#      #pid_turn_to(td-230,jump_time=0)
#      pid_move_to(16,td-270,kp = 3,jump_time=0)
#      wait(300,MSEC)
#      pid_turn_to(td-3,jump_time = 0)
#      pid_move_to(14,td-3,kp = 5)
#      pid_turn_to(td-320)
#      pid_move_to(38,td-350,kp = 5)
#      pid_turn_to(td-315,jump_time=0)
#      pid_move_to(4,td-315,jump_time=0)

#      move(100,50)
#      wait(200,MSEC)
#      pid_move_to(4,td-315,jump_time=0)
#      move(-30,-30)
#      wait(100,MSEC)
#      move(100,100)
#      wait(100,MSEC)
#      move(0,0)
#      wait(250,MSEC)
#      pid_move_to(-16,td- 309,jump_time=0)
#      pid_turn_to(td-75)
#      qi_tai()
#      pid_move_to(29,td-73,jump_time=0)
#      pid_move_to(8,td-70,kp=2)
    
    
# def js_blue_r():
#     td = 562
#     global autotime
#     autotime = brain.timer.time()
#     inertial.set_heading(263, DEGREES)#296
#     m_up.set_position(0,DEGREES)
#     m_up.set_velocity(50, PERCENT)
#     m_up.spin(FORWARD)
#     m_up.set_timeout(800,MSEC)
#     m_up.spin_to_position(180,DEGREES)
#     pid_move_to(-16,256)
#     upto_up()
#     pid_turn_to(559)
#     qi_open()
#     upto_stop()
#     pid_move_to(-30,559,kp = 3)
#     qi_close()
    
#     pid_turn_to(td+200)
#     take_in()
#     pid_move_to(12,td + 222,kp = 4,ki=0.3,jump_time = 0)
#      #pid_move_to(5,220,kp = 3,jump_time = 0)
#      #pid_turn_to(td-230,jump_time=0)
#     pid_move_to(19,td+270,kp = 3,jump_time=0)
#     wait(300,MSEC)
#     pid_turn_to(td-6,jump_time = 0)
#     pid_move_to(14,td-6,kp = 5)
#     pid_turn_to(td+320)
#     pid_move_to(40,td+355,kp = 5)
#     pid_turn_to(td+315,jump_time=0)
#     pid_move_to(4,td+315,jump_time=0)

#     move(50,100)
#     wait(200,MSEC)
#     move(0,0)
#     wait(100,MSEC)
#     move(100,100)
#     wait(200,MSEC)
#     # pid_move_to(4,td+315,jump_time=0)
#     move(-30,-30)

#     # wait(100,MSEC)
#     # move(0,0)
#     # wait(250,MSEC)
    
    
    
    
    
    
    
    
    
    
#     # pid_turn_to(180)
#     # pid_move_to(15,180,kp = 3)
#     # pid_turn_to(270)
#     # pid_move_to(17,270,kp = 3)
#     # pid_turn_to(180)
#     # upto_m()
#     # pid_move_to(34,180,kp = 3)
#     # pid_turn_to(335)
    
    
    
    
    
    
    
    
    
    
     
    