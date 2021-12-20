from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
from multiprocessing import Process
import numpy as np
import time

def main(client_dashboard, client_feedback):
    # Remove alarm
    client_dashboard.ClearError()
    time.sleep(0.5)
    # Description The upper function was enabled successfully
    client_dashboard.EnableRobot()
    time.sleep(0.5)
    # Select user and Tool coordinate system 0
    client_dashboard.User(0)
    client_dashboard.Tool(0)
    # Call the JointMovJ directive
    client_feedback.JointMovJ(0,50,0,0,0,0)
    time.sleep(5)
    client_feedback.JointMovJ(0,30,0,0,0,0)
    time.sleep(5)
    
    gPose = client_dashboard.GetPose()
    gPose = list(map(float, gPose[1:len(gPose)-1].split(',')))
    print("\033[0;37;45mGetPose: \033[0m", gPose[0], gPose[1], gPose[2], gPose[3], gPose[4], gPose[5])#[1:21])
    
    client_dashboard.RobotMode()
    time.sleep(.5)
    
    client_feedback.MovJ(CartP[0], CartP[1], CartP[2], CartP[3], CartP[4]+20, CartP[5])
    time.sleep(1.5)
    client_feedback.MovJ(CartP[0], CartP[1], CartP[2], CartP[3], CartP[4], CartP[5])
    time.sleep(1.5)

    print('!!!!!!END!!!!!!')

# The feedback information about port 30003 is displayed
def data_feedback(client_rt_feedback):
    tick = 0
    while True:
        # time.sleep(0.05)
        all = client_rt_feedback.socket_feedback.recv(10240)
        tickNow = time.time()
        print(tickNow - tick)
        tick = tickNow
        # print(len(all))
        data = all[0:1440]
        # print("all: ", bytes.decode(all,'utf-8'))  #
        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':
            print('len', np.around(a['len']))
            print('\033[0;42;45mrobot_mode\033[0m', a['robot_mode'])
            # controller_timer
            print('controller_timer', a['controller_timer'])
            # run_time
            print('run_time', a['run_time'])
            print('tool_vector_actual', np.around(a['tool_vector_actual'], decimals=4))
            print('q_actual', np.around(a['q_actual'], decimals=4))
            # Masterboard: Robot current
            print('i_robot', np.around(a['i_robot'], decimals=4))
            # Norm of Cartesian linear momentum
            print('linear_momentum_norm', np.around(a['linear_momentum_norm'], decimals=4))
            # Target joint moments (torques)
            print('m_target', np.around(a['m_target'], decimals=4))
            # Actual joint currents
            print('\033[0;42;45mi_actual\033[0m', np.around(a['i_actual'], decimals=4))
            # Actual joint moments (torques)
            print('m_actual', np.around(a['m_actual'], decimals=4)) # ~0.1Nm bias/noise
            print('motor_temperatures', np.around(a['motor_temperatures'], decimals=4))
            print('m_actual', np.var(a['m_actual']))

# Enable threads on ports 29999, 30003 and 30004
if __name__ == '__main__':
    client_dashboard = dobot_api_dashboard('192.168.5.1', 29999)    # 192.168.5.1/.1.6
    client_feedback = dobot_api_feedback('192.168.5.1', 30003)  # 192.168.5.1/.1.6
    client_rt_feedback = dobot_api_rt_feedback('192.168.5.1', 30004)  # 192.168.5.1/.1.6
    p1 = Process(target=main, args=(client_dashboard, client_feedback))
    p1.start()
    p2 = Process(target=data_feedback, args=(client_rt_feedback, )) #client_feedback, ))
    p2.daemon =True
    p2.start()
    p1.join()
    client_dashboard.close()
    client_feedback.close()
