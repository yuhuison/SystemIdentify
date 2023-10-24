import numpy as np
from scipy.optimize import minimize

class StepData:
    def __init__(self, speed, time, control):
        self.speed = float(speed)
        self.time = float(time)
        self.control = float(control)

def read_file(filename):
    data_list = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines[1:]:  # 跳过标题行
            values = line.strip().split('\t')
            if len(values) == 3:
                step = StepData(values[2], values[0], values[1])
                data_list.append(step)
    return data_list

# 假设你的数据存储在step_data_list中
step_data_list = read_file(r"C:\Users\LENOVO\Downloads\ident_data.txt")


def fit_params(step_data_list):
    # 提取输入和输出数据
    inputs = [data.control for data in step_data_list]
    outputs = [data.speed for data in step_data_list]

    # 定义误差函数（损失函数）
    def error_function(params):
        # 提取PID参数
        Kp, Ki, Kd = params

        # 初始化误差
        error = 0.0

        # 初始化上一次的误差和积分项
        prev_error = 0.0
        integral = 0.0

        # 循环计算每个步骤的误差
        for i in range(len(inputs)):
            # 计算误差
            error = outputs[i] - inputs[i]

            
            # 计算积分项
            integral += error

            # 计算控制信号
            control_signal = Kp * error + Ki * integral + Kd * (error - prev_error)

            # 更新上一次的误差
            prev_error = error

            # 累加误差
            error += abs(control_signal - inputs[i])
        return error

    # 初始参数猜测
    initial_guess = [1.0, 0.5, 0.5]

    # 使用优化算法最小化误差函数
    result = minimize(error_function, initial_guess)

    # 提取最优参数
    best_params = result.x
    return best_params

def simulate_pid_control(input_sequence, best_params,init_value):
    Kp = best_params[0]
    Ki = best_params[1]
    Kd = best_params[2]
    output_sequence = []
    output_sequence.append(init_value)
    prev_error = 0
    integral = 0.0
    for index,input_value in enumerate(input_sequence[0:100]):
        error =  input_value - output_sequence[index]
        integral += error
        output = Kp * error + Ki * integral + Kd * (error - prev_error)
        
        print(error,integral,prev_error,output,input_value)
        # 保存输出值
        output_sequence.append(output)

        # 更新上一个误差值
        prev_error = error

    return output_sequence


parms = fit_params(step_data_list)
# 打印最优参数
print("Best PID parameters:", parms)
print(simulate_pid_control([d.control for d in step_data_list],parms,0))