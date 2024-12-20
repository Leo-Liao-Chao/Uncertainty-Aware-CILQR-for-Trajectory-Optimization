#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

import rosbag
import rospy
import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from vehiclepub.msg import Experiment

# def read_experiment_bag(bag_file):
#     data = {
#         "Start Time": [],
#         "Start Position": [],
#         "Planning Time": [],
#         "X": [],
#         "U": []
#     }

#     # 打开bag文件
#     bag = rosbag.Bag(bag_file, 'r')

#     try:
#         # 读取特定话题的数据
#         for topic, msg, t in bag.read_messages(topics=['/experiment']):
#             data["Start Time"].append(msg.start_time.to_sec())
#             data["Start Position"].append(msg.start_pos)
#             data["Planning Time"].append(msg.planning_time)
#             data["X"].append(msg.X)
#             data["U"].append(msg.U)
            
#     finally:
#         bag.close()
    
#     # 将数据转换为DataFrame
#     df = pd.DataFrame(data)

#     # 返回DataFrame
#     return df
def plot_positions_with_obstacles(df, obstacles, start_pos, end_pos):
    # 提取所有 x 和 y 坐标
    x = df["Start Position"].apply(lambda pos: pos[0])  # 提取 x 坐标
    y = df["Start Position"].apply(lambda pos: pos[1])  # 提取 y 坐标
    
    # 绘制散点图
    plt.figure(figsize=(10, 6))
    plt.scatter(x, y, c='blue', marker='o', label='Start Positions')
    
    # 绘制障碍物
    for obs in obstacles:
        obs_x, obs_y, theta, length, width = obs
        rect = Rectangle((obs_x - length / 2, obs_y - width / 2), length, width, 
                         angle=np.degrees(theta), fill=True, color='red', alpha=0.5)
        plt.gca().add_patch(rect)
    
    # 设置图表的显示范围
    plt.xlim(start_pos[0], end_pos[0])
    plt.ylim(start_pos[1], end_pos[1])


    plt.gca().set_aspect('equal', adjustable='box')
    
    plt.title("Scatter Plot of Start Positions with Obstacles")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)
    plt.show()


def data_process(df, start_pos, end_pos, planning_time_threshold=0.00):
    # 将 start_pos 和 end_pos 转换为 numpy 数组，以便进行逐元素比较
    start_pos = np.array(start_pos[:2])  # 只取前两个元素 (x, y)
    end_pos = np.array(end_pos[:2])      # 只取前两个元素 (x, y)
    
    # 选择满足条件的行
    x = df["Start Position"].apply(lambda pos: pos[0])  # 提取 x 坐标
    y = df["Start Position"].apply(lambda pos: pos[1])  # 提取 y 坐标
    
    # 计算矩形边界
    xmin, ymin = np.min([start_pos, end_pos], axis=0)
    xmax, ymax = np.max([start_pos, end_pos], axis=0)
    
    # 筛选在矩形区域内的点
    position_mask = (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
    
    # 筛选 Planning Time 大于 threshold 的点
    planning_time_mask = df["Planning Time"] > planning_time_threshold
    
    # 结合两个筛选条件
    combined_mask = position_mask & planning_time_mask
    
    # 返回筛选后的 DataFrame
    return df[combined_mask]

def calculate_distance(pos, obstacle):
    """
    计算位置点与矩形障碍物的最短距离（不考虑旋转）。
    pos: [x, y] 位置点
    obstacle: [x, y, theta, length, width] 矩形障碍物参数
    """
    # 提取位置点和障碍物参数
    px, py = pos[:2]
    ox, oy, otheta, olength, owidth = obstacle
    

    
    # 计算位置点到障碍物边界的最短距离
    dx = ox-px
    dy = oy-py
    
    distance = np.sqrt(dx**2 + dy**2)
    
    return distance

def compute_jerks(pos, dt):
    """
    计算二维轨迹的jerks（加加速度）。
    
    参数:
    pos: 轨迹的坐标列表，形状为 (2, n)，包含 (x, y)
    dt: 时间间隔
    
    返回:
    jerks: jerks的列表
    """
    x, y = pos[0], pos[1]
    
    # 检查轨迹点数量是否足够
    if len(x) < 3 or len(y) < 3:
        print("轨迹点数量不足，无法计算jerks")
        return None
    
    # 计算速度
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)
    
    # 计算加速度
    ax = np.gradient(vx, dt)
    ay = np.gradient(vy, dt)
    
    # 计算jerks（加加速度）
    jx = np.gradient(ax, dt)
    jy = np.gradient(ay, dt)
    
    # 计算jerks的大小
    jerks = np.sqrt(jx**2 + jy**2)
    
    return jerks


def compute_curvature(trajectory):
    """
    计算二维轨迹的曲率
    trajectory: numpy array of shape (2, n), 其中第1维是x和y坐标，n是点的数量
    """
    if trajectory.shape[1] < 3:
        raise ValueError("轨迹数据点数太少，无法计算曲率。")
    
    x = trajectory[0, :]
    y = trajectory[1, :]
    
    # 计算一阶导数（速度）
    dx = np.gradient(x)
    dy = np.gradient(y)
    
    # 计算二阶导数（加速度）
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # 计算曲率公式中的分子和分母
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = (dx ** 2 + dy ** 2) ** 1.5
    
    # 避免除以零的情况
    with np.errstate(divide='ignore', invalid='ignore'):
        curvature = np.true_divide(numerator, denominator)
        curvature[denominator == 0] = 0  # 在速度为零的情况下，曲率定义为0
    
    return curvature



def data_analysis(data, obstacles):
    # 获取 Planning Time 列
    planning_times = data["Planning Time"]
    
    # 计算最小值、最大值、平均值和方差
    min_time = planning_times.min()
    max_time = planning_times.max()
    mean_time = planning_times.mean()
    variance_time = np.var(planning_times, ddof=0)  # ddof=0 表示总体方差
    
    # 计算每个位置到所有障碍物的最小距离
    distances = []
    for pos in data["Start Position"]:
        min_distance = float('inf')
        for obstacle in obstacles:
            distance = calculate_distance(pos[:2], obstacle)
            if distance < min_distance:
                min_distance = distance
        distances.append(min_distance)
    
    # 将距离添加到DataFrame中
    data["Min Distance to Obstacle"] = distances
    
    # 对距离进行分析
    min_distance = np.min(distances)
    max_distance = np.max(distances)
    mean_distance = np.mean(distances)
    variance_distance = np.var(distances, ddof=0)

    trajectory = np.array([pos[:2] for pos in data["Start Position"]])  # 只取 x 和 y 作为轨迹
    if trajectory.shape[0] >= 3:  # 至少需要三个点来计算 jerks
        jerks = compute_jerks(trajectory.T, dt=0.1)  # 假设 dt 为 0.1 秒，使用 trajectory.T 转置使之成为 (2, n)
        mean_jerk = np.mean(jerks) if jerks is not None else np.nan
    else:
        mean_jerk = np.nan
    # 将平均 jerk 添加到数据中
    data["Mean Jerk"] = mean_jerk

    curvature = compute_curvature(trajectory.T)

    data["curvature"] = curvature

    min_curvature = np.min(curvature)
    max_curvature = np.max(curvature)
    mean_curvature = np.mean(curvature)
    variance_curvature = np.var(curvature)

    velocity = np.array([pos[2] for pos in data["Start Position"]])  # 只取 v

    min_velocity = np.min(velocity)
    max_velocity = np.max(velocity)
    mean_velocity = np.mean(velocity)
    variance_velocity = np.var(velocity)

    
    # 打印结果
    print("Planning Time Analysis:")
    print("Minimum Planning Time: {}".format(min_time))
    print("Maximum Planning Time: {}".format(max_time))
    print("Average Planning Time: {}".format(mean_time))
    print("Variance of Planning Time: {}".format(variance_time))

    # 打印距离障碍物分析
    print("\nDistance to Obstacles Analysis:")
    print("Minimum Distance to Obstacle: {}".format(min_distance))
    print("Maximum Distance to Obstacle: {}".format(max_distance))
    print("Average Distance to Obstacle: {}".format(mean_distance))
    print("Variance of Distance to Obstacle: {}".format(variance_distance))

    # 打印加加速度分析
    print("\nJerks Analysis:")
    print("Jerks: {}".format(mean_jerk))

    # 打印曲率分析
    print("\nCurvature Analysis:")
    print("Minimum Curvature: {}".format(min_curvature))
    print("Maximum Curvature: {}".format(max_curvature))
    print("Average Curvature: {}".format(mean_curvature))
    print("Variance of Curvature: {}".format(variance_curvature))

    print("\nvelocity Analysis:")
    print("Minimum velocity: {}".format(min_velocity))
    print("Maximum velocity: {}".format(max_velocity))
    print("Average velocity: {}".format(mean_velocity))
    print("Variance of velocity: {}".format(variance_velocity))

    
    # 也可以返回结果以便进一步处理
    return {
    "planning_time_min": min_time,
    "planning_time_max": max_time,
    "planning_time_mean": mean_time,
    "planning_time_variance": variance_time,
    
    "distance_to_obstacles_min": min_distance,
    "distance_to_obstacles_max": max_distance,
    "distance_to_obstacles_mean": mean_distance,
    "distance_to_obstacles_variance": variance_distance,
    
    "jerks_mean": mean_jerk,
    
    "curvature_min": min_curvature,
    "curvature_max": max_curvature,
    "curvature_mean": mean_curvature,
    "curvature_variance": variance_curvature,
    
    "velocity_min": min_velocity,
    "velocity_max": max_velocity,
    "velocity_mean": mean_velocity,
    "velocity_variance": variance_velocity
}

# if __name__ == '__main__':
#     rospy.init_node('experiment_bag_reader', anonymous=True)
    
#     for i in range(10):
#         index = i + 1
        

#     path = '/home/h301/liaochao/workspace/current/Expriment/CILQR/Experiment/UncertaintyPropagation_CILQR/data/V1_sigma/scenario1/'
#     file = '1.bag'
#     bag_file = path + file

#     obstacles = [
#         (123.32, -306.74, 0, 3.63, 1.84),
#         (103.32, -306.74, 0, 3.63, 1.84),
#         (193.9, -230.74, -np.pi/2.0, 3.63, 1.84),
#         (192.9, -190.74, np.pi*4.0/3.0, 3.63, 1.84),
#         (189.6, -210.74, np.pi/2.0, 3.63, 1.84),
#         (191.0, -110.6, np.pi*4.0/3.0, 3.63, 1.84),
#         (123.4, -105.6, np.pi, 3.63, 1.84),
#         (103.4, -105.6, np.pi, 3.63, 1.84),
#         (83.4, -105.6, np.pi, 3.63, 1.84)
#     ]
    
#     # 读取数据并转换为DataFrame
#     df = read_experiment_bag(bag_file)
#     # plot_positions_with_obstacles(df, obstacles)
#     # 设置筛选条件
#     start_pos = [93, -310]  # 替换为你的起始坐标 (x, y)
#     end_pos = [133, -300]  # 替换为你的结束坐标 (x, y)
    
#     # 筛选数据
#     filtered_df = data_process(df, start_pos, end_pos)# 0.06 is the threshhold of follow and avoid obstacle

#     data_analysis(filtered_df,obstacles)
    
#     plot_positions_with_obstacles(filtered_df, obstacles,start_pos,end_pos)
#     # 将筛选后的 DataFrame 保存为 Excel 文件
#     output_file = path + 'filtered_output.xlsx'
#     filtered_df.to_excel(output_file, index=False)
    
#     print("Filtered data has been saved to {}".format(output_file))



def read_experiment_bag(bag_file):
    # 初始化数据字典
    data = {
        "Start Time": [],
        "Start Position": [],
        "Planning Time": [],
        "X": [],
        "U": []
    }
    
    # 检查文件是否存在
    if not os.path.exists(bag_file):
        print(f"File '{bag_file}' does not exist.")
        return None

    try:
        # 打开bag文件
        bag = rosbag.Bag(bag_file, 'r')

        # 读取特定话题的数据
        for topic, msg, t in bag.read_messages(topics=['/experiment']):
            data["Start Time"].append(msg.start_time.to_sec())
            data["Start Position"].append(msg.start_pos)
            data["Planning Time"].append(msg.planning_time)
            data["X"].append(msg.X)
            data["U"].append(msg.U)
    
    except rosbag.ROSBagException as e:
        # 捕获rosbag异常并打印错误信息
        print(f"Failed to read bag file '{bag_file}': {e}")
        return None
    
    finally:
        # 确保bag文件在处理后关闭
        try:
            bag.close()
        except:
            print(f"Failed to close bag file '{bag_file}'.")

    # 将数据转换为DataFrame
    df = pd.DataFrame(data)

    # 返回DataFrame
    return df

def process_multiple_bags(bag_files, obstacles, start_pos, end_pos, output_file):
    # 创建一个DataFrame来存储所有bag文件的分析结果
    all_analysis_results = pd.DataFrame()

    # 创建一个ExcelWriter，用于将不同bag文件的分析结果写入不同的sheet
    with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
        for i, bag_file in enumerate(bag_files):
            print(f"Processing {bag_file}...")

            # 读取bag文件，若文件不存在则跳过
            df = read_experiment_bag(bag_file)
            if df is None:
                print(f"Skipping {bag_file} as it does not exist.")
                continue

            # 筛选数据
            filtered_df = data_process(df, start_pos, end_pos)

            # 进行数据分析
            analysis_result = data_analysis(filtered_df, obstacles)

            # 保存筛选后的 DataFrame 到单独的sheet
            filtered_df.to_excel(writer, sheet_name=f"Filtered Data {i+1}", index=False)

            # 将分析结果转换为DataFrame并添加到汇总表中
            analysis_df = pd.DataFrame([analysis_result])
            analysis_df["bag_file"] = bag_file  # 添加bag文件名作为一列，方便追踪
            all_analysis_results = pd.concat([all_analysis_results, analysis_df], ignore_index=True)

        # 将所有分析结果写入到一个单独的sheet
        all_analysis_results.to_excel(writer, sheet_name="All Analysis Results", index=False)

    print(f"All data has been saved to {output_file}")

    # 截取指定的部分列

    selected_columns = [
    'bag_file',  # 添加这个列
    'planning_time_min', 
    'planning_time_max', 
    'planning_time_mean', 
    'planning_time_variance', 
    'distance_to_obstacles_min', 
    'curvature_min', 
    'curvature_max', 
    'curvature_mean', 
    'curvature_variance', 
    'velocity_min', 
    'velocity_max', 
    'velocity_mean', 
    'velocity_variance'
    ]


    selected_data = all_analysis_results[selected_columns]

    # 在同样的路径下保存所选列为新的Excel文件
    selected_output_file = output_file.replace(".xlsx", "_selected_columns.xlsx")
    selected_data.to_excel(selected_output_file, index=False)

    print(f"Selected columns have been saved to {selected_output_file}")



if __name__ == '__main__':
    rospy.init_node('experiment_bag_reader', anonymous=True)

    path1 = '/home/h301/liaochao/workspace/current/Expriment/Experiment_data/Experiment2/Sigma1/Scenario1/'
    name = 'CILQR_Base_one_obstacle'
    path2 = name+'/'
    path = path1 + path2
    # CCNMPC  CCNMPC_one_obstacle  CILQR  CILQR_Base  CILQR_one_obstacle  Frenet  NRB-RRT
    # CCNMPC  CILQR  CILQR_Base  Frenet
    # CILQR  CILQR_Base
    # CILQR  CILQR_Base  Frenet

    # CILQR-B  CILQR-M  CILQR-ME

    bag_files = [os.path.join(path, f'{i}.bag') for i in range(1, 11)]  # 包含1到10的bag文件

    obstacles = [
        # 1
        (123.32, -306.74, 0, 3.63, 1.84),
        # (103.32, -306.74, 0, 3.63, 1.84),
        # 2
        (193.9, -230.74, -np.pi/2.0, 3.63, 1.84),
        (190.5, -190.74, np.pi*4.0/3.0, 3.63, 1.84),
        (189.6, -210.74, np.pi/2.0, 3.63, 1.84),
        # 3
        (189.2, -111.6, np.pi*230.0/180.0, 3.63, 1.84),
        # 4
        (123.4, -105, np.pi, 3.63, 1.84),
        (103.4, -105, np.pi, 3.63, 1.84),
        (83.4, -105, np.pi, 3.63, 1.84)
    ]
    # 1
    start_pos = [93, -310]  # 替换为你的起始坐标 (x, y)
    # # # # start_pos = [110, -310]  # 替换为你的起始坐标 (x, y)
    end_pos = [133, -300]   # 替换为你的结束坐标 (x, y)
    # 2
    # start_pos = [179, -240]  # 替换为你的起始坐标 (x, y)
    # end_pos = [203, -180]   # 替换为你的结束坐标 (x, y)
    # 3
    # start_pos = [179, -121]  # 替换为你的起始坐标 (x, y)
    # end_pos = [199, -101]   # 替换为你的结束坐标 (x, y)
    # 4
    # start_pos = [73, -115]  # 替换为你的起始坐标 (x, y)
    # end_pos = [133, -95]   # 替换为你的结束坐标 (x, y)
    # all
    # start_pos = [0, -400]  # 替换为你的起始坐标 (x, y)
    # end_pos = [300, 0]   # 替换为你的结束坐标 (x, y)

    # 定义输出文件路径
    output_file = path + 'combined_analysis_output.xlsx'

    # 处理多个bag文件并保存到Excel
    process_multiple_bags(bag_files, obstacles, start_pos, end_pos, output_file)