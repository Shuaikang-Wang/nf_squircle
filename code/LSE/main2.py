import pygame
import random
import time
import numpy as np

import env
import sensors
import features

detect_lines = True  # set False to show only the sensed laser points
detect_landmarks = True  # set False to stop data association

'''
TODO: change the mouse with my robot 
FeatureMAP.FEATURES 存储的是[线段方程，array(起点，终点)]也是当前时刻所有的线段
'''
position =(250,500)

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))


if __name__ == '__main__':
    # initiate the internal Map, the lidar some other relevant variables
    FeatureMAP = features.FeaturesDetection()
    environment = env.buildEnvironment((600, 1200), 'map/map1.png')
    originalMap = environment.map.copy()
    laser = sensors.LaserSensor(200, 200, originalMap, uncertanty=(0.5, 0.01))
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()
    originalMap = environment.map.copy()
    running = True
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0

    num_events = 0
    sum_time = 0

    while running:
        FEATURE_DETECTION = True
        BREAK_POINT_IND = 0
        ENDPOINTS = [0, 0]
        sensorON = False
        # check that the game is still running with the mouse on the environment
        # 检查鼠标是否任然在动
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("\nNumber of Events: %s - total time: %s sec - time per event: %s sec" %
                      (num_events, round(sum_time, 3), round(sum_time / num_events, 3)))
                print("")
                running = False
            if pygame.mouse.get_focused():#调用mouse模块中的get_focused()函数，用于获取当前鼠标是否在窗口内的状态
                sensorON = True
            elif not pygame.mouse.get_focused():
                sensorON = False
        # start a new scanning round
        if sensorON:
            # update the current mouse / robot position
            # position = pygame.mouse.get_pos()
            # laser.position = position

            position = (position[0],position[1] -1)
            laser.position = position

            # erase the last located features
            environment.infomap = originalMap.copy()
            # visualize the current scanning radius
            # 画出扫描半径！
            pygame.draw.circle(environment.infomap, (255, 0, 0), laser.position, laser.Range, 2)
            pygame.draw.line(environment.infomap, (255, 0, 0), laser.position,
                             (laser.position[0] + laser.Range, laser.position[1]))


#********************************************************************************************************************************************
            # collect Lidar data收集传感器数据
            #TODO: change this sensordata with our sensor 返回障碍物点云即可FeatureMAP.laser_points_set(sensor_data)
            #TODO:FeatureMAP.LASERPOINTS存的是障碍物点云数据
            sensor_data = laser.sense_obstacles()
            # convert the measured distances and angles to pixels, depending on the current robot position
            FeatureMAP.laser_points_set(sensor_data)

            for point in FeatureMAP.LASERPOINTS:#画图部分把point画成红点 该代码的功能是在一个环境中的特定位置设置一个红色的点。
                environment.infomap.set_at((int(point[0]), int(point[1])), (255, 0, 0))

            if not sensor_data:#把map画在屏幕上，更新绘制内容
                environment.map.blit(environment.infomap, (0, 0))
                pygame.display.update()
                continue
# *****************************************************************************************************#*****************************************************************************************************

            start_time = time.time()

            # start detecting seed segments and grow their region
            # ---------------------------------------------------------------------------------------------
            while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN) and detect_lines:
                # detect the next segment
                seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND)
                #seedSeg是 雷达点云中提取 一条能拟合的线段 返回的是[点，(index)]
                # print("seedSeg",seedSeg)
                if not seedSeg:
                    break
                else:
                    seedSegment = seedSeg[0] #当前这一次循环提取出来的 可拟合线段的点
                    INDICES = seedSeg[1] #当前这次线段的index

                    # grow the region of the segment, it has to contain enough points and has to be long enough
                    results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)#在当前线段的基础上向index范围的两侧扩充看是否能扩大线段
                    if not results:#如果result 为空那就 跳过，并从breakpoint开始遍历
                        BREAK_POINT_IND = INDICES[1]
                        # print("breakpoint",BREAK_POINT_IND )
                        continue#即从当前的线段的结尾index处开始遍历
                    else:
                        line_eq = results[0]
                        PB = results[1]
                        PF = results[2]
                        BREAK_POINT_IND = PF

                        # calculate the start and end pixel of the scanned line, so it can be drawn on the infomap
                        ENDPOINTS = FeatureMAP.projection_point2line(
                            line_eq, FeatureMAP.LASERPOINTS[[PB, PF - 1]])
                        # print("FeatureMAP.LASERPOINTS[[PB, PF - 1]]",FeatureMAP.LASERPOINTS[[PB, PF - 1]])
                        if detect_landmarks:  # save the current line segment
                            FeatureMAP.FEATURES.append([line_eq, ENDPOINTS])
                            # print("FeatureMAP.FEATURES",FeatureMAP.FEATURES)
                        else:  # draw the current line segment
                            COLOR = random_color()
                            for point in FeatureMAP.LASERPOINTS[PB:PF]:
                                pygame.draw.circle(environment.infomap, COLOR, (int(point[0]), int(point[1])), 2, 0)
                            pygame.draw.line(environment.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)

            if detect_landmarks and detect_lines:  # start data association
                new_rep = []  # new feature representation holds the line params, start & end point and a projection
                for feature in FeatureMAP.FEATURES:
                    # print("FeatureMAP.FEATURES",FeatureMAP.FEATURES)
                    # draw the detected line in green
                    # print("feature",feature)
                    #画出 目前时刻扫到的直线部分
                    pygame.draw.line(environment.infomap, (0, 255, 0), feature[1][0], feature[1][1], 1)
                    new_rep.append([feature[0], feature[1], FeatureMAP.projection_point2line(feature[0], np.zeros(2))])
                    #feature[1] 是线段端点
                # print("new_rep",new_rep)
                FeatureMAP.landmark_association(new_rep)
                FeatureMAP.FEATURES = []

            if detect_lines:
                sum_time += time.time() - start_time
            num_events += 1

        if detect_landmarks:  # draw all stored landmarks#画直线的部分
            for landmark in FeatureMAP.LANDMARKS:
                # 画出所有的直线部分
                # pygame.draw.line(environment.infomap, (0, 0, 255), landmark[1][0], landmark[1][1], 2)
                pass
        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()
