import rosbag
import os, sys
import math
from geometry_msgs.msg import Point
import pickle
import matplotlib.pyplot as plt
import random
from partition import Partitioner, Observables, VelocityDirection, DistanceToCrosswalk, Sidewalk_Position, OnSidewalk, ObservablestoJson, convert_traj_vel
import json
import numpy as np
import statistics

random.seed(30)

#  @TODO: Ideas for extensions:
# -> More velocity directions to cover the peds walking partially sideways
# -> Including a flag for being on a sidewalk
# -> Having a direction towards the sidewalk

# bag_path = os.getcwd() + '/../record/22_3_2021/timesteps_fixed_5runs'
bag_path = os.getcwd() + '/'
print("Bag: " + bag_path)

time_step = 0.12
prediction_horizon = 40

max_support_ = 6
confidence_ = 1e-6


def main():

    bag = rosbag.Bag(bag_path + 'cv_seed4_50.bag')
    obs = dict()
    obs_test=dict()
    veh = []
    vehicle_id = -1
    unique_obs_id=[]
    # For the vehicle and objects
    for topic, msg, t in bag.read_messages(topics=['/carla/ego_vehicle/odometry', '/carla/objects']):
        # If this is the vehicle message, save its position
        if topic == '/carla/ego_vehicle/odometry':
            veh.append(msg.pose.pose.position)

        # If more objects than the ego-vehicle are recorded
        if topic == '/carla/objects':
            if(len(msg.objects)) > 1:
                
                for obstacle in msg.objects:
                    print(obstacle.id)
                    if obstacle.id not in obs and obstacle.id not in obs_test:
                        # If this is the ego_vehicle, save its id, don't save it as an obstacle
                        if (math.sqrt((obstacle.pose.position.x - veh[-1].x) ** 2 + (
                                obstacle.pose.position.y - veh[-1].y) ** 2) < 0.01):
                            vehicle_id = obstacle.id
                        
                            
                        else:
                            
                            if(obstacle.id not in unique_obs_id ):
                                unique_obs_id.append(obstacle.id)                                     
                                unique_np=np.array(unique_obs_id)
                            if(len(unique_obs_id)<100):
                                obs_test[obstacle.id] = []
                            else:
                                obs[obstacle.id] = []
                    # append data if not an obstacle
                    if obstacle.id != vehicle_id:
                        if(obstacle.pose.position.x<200 or obstacle.pose.position.x>360):
                            continue
                        if((obstacle.pose.position.y<-10 and not(obstacle.pose.position.x > 325 and obstacle.pose.position.x < 340)) or obstacle.pose.position.y>10 or obstacle.pose.position.y<-35 ):
                            continue
                        if(obstacle.id in obs):
                            obs[obstacle.id].append(obstacle)
                        if(obstacle.id in obs_test):                         
                            obs_test[obstacle.id].append(obstacle)

    print('Done loading data')
    print(len(obs))
    print(len(obs_test))
    # Result here is a dict with id - position for obstacles
    # and position list for vehicle

    # Construct a velocity list
    vel = dict()
    pos = dict()
    # For each obstacle
    for id in obs.keys():

        # We need to define stints, since there are zero values in the data occasionally
        stint = 0
        # For all positions of this obstacle
        for i in range(0,len(obs[id]),1):
            if i == 0:
                
                vel[id] = dict()
                pos[id] = dict()
            else:
                cur_vel = Point(obs[id][i-1].twist.linear.x ,obs[id][i-1].twist.linear.y, 0.0)
                # cur_vel = Point((obs[id][i].pose.position.x - last_position.pose.position.x) / time_step, (obs[id][i].pose.position.y - last_position.pose.position.y) / time_step, 0.0)

                # If there is a stint already, and it has a size, but this data point is wrong, then the stint ends here
                if (stint in vel[id]) and (len(vel[id][stint]) > 0) and (cur_vel.x == 0.0 and cur_vel.y == 0.0):
                    stint += 1
                    continue

                # If this is the first data point in a stint
                if stint not in vel[id]:
                    vel[id][stint] = []
                    pos[id][stint] = []

                vel[id][stint].append(cur_vel)
                pos[id][stint].append(obs[id][i].pose.position)
                
            last_position = obs[id][i]

            
    vel_test = dict()
    pos_test = dict()
    
    for id in obs_test.keys():
        
        # We need to define stints, since there are zero values in the data occasionally
        stint = 0
        # For all positions of this obstacle
        for i in range(0,len(obs_test[id]),1):
            
            if i == 0:
                vel_test[id] = dict()
                pos_test[id] = dict()
            else:
                
                cur_vel_test = Point(obs_test[id][i-1].twist.linear.x ,obs_test[id][i-1].twist.linear.y, 0.0)
                # cur_vel_test = Point((obs_test[id][i].pose.position.x - last_position.pose.position.x) / time_step, (obs_test[id][i].pose.position.y - last_position.pose.position.y) / time_step, 0.0)
                # If there is a stint already, and it has a size, but this data point is wrong, then the stint ends here
                if (stint in vel_test[id]) and (len(vel_test[id][stint]) > 0) and (cur_vel_test.x == 0.0 and cur_vel_test.y == 0.0):
                    stint += 1
                    continue

                # If this is the first data point in a stint
                if stint not in vel_test[id]:
                    vel_test[id][stint] = []
                    pos_test[id][stint] = []

                vel_test[id][stint].append(cur_vel_test)
                pos_test[id][stint].append(obs_test[id][i].pose.position)
                
            last_position = obs_test[id][i]

    # Here we have for each obstacle id a set of stints with velocities that are not flawed
    # i.e., vel -> ID -> Stint -> Velocities
    # Trajectory method (vs stage method)
    # after N steps, start to associate V_1:N with V_0
    # Some plot to show some example data
    # data_catagorizer.plot_data_comparison()
    # # data_catagorizer.compare_data_for_flag(1)
    # data_catagorizer.print_data_summary()
    
    print('hi')
    plt.show()
    test_data=[]
    test_data_observables=[]
    test_data_json={}
    data_catagorizer = Partitioner()
    history = 8
    for id in pos_test.keys():
        
        for stint in pos_test[id]:
            # Set of velocities here

            json_count=0
            
            sidewalk_angle = 0.0
            for i in range(len(pos_test[id][stint])):
                
                if i > (prediction_horizon+history):
                    angle_array = []
                    sidewalk_dist_array = []
                    crosswalk_distance_array = []
                    sidewalk_velocity_x_array = []
                    sidewalk_velocity_y_array = []
                    sidewalk_angle_array = []
                    observable_vel_array = []
                    to_sidewalk = []
                    to_road = []
                    sidewalk_entered = 6
                    road_entered = 6
                    vel_0_x=[]
                    vel_0_y=[]
                    for k in range(0,history+1,2):
                        vel_0_x.append(vel_test[id][stint][i-prediction_horizon-2-k].x)
                        vel_0_y.append(vel_test[id][stint][i-prediction_horizon-2-k].y)
                        vel_0=Point()
                        vel_0.x = vel_test[id][stint][i-prediction_horizon-2-k].x
                        vel_0.y = vel_test[id][stint][i-prediction_horizon-2-k].y
                        pos_0 = pos_test[id][stint][i-prediction_horizon-2-k]
                        observable = Observables(pos_0, vel_0)
                        observable_vel_array.append(round(math.sqrt(vel_0.x**2 + vel_0.y**2),4))
                        observable_class = ObservablestoJson()
                        angle, sidewalk_distance, crosswalk_distance, _ ,side, sidewalk_velocity_x, sidewalk_velocity_y, sidewalk_angle = observable_class.all_observables(observable)
                        angle_array.append(angle)
                        sidewalk_dist_array.append(sidewalk_distance)
                        crosswalk_distance_array.append(crosswalk_distance)
                        sidewalk_velocity_x_array.append(sidewalk_velocity_x)
                        sidewalk_velocity_y_array.append(sidewalk_velocity_y)
                        sidewalk_angle_array.append(sidewalk_angle)
                    sidewalk_acceleration_x = np.mean(np.diff(np.array(sidewalk_velocity_x_array)))
                    sidewalk_acceleration_y = np.mean(np.diff(np.array(sidewalk_velocity_y_array)))
                    angle_rate_of_change =  np.mean(np.diff(np.array(angle_array)))
                    angle = np.mean(angle_array)

                    sidewalk_sign_change = np.sign(sidewalk_dist_array)
                    to_sidewalk = ((np.roll(sidewalk_sign_change, -1) - sidewalk_sign_change) < 0).astype(int)
                    to_road = ((np.roll(sidewalk_sign_change, -1) - sidewalk_sign_change) > 0).astype(int)
                    to_sidewalk[0] = 0
                    to_road[0] =0
                    to_sidewalk[-1] = 0
                    to_road[-1] =0
                    if(np.max(to_sidewalk)>0):
                        # print(to_sidewalk)
                        # print(to_road)
                        # print(sidewalk_dist_array)
                        sidewalk_entered = int(np.where(to_sidewalk==1)[0][0])
                    if(np.max(to_road)>0):
                        road_entered = int(np.where(to_road==1)[0][0])
                    
                    sidewalk_distance = sidewalk_dist_array[0]
                    crosswalk_distance = crosswalk_distance_array[0]
                    sidewalk_distance_change = np.mean(np.diff(np.array(sidewalk_dist_array)))
                    crosswalk_distance_change = np.mean(np.diff(np.array(crosswalk_distance_array)))
                    sidewalk_angle = sidewalk_angle_array[0]
                    vel_0.x = vel_test[id][stint][i-prediction_horizon-2].x
                    vel_0.y = vel_test[id][stint][i-prediction_horizon-2].y
                    observable_vel= observable_vel_array[0]
                    observable_acc = np.mean(np.diff(np.array(observable_vel_array)))
                    sidewalk_vel_x= sidewalk_velocity_x_array[0]
                    sidewalk_vel_y= sidewalk_velocity_y_array[0]
                    vel_0.z=0.
                    
                    observable_vel_x=vel_0.x
                    observable_vel_y=vel_0.y
                    
                    observable_pos_x=pos_test[id][stint][i-prediction_horizon-2].x
                    observable_pos_y=pos_test[id][stint][i-prediction_horizon-2].y
                    
                    cur_traj = pos_test[id][stint][i-prediction_horizon:i].copy()
                    cur_traj = cur_traj[::2]
                    # cur_traj = vel_test[id][stint][i-2-prediction_horizon:i-2].copy()
                    # print("Length",len(cur_traj))
                    # cur_traj = cur_traj[::2]
                    test_data.append(cur_traj)
                    json_x={}
                    json_y={}
                    
                    index = data_catagorizer.data_to_index(observable, True)
                    # index=1
                    
                    # direction.data_to_index(observable)
                    # angle= direction.angle 
                    # sidewalk= OnSidewalk()
                    # sidewalk_position = sidewalk.data_to_index(observable)
                    # crosswalk = DistanceToCrosswalk()
                    # croswalk_distance = crosswalk.data_to_index(observable)
                                        
                    test_data_observables.append([observable_vel,observable_vel_x,observable_vel_y,observable_pos_x,observable_pos_y, index, angle, sidewalk_distance, 
                                                                crosswalk_distance, side,sidewalk_acceleration_x, sidewalk_acceleration_y, 
                                                  sidewalk_angle, angle_rate_of_change,sidewalk_vel_x,sidewalk_vel_y,sidewalk_distance_change,crosswalk_distance_change, observable_acc,sidewalk_entered,road_entered])
                    
    random_samples=random.sample(range(1, len(test_data)), 10000)
    count=0
    for i in (random_samples):
        
        json_x={}
        json_y={}
        for k in range(len(test_data[i])):
            # trajectory_x="x"+str(k)
            # trajectory_y="y"+str(k)
            # # to_rotate = np.array([[test_data[i][k].x],[test_data[i][k].y]])
            # # rotated_velocity = convert_traj_vel.to_sidewalk_vel(test_data_observables[i][12], to_rotate)
            # json_x[trajectory_x]=float(test_data[i][k].x)
            # json_y[trajectory_y]=float(test_data[i][k].y)
            trajectory_x="x"+str(k)
            trajectory_y="y"+str(k)
            # to_rotate = np.array([[test_data[i][k].x],[test_data[i][k].y]])
            # rotated_velocity = convert_traj_vel.to_sidewalk_vel(test_data_observables[i][12], to_rotate)
            json_x[trajectory_x]=float(test_data[i][k].x)
            json_y[trajectory_y]=float(test_data[i][k].y)
            
            
        trajectory_number=str(count)
        test_data_json.update({trajectory_number:{
        "Direction":test_data_observables[i][6],
        "Sidewalk_Acceleration_x": test_data_observables[i][10],
        "Sidewalk_Acceleration_y": test_data_observables[i][11],
        "Sidewalk_Angle": test_data_observables[i][12],
        "Sidewalk":test_data_observables[i][7],
        "Sidewalk_change":test_data_observables[i][16],
        "Crosswalk_change":test_data_observables[i][17],
        "Crosswalk": test_data_observables[i][8], 
        "Side": test_data_observables[i][9], 
        "Direction_change": test_data_observables[i][13],
        "Index":test_data_observables[i][5],  
        "Observable_Velocity":test_data_observables[i][0],
        "Observable_Acceleration":test_data_observables[i][18],
        "Observable_Velocity_x":test_data_observables[i][1],
        "Observable_Velocity_y":test_data_observables[i][2],
        "Sidewalk_Velocity_x":test_data_observables[i][14],
        "Sidewalk_Velocity_y":test_data_observables[i][15],
        "Observable_Position_X": test_data_observables[i][3],
        "Observable_Position_Y": test_data_observables[i][4],
        "Sidewalk_Entered":test_data_observables[i][19],
        "Road_Entered":test_data_observables[i][20],
        "Trajectory_x": json_x,
        "Trajectory_y":json_y}})
        count+=1
    print(len(test_data_json))

    file_path=os.getcwd() + '/test_data/test_data.json'
    cur_file = open(file_path  , "w")
    
    json.dump(test_data_json, cur_file)
    cur_file.close()

    print("hi again")
    test_data=[]
    test_data_observables=[]
    test_data_json={}
    data_catagorizer = Partitioner()
    for id in pos.keys():
        
        for stint in pos[id]:
            # Set of velocities here

            json_count=0
            sidewalk_angle = 0.0
            for i in range(len(pos[id][stint])):
                
                if i > (prediction_horizon+history):
                    angle_array = []
                    sidewalk_dist_array = []
                    crosswalk_distance_array = []
                    sidewalk_velocity_x_array = []
                    sidewalk_velocity_y_array = []
                    sidewalk_angle_array = []
                    to_sidewalk = []
                    to_road = []
                    sidewalk_entered = 6
                    road_entered = 6


                    vel_0_x=[]
                    vel_0_y=[]
                    observable_vel_array = []
                    for k in range(0,history+1,2):
                        
                        vel_0_x.append(vel[id][stint][i-prediction_horizon-2-k].x)
                        vel_0_y.append(vel[id][stint][i-prediction_horizon-2-k].y)
                        vel_0=Point()
                        vel_0.x = vel[id][stint][i-prediction_horizon-2-k].x
                        vel_0.y = vel[id][stint][i-prediction_horizon-2-k].y
                        pos_0 = pos[id][stint][i-prediction_horizon-2-k]
                        observable = Observables(pos_0, vel_0)
                        angle, sidewalk_distance, crosswalk_distance, _ ,side, sidewalk_velocity_x, sidewalk_velocity_y, sidewalk_angle = observable_class.all_observables(observable)
                        angle_array.append(angle)
                        sidewalk_dist_array.append(sidewalk_distance)
                        crosswalk_distance_array.append(crosswalk_distance)
                        sidewalk_velocity_x_array.append(sidewalk_velocity_x)   
                        sidewalk_velocity_y_array.append(sidewalk_velocity_y)
                        sidewalk_angle_array.append(sidewalk_angle)
                        observable_vel_array.append(round(math.sqrt(vel_0.x**2 + vel_0.y**2),4))
                    sidewalk_acceleration_x = np.mean(np.diff(np.array(sidewalk_velocity_x_array)))
                    sidewalk_acceleration_y = np.mean(np.diff(np.array(sidewalk_velocity_y_array)))
                    angle_rate_of_change =  np.mean(np.diff(np.array(angle_array)))
                    angle = np.mean(angle_array)

                    sidewalk_sign_change = np.sign(sidewalk_dist_array)
                    to_sidewalk = ((np.roll(sidewalk_sign_change, -1) - sidewalk_sign_change) < 0).astype(int)
                    to_road = ((np.roll(sidewalk_sign_change, -1) - sidewalk_sign_change) > 0).astype(int)
                    to_sidewalk[0] = 0
                    to_road[0] =0
                    to_sidewalk[-1] = 0
                    to_road[-1] =0
                    if(np.max(to_sidewalk)>0):
                        sidewalk_entered = int(np.where(to_sidewalk==1)[0][0])
                    if(np.max(to_road)>0):
                        road_entered = int(np.where(to_road==1)[0][0])
                    sidewalk_distance = sidewalk_dist_array[0]
                    crosswalk_distance = crosswalk_distance_array[0]
                    sidewalk_distance_change = np.mean(np.diff(np.array(sidewalk_dist_array)))
                    crosswalk_distance_change = np.mean(np.diff(np.array(crosswalk_distance_array)))
                    sidewalk_angle = sidewalk_angle_array[0]
                    vel_0.x = vel[id][stint][i-prediction_horizon-2].x
                    vel_0.y = vel[id][stint][i-prediction_horizon-2].y
                    observable_vel= observable_vel_array[0]
                    observable_acc = np.mean(np.diff(np.array(observable_vel_array)))
                    sidewalk_vel_x= sidewalk_velocity_x_array[0]
                    sidewalk_vel_y= sidewalk_velocity_y_array[0]    
                    vel_0.z=0.               
                    observable_vel_x=vel_0.x
                    observable_vel_y=vel_0.y

                    observable_pos_x=pos[id][stint][i-prediction_horizon-2].x
                    observable_pos_y=pos[id][stint][i-prediction_horizon-2].y
                    cur_traj = vel[id][stint][i-prediction_horizon-2:i-2].copy()
                    cur_traj = cur_traj[::1]
                    test_data.append(cur_traj)
                    json_x={}
                    json_y={}
                    # test_data.append(cur_traj)
                    index=1
                    # direction = 
                    # angle, sidewalk_distance, crosswalk_distance, _ ,side, sidewalk_velocity_x, sidewalk_velocity_y, sidewalk_angle = observable_class.all_observables(observable)
                    # print(f"type of angle: {type(float(angle))}. sidewalk :{type(sidewalk_distance)}, crosswalk:{type(crosswalk_distance)}")
                    
                    test_data_observables.append([observable_vel,observable_vel_x,observable_vel_y,observable_pos_x,observable_pos_y, index, angle, sidewalk_distance, 
                                                                crosswalk_distance, side,sidewalk_acceleration_x, sidewalk_acceleration_y, 
                                                  sidewalk_angle, angle_rate_of_change,sidewalk_vel_x,sidewalk_vel_y,sidewalk_distance_change,crosswalk_distance_change, observable_acc,sidewalk_entered,road_entered])
    print("I am here")
    print(len(test_data))
    random_samples=random.sample(range(1, len(test_data)),200000) #, 554980) 864700) 
    count=0
    print("Last step")
    for i in (random_samples):
        if(count%100)==0:
            print(count)
        json_x={}
        json_y={}
        for k in range(len(test_data[i])):
            trajectory_x="x"+str(k)
            trajectory_y="y"+str(k)
            to_rotate = np.array([[test_data[i][k].x],[test_data[i][k].y]])
            rotated_velocity = convert_traj_vel.to_sidewalk_vel(test_data_observables[i][12], to_rotate)
            json_x[trajectory_x]=float(rotated_velocity[0])
            json_y[trajectory_y]=float(rotated_velocity[1])
            
        trajectory_number=str(count)
        test_data_json.update({trajectory_number:{
        "Direction":test_data_observables[i][6],
        "Sidewalk_Acceleration_x": test_data_observables[i][10],
        "Sidewalk_Acceleration_y": test_data_observables[i][11],
        "Sidewalk_Angle": test_data_observables[i][12],
        "Sidewalk":test_data_observables[i][7],
        "Sidewalk_change":test_data_observables[i][16],
        "Crosswalk_change":test_data_observables[i][17],
        "Crosswalk": test_data_observables[i][8], 
        "Side": test_data_observables[i][9], 
        "Index":test_data_observables[i][5],  
        "Observable_Velocity":test_data_observables[i][0],
        "Observable_Acceleration":test_data_observables[i][18],
        "Observable_Velocity_x":test_data_observables[i][1],
        "Observable_Velocity_y":test_data_observables[i][2],
        "Sidewalk_Velocity_x":test_data_observables[i][14],
        "Sidewalk_Velocity_y":test_data_observables[i][15],
        "Observable_Position_X": test_data_observables[i][3],
        "Observable_Position_Y": test_data_observables[i][4],
        "Sidewalk_Entered":test_data_observables[i][19],
        "Road_Entered":test_data_observables[i][20],
        "Trajectory_x": json_x,
        "Trajectory_y":json_y}})
        count+=1
        
 
    print(len(test_data_json))

    file_path=os.getcwd() + '/test_data/data.json'
    cur_file = open(file_path  , "w")
    
    json.dump(test_data_json, cur_file)
    cur_file.close()

if __name__ == '__main__':
    main()
                         