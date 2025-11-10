from tkinter import X
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import json
from sklearn.cluster import DBSCAN, AgglomerativeClustering
import os

def point_in_hull(point, hull, tolerance=1e-12):
    return all(
    (np.dot(eq[:-1], point) + eq[-1] <= tolerance)
    for eq in hull.equations)

def read_files(file_path):
    # print(file_path)
    with open(file_path, 'r') as file:
        
        lines=file.read().splitlines()

    return lines
# path =r"output"
# 
no_of_traj=100
path=os.getcwd()+"/output"
os.chdir(path)
area_hull=np.zeros(20,dtype=float)
for Distance_Threshold in ([2,3,5,7,10]):
    no_of_predicted_traj=0
    count=0
    combined_results=np.zeros(20)
    for file in os.listdir():
        # print(file)
        if(count%200 ==0):
           print(combined_results)
        if file.endswith('.txt'):
            # Create the filepath of particular file
            file_path =f"{path}/{file}"

            lines=read_files(file_path)
                
            x_values=np.array([])
            y_values=np.array([])

            for i in lines:
                
                b=i.split()
                x_values=np.append(x_values,round(float(b[0]),3))
                y_values=np.append(y_values,round(float(b[1]),3))
                

            x_values=np.reshape(x_values,(-1,20))
            y_values=np.reshape(y_values,(-1,20))
            rng = np.random.default_rng()
            points= np.zeros((2, x_values.shape[0]))
            # print(x_values.shape[0])
            if(x_values.shape[0]>no_of_predicted_traj):
                
                no_of_predicted_traj=x_values.shape[0]
            point_x=[]
            point_y=[]
            f = open('test_data.json')
            data = json.load(f)
            # points[0,:]=x_values[:,19]
            # points[1,:]=y_values[:,19]
            
            # clustering=DBSCAN(eps=0.5, min_samples=3).fit(cluster_points)
            final_cluster=[]
            trj_string="".join(filter(str.isdigit, file_path))
            observable_position_x = data[trj_string]["Observable_Position_X"]
            observable_position_y = data[trj_string]["Observable_Position_Y"]
            previous_position_x = observable_position_x
            previous_position_y = observable_position_y
            dt = 0.2
            for i in range(20):
                points[0,:]=x_values[:,i]
                points[1,:]=y_values[:,i]
                cluster_points=points.T
                clustering = AgglomerativeClustering(n_clusters=None, distance_threshold=Distance_Threshold, affinity='euclidean', linkage='complete').fit_predict(cluster_points)
                # if(i==0):
                # plt.scatter(points[0,:],points[1,:])
                cluster_points=points.T
                stage_array=np.ones(len(clustering))*i
                clustered_points = np.column_stack((cluster_points, clustering,stage_array))
                final_cluster.append(clustered_points)

                
                # print(trj_string)
                position_x =   previous_position_x + float(data[trj_string]["Trajectory_x"]["x"+str(i)])*dt
                position_y =   previous_position_y + float(data[trj_string]["Trajectory_y"]["y"+str(i)])*dt 
                point_x.append(round(position_x,3))
                point_y.append(round(position_y,3))
                previous_position_x = position_x
                previous_position_y = position_y
                # plt.scatter(point_x[i],point_y[i],c="black",marker = 'X')
            a=np.array(final_cluster)
            # print(a.shape)
            a=np.reshape(a,(len(x_values)*20,4))
            # print(a.shape)
            final_cluster=np.array(a)
            # print(final_cluster.shape)
            sorted_clustered_points = final_cluster[final_cluster[:, 2].argsort()]
            _, cluster_indices = np.unique(sorted_clustered_points[:, 2], return_index=True)
            grouped_clustered_points = np.split(sorted_clustered_points, cluster_indices[1:])
            is_cluster_in_stage=np.zeros(20)
            is_cluster_in_stage=is_cluster_in_stage.astype(int)

            # print(np.unique(clustering))
            for i in  (np.unique(clustering)):
                
                for k in range(0,20):
                    
                    group_stage=grouped_clustered_points[i][grouped_clustered_points[i][:,3]==k]
                    group_stage=group_stage[:,:2]
                    if(len(group_stage)>2 ):    
                        hull=ConvexHull(group_stage,qhull_options= 'QJ')
                        area_hull[k]+=hull.area
                        # print(k)
                        point=np.array([point_x[k],point_y[k]])
                        if(is_cluster_in_stage[k]==0):
                            is_cluster_in_stage[k]=point_in_hull(point, hull)
                            # print("Stage :{}, cluster :{} is {} ".format(k,i,point_in_hull(point, hull) ),trj_string )
                        # for simplex in hull.simplices:
                            
                        #     if(k==0):
                        #         print("ctrajector:{}, Stage :{}, cluster :{} is {} ".format(trj_string,k,i,point_in_hull(point, hull) ),trj_string )
                        #     # print()
                        #         plt.plot(group_stage[simplex,0],group_stage[simplex,1])
                # # print(is_cluster_in_stage)
            
                # print(area_hull)
                # plt.scatter(grouped_clustered_points[i][:,0],grouped_clustered_points[i][:,1])
            
            combined_results+=is_cluster_in_stage
            combined_results=combined_results.astype(int)
            # print(combined_results)
            # plt.show() 
            file_to_save="Results_ClusterThreshold_"+str(Distance_Threshold)+"_Predicted_trajs_"+str(no_of_predicted_traj)
            file_to_save_area  = file_to_save+"area"
            count+=1
            if(count%10==0):
             print(count)
            # # point=np.array(points[0,0],points[0,1])
        
    area_hull /= 10000
    np.savetxt(file_to_save_area,area_hull)
    np.savetxt(file_to_save,combined_results)

    # plt.show()
    
# plt.show() 