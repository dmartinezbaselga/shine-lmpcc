import torch
import torch.nn as nn
import numpy as np
from torch.autograd import Variable
import torch.optim as optim
import torchvision
from sklearn.cluster import DBSCAN, KMeans
import json
from kmodes.kprototypes import KPrototypes
from time import time
import pickle



def NormalizeData(data):
    print(np.min(data),np.max(data))
    return 2*(data - np.min(data)) / (np.max(data) - np.min(data)) - np.ones(len(data))
    
print("HI")
with open('data.json', 'r') as f:
  trajs = json.load(f)
print("HI")
# traj_features = np.array([])
traj_features = []
traj_names = np.empty_like(1)
print("HI")
weight = 1.5
for i in range(200000):
    traj_features.append(
    [trajs[str(i)]["Sidewalk"],
    trajs[str(i)]["Crosswalk"],
    trajs[str(i)]["Direction"],
    trajs[str(i)]["Sidewalk_Velocity_x"],
    trajs[str(i)]["Sidewalk_Velocity_y"],
    trajs[str(i)]["Sidewalk_change"],
    trajs[str(i)]["Sidewalk_Acceleration_x"],
    trajs[str(i)]["Sidewalk_Acceleration_y"]
    ]
    )
    # traj_names = np.append(traj_names,str(i))
traj_features = np.array(traj_features)
traj_features=np.reshape(traj_features,(-1,8))
print(traj_features.shape)
for i in range (traj_features.shape[1]):
  traj_features[:,i] = NormalizeData (traj_features[:,i])
  if ( i ==3 or i==4 ):
     traj_features[:,i] *= 4
  else:
    traj_features[:,i]*=2
print(f"Length: {len(traj_features)}")
# kp = KPrototypes(n_clusters=15, n_init=15)
kp = KMeans(n_clusters=22, random_state=10)

print("Sarting to train")
kp.fit(traj_features)
print(f"Train: {kp}")
a=time()
# array_clusters=kp.predict(traj_features)
pickle.dump(kp, open("model_continous.pkl", "wb"))
# kp = pickle.load(open("model_continous.pkl", "rb"))
array_clusters=kp.predict(traj_features)

count,numbers = np.unique(array_clusters, return_counts=True)
print(f"Count={numbers>1000}, Time to cluster {time()-a}")
array_clusters = np.expand_dims(array_clusters, axis=1)
new_array=np.hstack((traj_features,array_clusters))
np.savetxt('cluster_continous.txt',new_array)
with open('test_data.json', 'r') as f:
  trajs = json.load(f)
traj_features = np.array([])
traj_names = np.empty_like(1)
for i in range(10000):
    traj_features = np.append(
    traj_features,(
    trajs[str(i)]["Sidewalk"],
    trajs[str(i)]["Crosswalk"],
    trajs[str(i)]["Direction"],
    trajs[str(i)]["Sidewalk_Velocity_x"],
    trajs[str(i)]["Sidewalk_Velocity_y"],
    trajs[str(i)]["Sidewalk_change"],
    trajs[str(i)]["Sidewalk_Acceleration_x"],
    trajs[str(i)]["Sidewalk_Acceleration_y"]))
    traj_names = np.append(traj_names,str(i))
traj_features=np.reshape(traj_features,(-1,8))
print(f"Length: {len(traj_features)}")

for i in range (traj_features.shape[1]):
  traj_features[:,i] = NormalizeData (traj_features[:,i])
  if (i==1 or i ==3 or i==4 ):
     traj_features[:,i] *= 4
  else:
    traj_features[:,i]*=2 
print(f"Length: {len(traj_features)}")
  # traj_features[4:6,i] *= weight
print(kp.predict(traj_features))
np.savetxt('cluster_continous_test.txt',kp.predict(traj_features))
