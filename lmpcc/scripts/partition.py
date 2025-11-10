from geometry_msgs.msg import Point
import math
import numpy as np
import os
import matplotlib.pyplot as plt
import json
import ctypes

save_file_path1 = os.getcwd() + '/json_data/'
save_file_path = os.getcwd() + '/data/'

time_step = 0.1
prediction_horizon = 20

max_support_ = 6
confidence_ = 1e-6


def rootedNChooseK(N, k, root):
    result = 1.0
    for i in range(k):
        result *= ((N - (k - i)) / (k - i + 1)) ** (1.0 / root)

    return result


def epsilon(sample_size):
    support = max_support_

    return 1.0 - (confidence_ / (float)(max_support_)) ** (1.0 / ((float)(sample_size - support))) \
           * (1.0 / rootedNChooseK(sample_size, support, sample_size - support))


class road_statistics:
    def __init__(self) -> None:
        pass

    def closest_line(observable, road_network):
        closest_node = {}
        distance_right_minimum = {}
        distance_left_minimum = {}
        distance_right_minimum["distance"] = 1000
        distance_left_minimum["distance"] = 1000
        for i in range(len(road_network)):

            for j in range(len(road_network[str(i)]['right_side']) - 1):
                if (np.linalg.norm(road_network[str(i)]['right_side'][j] - [observable.pos_.x,
                                                                            observable.pos_.y]) + np.linalg.norm(
                        road_network[str(i)]['right_side'][j + 1] - [observable.pos_.x, observable.pos_.y]) <
                        distance_right_minimum["distance"]):
                    distance_right_minimum["distance"] = np.linalg.norm(
                        road_network[str(i)]['right_side'][j] - [observable.pos_.x,
                                                                 observable.pos_.y]) + np.linalg.norm(
                        road_network[str(i)]['right_side'][j + 1] - [observable.pos_.x, observable.pos_.y])
                    distance_right_minimum["id"] = i
                    distance_right_minimum["nodes"] = np.array(
                        [road_network[str(i)]['right_side'][j], road_network[str(i)]['right_side'][j + 1]])
                    distance_right_minimum["node_id"] = np.array([j, j + 1])
                if (np.linalg.norm(
                        road_network[str(i)]['left_side'][j] - [observable.pos_.x, observable.pos_.y]) + np.linalg.norm(
                        road_network[str(i)]['left_side'][j + 1] - [observable.pos_.x, observable.pos_.y]) <
                        distance_left_minimum["distance"]):
                    distance_left_minimum["distance"] = np.linalg.norm(
                        road_network[str(i)]['left_side'][j] - [observable.pos_.x, observable.pos_.y]) + np.linalg.norm(
                        road_network[str(i)]['left_side'][j + 1] - [observable.pos_.x, observable.pos_.y])
                    distance_left_minimum["id"] = i
                    distance_left_minimum["node_id"] = np.array([j, j + 1])
                    distance_left_minimum["nodes"] = np.array(
                        [road_network[str(i)]['left_side'][j], road_network[str(i)]['left_side'][j + 1]])
        if (distance_left_minimum["distance"] < distance_right_minimum["distance"]):
            closest_node["id"] = distance_left_minimum["id"]
            closest_node["nodes"] = distance_left_minimum["nodes"]
            closest_node["node_id"] = distance_left_minimum["node_id"]
            closest_node["side"] = "left"
        else:
            closest_node["id"] = distance_right_minimum["id"]
            closest_node["nodes"] = distance_right_minimum["nodes"]
            closest_node["node_id"] = distance_right_minimum["node_id"]
            closest_node["side"] = "right"
        return closest_node

    def sidewalk_angle(closest_node):
        node_vector = np.array([closest_node["nodes"][1, 0] - closest_node["nodes"][0, 0],
                                closest_node["nodes"][1, 1] - closest_node["nodes"][0, 1]])
        global_x_axis = [1, 0]
        dot_pdt = np.dot(node_vector, global_x_axis)
        determinant = node_vector[1] * global_x_axis[0] - node_vector[0] * global_x_axis[1]
        angle = math.atan2(determinant, dot_pdt)
        return angle

    def angle_to_closest_node(observable, closest_node):
        velocity_vector = np.array([[observable.vel_.x, observable.vel_.y]])
        node_vector = np.array([closest_node["nodes"][1, 0] - closest_node["nodes"][0, 0],
                                closest_node["nodes"][1, 1] - closest_node["nodes"][0, 1]])

        dot_pdt = np.dot(velocity_vector, node_vector)
        determinant = node_vector[0] * observable.vel_.y - node_vector[1] * observable.vel_.x
        norms = np.linalg.norm(velocity_vector) * np.linalg.norm(node_vector)
        # cos = dot_pdt/norms
        # angle = np.arccos(np.clip(cos, -1.0,1.0))
        angle = math.atan2(determinant, dot_pdt)
        return angle

    def sidewalk_velocity(observable, closest_node):
        velocity_vector = np.array([[observable.vel_.x], [observable.vel_.y]])
        node_vector = np.array([closest_node["nodes"][1, 0] - closest_node["nodes"][0, 0],
                                closest_node["nodes"][1, 1] - closest_node["nodes"][0, 1]])
        global_x_axis = [1, 0]
        dot_pdt = np.dot(node_vector, global_x_axis)
        determinant = node_vector[1] * global_x_axis[0] - node_vector[0] * global_x_axis[1]
        angle = math.atan2(determinant, dot_pdt)
        transformation_matrix = np.array([[math.cos(-angle), -math.sin(-angle)], [math.sin(-angle), math.cos(-angle)]])
        translation_matrix = np.array(
            [closest_node["nodes"][0, 0], closest_node["nodes"][0, 1]])  # Not needed for velocity rotation
        sidewalk_velocity = np.matmul(transformation_matrix, velocity_vector)
        return sidewalk_velocity[0], sidewalk_velocity[1]

    def sidewalk_distance(observable, closest_node):
        if (closest_node["id"] == 0):
            if (closest_node["side"] == "left"):
                if ((observable.pos_.y >= 4.0)):
                    return  abs(observable.pos_.y - (4.0))
                else:
                    return  abs(observable.pos_.y) - 4.0

            else:
                if ((observable.pos_.y <= -4.0)):
                    return  abs(observable.pos_.y - (-4.0))

                else:
                    return  abs(observable.pos_.y) - 4.0

        elif (closest_node["id"] == 1):
            if (closest_node["side"] == "left"):
                if ((observable.pos_.x >= 341)):
                    return  abs(observable.pos_.x - (341))
                else:
                    return  abs(observable.pos_.x) - 341

            else:
                if ((observable.pos_.x <= 333)):
                    return  abs(observable.pos_.x - (333))

                else:
                    return  333. - abs(observable.pos_.x)

    def crosswalk_distance(observable, closest_node):
        crosswalk = Point()
        crosswalk.x = [222., 227., 325., 333.]
        crosswalk.y = [-6., 4.0]
        if (closest_node["id"] == 0):
            if ((observable.pos_.x > crosswalk.x[0] and observable.pos_.x < crosswalk.x[1]) or (
                    observable.pos_.x > crosswalk.x[2] and observable.pos_.x < crosswalk.x[3])):
                if (observable.pos_.x > crosswalk.x[0] and observable.pos_.x < crosswalk.x[1]):
                    if (abs(observable.pos_.x - crosswalk.x[0]) < abs(observable.pos_.x - crosswalk.x[1])):
                        return -(abs(observable.pos_.x - crosswalk.x[0]))
                    else:
                        return -(abs(observable.pos_.x - crosswalk.x[1]))

                elif (observable.pos_.x > crosswalk.x[2] and observable.pos_.x < crosswalk.x[3]):
                    if (abs(observable.pos_.x - crosswalk.x[2]) < abs(observable.pos_.x - crosswalk.x[3])):
                        return -(abs(observable.pos_.x - crosswalk.x[2]))
                    else:
                        return -(abs(observable.pos_.x - crosswalk.x[3]))

            elif (observable.pos_.x <= crosswalk.x[0]):
                if (observable.vel_.x >= 0):
                    if (abs(observable.pos_.x - crosswalk.x[0]) < 10):
                        return abs(observable.pos_.x - crosswalk.x[0])
                    else:
                        return 10
                else:
                    return 10


            elif (observable.pos_.x >= crosswalk.x[1]):
                if (observable.vel_.x <= 0):
                    if (abs(observable.pos_.x - crosswalk.x[1]) < 10):
                        return abs(observable.pos_.x - crosswalk.x[1])
                    elif (abs(observable.pos_.x - crosswalk.x[3]) < 10):
                        return abs(observable.pos_.x - crosswalk.x[3])
                    else:
                        return 10
                else:
                    if (abs(observable.pos_.x - crosswalk.x[2]) < 10):
                        return abs(observable.pos_.x - crosswalk.x[2])
                    else:
                        return 10

        if (closest_node["id"] == 1):

            if (observable.pos_.y > crosswalk.y[0] and observable.pos_.y < crosswalk.y[1]):
                if (abs(observable.pos_.y - crosswalk.y[0]) < abs(observable.pos_.y - crosswalk.y[1])):
                    return -(abs(observable.pos_.y - crosswalk.y[0]))
                else:
                    return -(abs(observable.pos_.y - crosswalk.y[1]))

            elif (observable.pos_.y <= crosswalk.y[0]):
                if (observable.vel_.y >= 0):
                    if (abs(observable.pos_.y - crosswalk.y[0]) < 10):
                        return abs(observable.pos_.y - crosswalk.y[0])
                    else:
                        return 10
                else:
                    return 10


            elif (observable.pos_.y >= crosswalk.y[1]):
                if (observable.vel_.y <= 0):
                    if (abs(observable.pos_.y - crosswalk.y[1]) < 10):
                        return abs(observable.pos_.y - crosswalk.y[1])
                    else:
                        return 10
                else:
                    return 10


class convert_traj_vel:
    def __init__(self) -> None:
        pass

    def to_sidewalk_vel(angle, velocity_vector):
        transformation_matrix = np.array([[math.cos(-angle), -math.sin(-angle)], [math.sin(-angle), math.cos(-angle)]])

        sidewalk_velocity = np.matmul(transformation_matrix, velocity_vector)

        return sidewalk_velocity

    def to_world_vel(angle, velocity_vector):
        transformation_matrix = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])

        global_vel = np.matmul(transformation_matrix, velocity_vector)

        return global_vel


class Goal_Position:
    def __init__(self) -> None:
        pass

    def goal_prediction(observable, trajectory):
        pass


class Observables:
    def __init__(self, pos, vel):
        self.pos_ = pos
        self.vel_ = vel


class Partition:

    def data_to_index(self, observable):
        pass


class CloseToSidewalk(Partition):

    def __init__(self):
        self.size_ = 2
        self.name_ = 'close_to_sidewalk'

    def data_to_index(self, observable):
        sidewalk = Point()
        sidewalk.x = [75., 140., 220.]
        sidewalk.y = [-2., -2., -2.]

        for i in range(len(sidewalk.x)):
            dx = observable.pos_.x - sidewalk.x[i]
            dy = observable.pos_.y - sidewalk.y[i]
            threshold = 20
            if (math.sqrt(dx ** 2 + dy ** 2) < threshold):
                return 1
        return 0


class OnSidewalk(Partition):
    def __init__(self):
        self.size_ = 3
        self.name_ = 'on_sidewalk'

    def data_to_index(self, observable):
        sidewalk = Point()
        sidewalk.x = [75., 142., 225.]
        sidewalk.y = [0., 0., 0.]

        width = 8

        # for i in range(len(sidewalk.x)):
        #     # print("SIDEWALK",abs(observable.pos_.x - sidewalk.x[i]),(observable.pos_.y > -9 and observable.pos_.y < 7))
        if ((observable.pos_.y < -4.0)):
            # (observable.pos_.y > sidewalk.y[i] - 4 and observable.pos_.y < sidewalk.y[i] + 4)):
            return abs(observable.pos_.y - (-4.0))
        elif ((observable.pos_.y > 4.0)):
            # (observable.pos_.y > sidewalk.y[i] - 4 and observable.pos_.y < sidewalk.y[i] + 4)):
            return abs(observable.pos_.y - (4.0))
        else:
            return abs(observable.pos_.y) - 4.0


class Sidewalk_Position(Partition):
    def __init__(self):
        self.size_ = 3
        self.name_ = 'on_sidewalk'

    def data_to_index(self, observable):

        # for i in range(len(sidewalk.x)):
        #     # print("SIDEWALK",abs(observable.pos_.x - sidewalk.x[i]),(observable.pos_.y > -9 and observable.pos_.y < 7))
        if ((observable.pos_.y <= -4.0)):
            # (observable.pos_.y > sidewalk.y[i] - 4 and observable.pos_.y < sidewalk.y[i] + 4)):
            # if((observable.pos_.y <= -4.0 and observable.pos_.y > -4.5  )):
            #     return 1
            # else:
            #     return 2
            return 1

        elif ((observable.pos_.y >= 4.0)):
            # if(observable.pos_.y >= 4.0 and observable.pos_.y < 4.5  ):
            #     return 3
            # else:
            #     return 4
            return 2
            # (observable.pos_.y > sidewalk.y[i] - 4 and observable.pos_.y < sidewalk.y[i] + 4)):
        return 0

    # def data_to_index(self, observable):
    #     sidewalk = Point()
    #     sidewalk.x = [75., 142., 225.]
    #     sidewalk.y = [0., 0., 0.]

    #     width = 8

    #     # for i in range(len(sidewalk.x)):
    #     #     # print("SIDEWALK",abs(observable.pos_.x - sidewalk.x[i]),(observable.pos_.y > -9 and observable.pos_.y < 7))
    #     if((observable.pos_.y < -4.0 )):
    #             # (observable.pos_.y > sidewalk.y[i] - 4 and observable.pos_.y < sidewalk.y[i] + 4)):
    #         return abs(observable.pos_.y - (-4.0))
    #     elif((observable.pos_.y > 4.0 )):
    #             # (observable.pos_.y > sidewalk.y[i] - 4 and observable.pos_.y < sidewalk.y[i] + 4)):
    #         return abs(observable.pos_.y - (4.0))
    #     else:
    #         return abs(observable.pos_.y) - 4.0


class DistanceToCrosswalk(Partition):
    def __init__(self):
        self.size_ = 3
        self.name_ = 'distance_crosswalk'

    def data_to_index(self, observable):
        sidewalk = Point()
        sidewalk.x = [222., 227.]
        sidewalk.y = [0., 0., 0.]

        if (observable.pos_.x > sidewalk.x[0] and observable.pos_.x < sidewalk.x[1]):
            return 0

        elif (observable.pos_.x < sidewalk.x[0]):
            if (observable.vel_.x >= 0):
                # return abs(sidewalk.x[0]-observable.pos_.x)
                return 1
            # else:
            #     # return (observable.pos_.x - sidewalk.x[0])
            #     return 2

        elif (observable.pos_.x > sidewalk.x[1]):
            if (observable.vel_.x <= 0):
                # return abs(sidewalk.x[1]-observable.pos_.x)
                return 1

                # return (sidewalk.x[1] - observable.pos_.x)
        return 2

    # def data_to_index(self, observable):
    #     crosswalk = Point()
    #     crosswalk.x = [222.,227.]
    #     crosswalk.y = [0., 0., 0.]

    #     if(observable.pos_.x > crosswalk.x[0] and observable.pos_.x < crosswalk.x[1]):
    #         return 0.

    #     elif(observable.pos_.x < crosswalk.x[0]):
    #         if(observable.vel_.x>0):
    #             if(abs(observable.pos_.x-crosswalk.x[1])<10):
    #                 return abs(observable.pos_.x-crosswalk.x[1])
    #             else:
    #                 return 10
    #         else:
    #             return 10

    #     elif(observable.pos_.x > crosswalk.x[1]):
    #         if( observable.vel_  .x<0 ):
    #             if(abs(observable.pos_.x-crosswalk.x[1])<10):
    #                 return abs(observable.pos_.x-crosswalk.x[1])
    #             else:
    #                 return 10
    #         else:
    #             return 10


class VelocityDirection(Partition):

    def __init__(self, size=2):
        self.size_ = size
        self.name_ = 'direction'
        self.angle = 0.0

    # We check the angle
    def data_to_index(self, observable):
        angle = math.atan2(observable.vel_.y, observable.vel_.x)

        # Rotate back
        angle += math.pi / self.size_
        self.angle = ((angle + 2 * math.pi) % (2 * math.pi))
        # Force angles greater than 0, then divide in even pieces
        return math.floor(((angle + 2 * math.pi) % (2 * math.pi)) / (2 * math.pi / self.size_))


class VelocityMagnitude(Partition):

    def __init__(self):
        self.size_ = 2
        self.name_ = 'speed'

    def data_to_index(self, observable):
        # speed = math.sqrt(observable.vel_.x**2 + observable.vel_.y**2)
        if (observable.vel_.x > 0.75):
            return 1
        # Error guard
        # if speed == 0.0:
        #     return -1

        # if speed > 1.0:
        #     return 0
        # # elif speed > 0.9:
        # #     return 1
        # else:
        #     return 1
        return 0


class ObservablestoJson():

    # We check the angle
    def all_observables(self, observable):
        waypoints_1 = np.array(
            [[200, 4.0], [210, 4.0], [220, 4.0], [230, 4.0], [240, 4.0], [250, 4.0], [260, 4.0], [270, 4.0], [280, 4.0],
             [290, 4.0], [300, 4.0], [310, 4.0], [320, 4.0], [330, 4.0], [346, 4.0], [350, 4.0], [360, 4.0],
             [370, 4.0]])
        waypoints_2 = np.array(
            [[200, -4.0], [210, -4.0], [220, -4.0], [230, -4.0], [240, -4.0], [250, -4.0], [260, -4.0], [270, -4.0],
             [280, -4.0], [290, -4.0], [300, -4.0], [310, -4.0], [320, -4.0], [330, -4.0], [341, -4.0], [350, -4.0],
             [360, -4.0], [370, -4.0]])
        waypoints_3 = np.array([[343, -4.0], [343, -14.0], [343, -24.0], [343, -34.0], [343, -44.0], [343, -54.0]])
        waypoints_4 = np.array([[333, -4.0], [333, -14.0], [333, -24.0], [333, -34.0], [333, -44.0], [333, -54.0]])
        road_network = {}
        road_network['0'] = {"left_side": waypoints_1, "right_side": waypoints_2}
        road_network['1'] = {"left_side": waypoints_3, "right_side": waypoints_4}
        closest_node = road_statistics.closest_line(observable, road_network)
        angle = road_statistics.angle_to_closest_node(observable, closest_node)
        sidewalk_angle = road_statistics.sidewalk_angle(closest_node)
        sidewalk_velocity = road_statistics.sidewalk_velocity(observable, closest_node)
        sidewalk_distance = road_statistics.sidewalk_distance(observable, closest_node)
        crosswalk_distance = road_statistics.crosswalk_distance(observable, closest_node)
        if (closest_node["side"] == "left"):
            side = 0
        else:
            side = 1

        return float(angle), sidewalk_distance, crosswalk_distance, closest_node["nodes"], closest_node['id'], float(
            sidewalk_velocity[0]), float(sidewalk_velocity[1]), sidewalk_angle


class Partitioner:

    def __init__(self):
        self.data_ = []
        self.index_json_data_observables_ = []
        self.index_json_data_data_ = []
        self.json_data_ = {}
        self.partitions_ = []
        self.partitions_.append(VelocityDirection(2))
        self.partitions_.append(DistanceToCrosswalk())
        self.partitions_.append(Sidewalk_Position())
        self.json_partitions_ = []
        self.json_partitions_.append(VelocityDirection(2))
        self.json_partitions_.append(DistanceToCrosswalk())
        self.json_partitions_.append(Sidewalk_Position())

        self.size_ = 1
        self.json_size_ = 1
        for partition in self.partitions_:
            self.size_ *= partition.size_
        for json_partition in self.json_partitions_:
            self.json_size_ *= json_partition.size_
            # Stride
        size = self.size_

        for partition in self.partitions_:
            size /= partition.size_
            partition.stride_ = (int)(size)
        self.idx_map_ = []
        for i in range(self.size_):
            self.idx_map_.append([])
            left_over = i
            for partition in self.partitions_:
                # print(partition.stride_)
                # Divide by stride
                ii = math.floor(left_over / partition.stride_)
                left_over = left_over % partition.stride_

                self.idx_map_[i].append(ii)

        # Stride
        json_size = self.json_size_
        for json_partition in self.json_partitions_:
            json_size /= json_partition.size_
            json_partition.stride_ = (int)(json_size)
        self.json_idx_map_ = []

        for i in range(self.json_size_):
            self.json_idx_map_.append([])
            left_over = i
            for json_partition in self.json_partitions_:
                # Divide by stride
                ii = math.floor(left_over / json_partition.stride_)
                left_over = left_over % json_partition.stride_
                #  print("BC",ii,left_over,json_partition.stride_)
                self.json_idx_map_[i].append(ii)
        #  print(self.json_idx_map_[i])
        for i in range(self.size_):
            self.data_.append([])
        for i in range(self.json_size_):
            self.index_json_data_observables_.append([])
            self.index_json_data_data_.append([])
            # Observables is just a single velocity now

    def append(self, observables, data):
        # print('Appended trajectory of size {} to {}'.format(len(data), self.data_to_index(observables)))
        Json_variable = False
        index = self.data_to_index(observables, Json_variable)
        Json_variable = True
        json_index = self.data_to_index(observables, Json_variable)  # json_size_

        if index != -1:
            self.data_[index].append(data)
        if json_index != -1:
            self.index_json_data_observables_[json_index].append((observables.vel_))
            self.index_json_data_data_[json_index].append((data))

    # Code By Anish
    def save_json(self):
        remove_at_save = False
        global save_file_path1
        print(save_file_path1)
        if remove_at_save:
            _, _, filenames = next(os.walk(save_file_path1))
            for filename in filenames:
                os.remove(save_file_path1 + filename)

        for i in range(self.json_size_):
            cur_file = open(save_file_path1 + 'json_data_' + str(i) + ".json", "w")
            # cur_file.write("%d\n" % (len(self.index_json_data_data_[i])))
            sort_array = np.array([])
            for j in range(len(self.index_json_data_data_[i])):
                json_x = {}
                json_y = {}
                for k in range(len(self.index_json_data_data_[i][j])):
                    trajectory_x = "x" + str(k)
                    trajectory_y = "y" + str(k)
                    json_x[trajectory_x] = self.index_json_data_data_[i][j][k].x
                    json_y[trajectory_y] = self.index_json_data_data_[i][j][k].y
                Trajectory_Number = str(j)
                self.json_data_.update({Trajectory_Number: {"Observable": math.sqrt(
                    self.index_json_data_observables_[i][j].x ** 2 + self.index_json_data_observables_[i][j].y ** 2 +
                    self.index_json_data_observables_[i][j].z ** 2)
                    , "Trajectory_x": json_x, "Trajectory_y": json_y}})
                sort_array = np.append(sort_array, math.sqrt(
                    self.index_json_data_observables_[i][j].x ** 2 + self.index_json_data_observables_[i][j].y ** 2 +
                    self.index_json_data_observables_[i][j].z ** 2))

            #  if(math.sqrt(self.index_json_data_observables_[i][j].x**2+self.index_json_data_observables_[i][j].y**2+self.index_json_data_observables_[i][j].z**2)==0.00):

            #     print((self.index_json_data_observables_[i][j].x,self.index_json_data_observables_[i][j].y,self.index_json_data_observables_[i][j].z))
            json_sort = np.argsort(sort_array)
            json_data_copy = self.json_data_.copy()
            for count in range(len(self.index_json_data_data_[i])):
                self.json_data_[str(count)] = json_data_copy[str(json_sort[count])]

            json.dump(self.json_data_, cur_file)

            cur_file.close()
            self.json_data_ = {}

    def data_to_index(self, observables, Json_variable):
        if (Json_variable == False):
            index = 0
            size = self.size_
            for partition in self.partitions_:
                size /= partition.size_
                new_index = partition.data_to_index(observables)
                # print("{}: {}".format(partition.name_, new_index))
                # Fault check
                # print(partition.size_)
                if new_index == -1 or new_index is None:
                    return -1

                # i = i[0]s[0] + i[1]s[1]s[0] + ... + i[N]s[N]s[N-1]...s[0]
                index += (int)(new_index * size)

            return index
        else:
            index = 0
            size = self.json_size_
            for partition in self.json_partitions_:
                size /= partition.size_
                new_index = partition.data_to_index(observables)
                # print("{}: {}".format(partition.name_, new_index))
                if new_index == -1 or type(new_index) == 'None':
                    return -1

                # i = i[0]s[0] + i[1]s[1]s[0] + ... + i[N]s[N]s[N-1]...s[0]
                index += (int)(new_index * size)
            return index

    def get_random_data_from(self, index, N):
        random_indices = np.random.randint(len(self.data_[index]), size=N)
        return [self.data_[index][i] for i in random_indices]

    def print_data_summary(self):
        for index in range(self.size_):
            print("Index {}: Size = {} |\t\t Safety = {}".format(index, len(self.data_[index]),
                                                                 epsilon(len(self.data_[index]))))

    def save_to_txt(self):
        remove_at_save = True

        if remove_at_save:
            _, _, filenames = next(os.walk(save_file_path))
            for filename in filenames:
                os.remove(save_file_path + filename)

        for index in range(self.size_):
            cur_file = open(save_file_path + 'data_' + str(index), "w")
            cur_file.write("%d\n" % (len(self.data_[index])))
            for i in range(len(self.data_[index])):
                for k in range(len(self.data_[index][i])):
                    cur_file.write("%.12f %12f\n" % (self.data_[index][i][k].x, self.data_[index][i][k].y))

            cur_file.close()
            # if k == len(self.data_[index][i]) - 1:
            #     cur_file.write("\n")
        meta_file = open(save_file_path + "meta", "w")
        meta_file.write("%d" % (self.size_))
        meta_file.close()

    def plot_data_comparison(self):
        x = []
        y = []
        idx = []
        for index in range(self.size_):
            random_data = self.get_random_data_from(index, 100)

            for traj in random_data:
                pos_x = 0.0
                pos_y = 0.0
                for i in range(len(traj)):
                    pos_x += traj[i].x * time_step
                    pos_y += traj[i].y * time_step
                    x.append(pos_x)
                    y.append(pos_y)
                    idx.append(index / self.size_)

        plt.scatter(y, x, c=idx)

    # Plot the data with and without this flag
    def compare_data_for_flag(self, flag_index):

        cur_partition = self.partitions_[flag_index]

        x = []
        y = []
        idx = []
        for i in range(cur_partition.size_):
            x.append([])
            y.append([])
            idx.append([])

        this_idx = -1
        for i in range(len(self.partitions_)):
            if (self.partitions_[i] is cur_partition):
                this_idx = i
                break

        # Go through all other values to construct the index
        selected_index = -1
        for index in range(self.size_):
            selected_value = self.idx_map_[index][this_idx]
            random_data = self.get_random_data_from(index, 100)

            for traj in random_data:
                pos_x = 0.0
                pos_y = 0.0
                for i in range(len(traj)):
                    pos_x += traj[i].x * time_step
                    pos_y += traj[i].y * time_step
                    x[selected_value].append(pos_x)
                    y[selected_value].append(pos_y)
                    idx[selected_value].append(index / self.size_)

        div = (int)(math.floor(cur_partition.size_ / 2))
        fig, axs = plt.subplots((int)(cur_partition.size_ / div), div)
        for i in range(cur_partition.size_):
            if (div > 1):
                axs[i % 2, (int)(math.floor(i / 2))].scatter(y[i], x[i], c=idx[i])
                axs[i % 2, (int)(math.floor(i / 2))].set_title(cur_partition.name_ + ' interval: ' + str(i))
            else:
                axs[i % 2].scatter(y[i], x[i], c=idx[i])
                axs[i % 2].set_title(cur_partition.name_ + ' interval: ' + str(i))

        plt.savefig('compare_' + cur_partition.name_ + '_' + str(cur_partition.size_) + '.svg', dpi=300)

