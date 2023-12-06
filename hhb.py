import sys
import os
import time
import numpy
from pykml import parser
import xmltodict
import json
import simplekml
import math
from scipy.spatial import KDTree

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../.."))

from util import lla2enu, enu2lla

PI = 3.1415926535897932384626  # π

class PointInfo:
    def __init__(self, point, time_stamp, road_name="UNKNOWN"):
        self.point = point
        self.time_stamp = time_stamp
        self.road_name = road_name

class MapMatchingByHMM:
    def __init__(self) -> None:
        self.road_segment_by_name = None
        self.road_segment_names = None
        self.transform_probability = None
        self.road_connection = None

    def SetRoadNetwork(self, road_segment_by_name):
        self.road_segment_by_name = road_segment_by_name
        self.road_segment_names = list(road_segment_by_name.keys())
        self.transform_probability, self.road_connection = self.GetTransformProbability()

        all_start_points = []
        for road_segment_name, road_segment in road_segment_by_name.items():
            start_point = road_segment[0]
            all_start_points.append(start_point[0:2])
        self.kdtree = KDTree(all_start_points)

    def FindConnection(self, resolution):
        road_segments = list(self.road_segment_by_name.items())
        connection_info = {}
        for road_name, road_segment in road_segments:
            connection_info[road_name] = []

        road_name_by_pixel = {}
        for road_name, road_segment in road_segments:
            start = road_segment[0]
            end = road_segment[-1]
            for point in [start, end]:
                key = tuple((point[0:2] / resolution).astype(numpy.int32))
                if key in road_name_by_pixel:
                    road_name_by_pixel[key].append(road_name)
                else:
                    road_name_by_pixel[key] = [road_name]
        return road_name_by_pixel

    def GetTransformProbability(self):
        resolution = 0.1
        road_name_by_pixel = self.FindConnection(resolution)

        road_connection = {}

        # row is current state, column is next state
        transform_probability = numpy.zeros((len(self.road_segment_names), len(self.road_segment_names)))
        for road_idx, road_name in enumerate(self.road_segment_names):
            road_segment = self.road_segment_by_name[road_name]
            start_key = tuple((road_segment[0][0:2] / resolution).astype(numpy.int32))
            end_key = tuple((road_segment[-1][0:2] / resolution).astype(numpy.int32))

            neighbor_road_segments = set()
            neighbor_road_segments.update(road_name_by_pixel[start_key])
            neighbor_road_segments.update(road_name_by_pixel[end_key])

            probability = 1.0 / len(neighbor_road_segments)

            for road_segment_name in neighbor_road_segments:
                neighbor_road_idx = self.road_segment_names.index(road_segment_name)
                transform_probability[road_idx, neighbor_road_idx] = probability

            road_connection[road_name] = list(neighbor_road_segments)
        return transform_probability, road_connection
    
    def GetProjectPoint(self, start_pt, end_pt, src_pt):
        tgt_vector = (end_pt - start_pt)[0:2]
        src_vector = (src_pt - start_pt)[0:2]
        if (numpy.linalg.norm(tgt_vector) < 1e-3):
            return 999, 999, -1, start_pt, src_pt
        if (numpy.linalg.norm(src_vector) < 1e-3):
            return 0, 0, 0, start_pt, src_pt

        ratio = numpy.dot(tgt_vector, src_vector.T) / (numpy.linalg.norm(tgt_vector) ** 2)
        projected_pt = start_pt + (end_pt - start_pt) * ratio

        if (ratio > 1):
            closest_pt = end_pt
        elif (ratio < 0):
            closest_pt = start_pt
        else:
            closest_pt = projected_pt

        distance = numpy.linalg.norm((src_pt - closest_pt)[0:2])
        vertical_distance = numpy.linalg.norm((src_pt - projected_pt)[0:2])
        projected_distance = numpy.linalg.norm((closest_pt - projected_pt)[0:2])
        return distance, vertical_distance, projected_distance, ratio

    def GetOberservationProbability(self, road_segment, traj_point):
        start_pt = road_segment[0]
        end_pt = road_segment[1]
        distance, _, projected_distance, _ = self.GetProjectPoint(start_pt, end_pt, traj_point)
        observation_probability = math.exp(-distance * distance / 50.0) * math.exp(-projected_distance * projected_distance / 50.0)
        return observation_probability

    def GetBestStateQueue(self, probability, optimal_paths, s_point_idx, e_point_idx):
        part_state_sequence = []
        if e_point_idx - s_point_idx < 1:
            part_state_sequence = ["UNKNOWN"]
        else:
            curr_road_idx = numpy.argmax(probability[e_point_idx], axis=0)
            part_state_sequence.append(self.road_segment_names[curr_road_idx])

            for point_idx in range(e_point_idx, s_point_idx, -1):
                prev_road_idx = optimal_paths[point_idx][curr_road_idx]
                part_state_sequence.append(self.road_segment_names[prev_road_idx])
                curr_road_idx = prev_road_idx
            part_state_sequence.reverse()
        return part_state_sequence

    def FindMatchedPath(self, enu_traj_points):
        if self.road_segment_by_name is None:
            print("Error: road network is not set!")
            return None
        
        all_state_probabilities = numpy.zeros((len(enu_traj_points), len(self.road_segment_names)))
        min_probability = 1e-9

        s_point_idx = 0
        state_sequence = []
        optimal_paths = [] # store prev optimal road index
        while (s_point_idx < len(enu_traj_points)):
            # initialize probability
            s_traj_point = enu_traj_points[s_point_idx]         
            prev_optimal_path = []

            for road_segment_idx in range(len(self.road_segment_names)):
                prev_optimal_path.append(road_segment_idx) # init prev optimal path is current road index
                all_state_probabilities[s_point_idx, road_segment_idx] = 0.0

            radius = 300
            indices = self.kdtree.query_ball_point(s_traj_point.point[0:2], radius)
            for road_segment_idx in indices:
                road_segment_name = self.road_segment_names[road_segment_idx]
                road_segment = self.road_segment_by_name[road_segment_name]
                all_state_probabilities[s_point_idx, road_segment_idx] = self.GetOberservationProbability(road_segment, s_traj_point.point)
                
            optimal_paths.append(prev_optimal_path)

            e_point_idx = s_point_idx
            for point_idx in range(s_point_idx + 1, len(enu_traj_points)):
                prev_state_probabilities = all_state_probabilities[point_idx - 1, :]
                e_point_idx = point_idx
               
                prev_state_probabilities /= numpy.sum(prev_state_probabilities) # normalize

                # t3 = time.time()
                prev_optimal_path = [None] * len(self.road_segment_names)
                for curr_road_segment_idx, state_probability in enumerate(prev_state_probabilities):
                    if state_probability < min_probability:
                        continue
                    road_segment_name = self.road_segment_names[curr_road_segment_idx]
                    next_maybe_road_segment_names = self.road_connection[road_segment_name]
                    transform_probability = 1 / len(next_maybe_road_segment_names)
                    for next_maybe_road_segment_name in next_maybe_road_segment_names:
                        fuse_probability = state_probability * transform_probability
                        next_maybe_road_segment_idx = self.road_segment_names.index(next_maybe_road_segment_name)

                        if fuse_probability > all_state_probabilities[point_idx, next_maybe_road_segment_idx]:
                            all_state_probabilities[point_idx, next_maybe_road_segment_idx] = fuse_probability
                            prev_optimal_path[next_maybe_road_segment_idx] = curr_road_segment_idx
                optimal_paths.append(prev_optimal_path)
                # print("point_idx = {}, optimal_paths = {}".format(point_idx, len(optimal_paths)))
                
                # t4 = time.time()
                # print("take {}s to calculate next state".format(t4 - t3))  

                count = 0
                for road_segment_idx, road_segment_name in enumerate(self.road_segment_names):
                    observation_probability = 0.0
                    if all_state_probabilities[point_idx, road_segment_idx] > min_probability:
                        curr_traj_point = enu_traj_points[point_idx]
                        road_segment = self.road_segment_by_name[road_segment_name]
                        observation_probability = self.GetOberservationProbability(road_segment, curr_traj_point.point)
                        count += 1
                    all_state_probabilities[point_idx, road_segment_idx] *= observation_probability
                # t5 = time.time()
                # print("take {}s to calculate final state, point idx = {}".format(t5 - t4, point_idx))

                if (numpy.max(all_state_probabilities[point_idx], axis=0) < min_probability):
                    e_point_idx -= 1
                    optimal_paths.pop() # remove last optimal path
                    break

            part_state_sequence = self.GetBestStateQueue(all_state_probabilities, optimal_paths, s_point_idx, e_point_idx)
            state_sequence.extend(part_state_sequence)
            s_point_idx = e_point_idx + 1
        return state_sequence

def ParseRoadNetworkKmlData(kml_path):
    # 解析KML文件
    kml_data = None
    unique_anchor = []
    with open(kml_path, 'r') as f:
        kml_data = parser.parse(f).getroot()

    # 定义命名空间
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

    # 查找Placemark元素
    placemarks = kml_data.findall('.//kml:Placemark', namespace)

    # 遍历Placemark元素
    road_segment_by_name = {}
    for place_mark_idx, placemark in enumerate(placemarks):
        # description_dict = xmltodict.parse(placemark.description.text.strip())
        description_dict = json.loads(placemark.description.text.strip())
        road_name = description_dict['Name']
        if len(road_name) == 0:
            road_name = "无名路"
        road_name = str(place_mark_idx) + "_" + road_name # 为了防止同名路，这里加上序号

        road_enu_points = []
        # 这里需要特别注意，不同的kml文件，LineString的坐标点的分隔符可能是空格，也可能是换行符
        for coordinate in placemark.LineString.coordinates.text.strip().split("\n"):
                parts = coordinate.split(',')
                longitude = float(parts[0])
                latitude = float(parts[1])
                if len(unique_anchor) == 0:
                    unique_anchor = numpy.array([longitude * PI / 180, latitude * PI / 180, 0])

                enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, unique_anchor)
                road_enu_points.append(enu_point)
        for segment_idx in range(len(road_enu_points) - 1):
            road_segment_name = road_name + "_" + str(segment_idx)
            if road_segment_name in road_segment_by_name:
                print("Error: road_segment_name {} already exists!".format(road_segment_name))
            road_segment_by_name[road_segment_name] = road_enu_points[segment_idx:segment_idx + 2]
        
    return road_segment_by_name, unique_anchor

# def ParseTrajectoryKmlData(kml_path, unique_anchor):
#     # 解析KML文件
#     kml_data = None
#     with open(kml_path, 'r') as f:
#         kml_data = parser.parse(f).getroot()

#     # 定义命名空间
#     namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

#     # 查找Placemark元素
#     placemarks = kml_data.findall('.//kml:Placemark', namespace)

#     # 遍历Placemark元素
#     traj_points = []
#     for placemark in placemarks:
#         # 这里需要特别注意，不同的kml文件，LineString的坐标点的分隔符可能是空格，也可能是换行符
#         for coordinate in placemark.LineString.coordinates.text.strip().split(" "):
#                 parts = coordinate.split(',')
#                 longitude = float(parts[0])
#                 latitude = float(parts[1])
#                 enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, unique_anchor)
#                 traj_points.append(enu_point)
#     return traj_points

def ParseRawTrajectoryKmlData(kml_path, unique_anchor):
    # 解析KML文件
    kml_data = None
    with open(kml_path, 'r') as f:
        kml_data = parser.parse(f).getroot()

    # 定义命名空间
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

    # 查找Placemark元素
    placemarks = kml_data.findall('.//kml:Placemark', namespace)

    traj_point_info_list = []
    # 遍历Placemark元素
    for placemark in placemarks:
        time_stamp = int(placemark.TimeStamp.when.text)
        point = placemark.Point.coordinates.text.strip().split(',')
        longitude = float(point[0])
        latitude = float(point[1])
        enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, unique_anchor)
        point_info = PointInfo(enu_point, time_stamp)
        traj_point_info_list.append(point_info)

    return traj_point_info_list

def ParseProcessedTrajectoryKmlData(kml_path, unique_anchor):
    # 解析KML文件
    kml_data = None
    with open(kml_path, 'r') as f:
        kml_data = parser.parse(f).getroot()

    # 定义命名空间
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

    # 查找Placemark元素
    placemarks = kml_data.findall('.//kml:Placemark', namespace)

    traj_point_info_list = []
    # 遍历Placemark元素
    for placemark in placemarks:
        time_stamp = int(placemark.TimeStamp.when.text)
        road_name = placemark.name.text
        point = placemark.Point.coordinates.text.strip().split(',')
        longitude = float(point[0])
        latitude = float(point[1])
        enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, unique_anchor)
        stamped_point = PointInfo(enu_point, time_stamp, road_name)
        traj_point_info_list.append(stamped_point)

    return traj_point_info_list

def SaveProcessTrajecoryAsKml(traj_points, state_sequence, file_path):
    kml = simplekml.Kml()
    if len(traj_points) != len(state_sequence):
        print("Error: len(traj_points) != len(state_sequence)")
        return
    
    for point_idx, point in enumerate(traj_points):
        lontitute, latitute, altitute = enu2lla(point.point, unique_anchor)
        new_point = kml.newpoint(coords=[(lontitute * 180 / PI, latitute * 180 / PI, 0)])
        new_point.timestamp.when = point.time_stamp
        new_point.name = state_sequence[point_idx]
    kml.save(file_path)

def GenerateDebugFile(unique_anchor, road_segment_by_name, optimal_paths, file_path):
    relevant_road_segment_names = set()
    for road_segment_name in optimal_paths:
        relevant_road_segment_names.add(road_segment_name)
    
    kml = simplekml.Kml()
    for road_segment_name in relevant_road_segment_names:
        if road_segment_name not in road_segment_by_name:
            continue
        road_segment = road_segment_by_name[road_segment_name]
        line = kml.newlinestring(name=road_segment_name)
        for point in road_segment:
            longitude, latitude, altitute = enu2lla(point, unique_anchor)           
            line.coords.addcoordinates([(longitude * 180 / PI, latitude * 180 / PI, 0)])
        line.style.linestyle.color = "6000FFFF"
        line.style.linestyle.width = 10
    
    kml.save(file_path)

if __name__ == "__main__":
    traj_names = ["0b1a2b1ea6c62a7e07c702d79095dc5c", "0b1ae909da3facd0208de3b8cbd2c058", "0c36314e523cd05af508bc75c4463d7a"]

    # Mode = "process"
    Mode = "statistic"

    if Mode == "process":
        road_kml_file = "./data/road_network/xian_road.kml"
        t1 = time.time()
        road_segment_by_name, unique_anchor = ParseRoadNetworkKmlData(road_kml_file)

        map_matcher = MapMatchingByHMM()
        map_matcher.SetRoadNetwork(road_segment_by_name)
        t2 = time.time()
        print("take {}s to set road network".format(t2 - t1))

        for traj_name in traj_names:
            traj_kml_file = "./data/trajectories/{}.kml".format(traj_name)
            traj_point_info_list = ParseRawTrajectoryKmlData(traj_kml_file, unique_anchor)

            t3 = time.time()
            state_sequence = map_matcher.FindMatchedPath(traj_point_info_list)
            t4 = time.time()
            print("take {}s to find matched path".format(t4 - t3))

            GenerateDebugFile(unique_anchor, road_segment_by_name, state_sequence, "./data/{}_matched.kml".format(traj_name))
            SaveProcessTrajecoryAsKml(traj_point_info_list, state_sequence, "./data/processed_trajectories/{}_process.kml".format(traj_name))
    
    if Mode == "statistic":
        # 锚点建议和"process"模式下的锚点一致
        unique_anchor = numpy.array([1.9022442564921247, 0.5981393479806824, 0.0])

        for traj_name in traj_names:
            traj_kml_file = "./data/processed_trajectories/{}_process.kml".format(traj_name)
            traj_point_info_list = ParseProcessedTrajectoryKmlData(traj_kml_file, unique_anchor)
            print("len(traj_point_info_list) = {}".format(len(traj_point_info_list)))
            # 具体怎么统计，根据具体情况具体分析
            pass


        
        # traj_points里面存的是轨迹点的位置及时间戳，state_sequence里面存的是每个轨迹点匹配到的路段名称

        # 下面给出计算最后一段轨迹点所在路段的代码，其他轨迹点所在路段的代码类似
        # last_state_name = None
        # e_state_idx = None
        # s_state_idx = None
        # for state_idx in range(len(state_sequence) - 1, -1, -1):
        #     curr_state_name = state_sequence[state_idx]
        #     if last_state_name is None and state_sequence[state_idx] != "UNKNOWN":
        #         last_state_name = curr_state_name
        #         e_state_idx = state_idx
        #     if last_state_name is not None and last_state_name != curr_state_name:
        #         s_state_idx = state_idx + 1
        #         break
        
        # if s_state_idx is not None and e_state_idx is not None:
        #     print("车辆位于 {} 路段".format(last_state_name))
        #     s_traj_point = traj_points[s_state_idx]
        #     e_traj_point = traj_points[e_state_idx]

        #     speed = numpy.linalg.norm((e_traj_point.point - s_traj_point.point)[0:2]) / (e_traj_point.time_stamp - s_traj_point.time_stamp)

        #     print("车辆位于 {} 路段, 平均速度为{}".format(state_sequence[s_state_idx], speed))
            # print("------------------------")
