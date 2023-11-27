import sys
import os
import time
import numpy
from pykml import parser
import xmltodict
import json
import simplekml
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../.."))

from util import lla2enu

PI = 3.1415926535897932384626  # π

class MapMatchingByHMM:
    def __init__(self) -> None:
        self.road_segment_by_name = None
        self.road_segment_names = None
        self.transform_probability = None

    def SetRoadNetwork(self, road_segment_by_name):
        self.road_segment_by_name = road_segment_by_name
        self.road_segment_names = list(road_segment_by_name.keys())
        self.transform_probability = self.GetTransformProbability()

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
        return transform_probability
    
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
        
        probability = numpy.zeros((len(enu_traj_points), len(self.road_segment_names)))
        min_probability = 1e-9

        s_point_idx = 0
        state_sequence = []
        optimal_paths = [] # store prev optimal road index
        while (s_point_idx < len(enu_traj_points)):
            # initialize probability
            s_traj_point = enu_traj_points[s_point_idx]           
            prev_optimal_path = []
            t1 = time.time()
            for road_idx, road_segment_name in enumerate(self.road_segment_names):
                road_segment = self.road_segment_by_name[road_segment_name]
                probability[s_point_idx, road_idx] = self.GetOberservationProbability(road_segment, s_traj_point)
                prev_optimal_path.append(road_idx) # init prev optimal path is current road index
            optimal_paths.append(optimal_paths)
            t2 = time.time()
            print("take {}s to initialize probability".format(t2 - t1))

            e_point_idx = s_point_idx
            for point_idx in range(s_point_idx + 1, len(enu_traj_points)):
                prev_probability = probability[point_idx - 1, :]
                e_point_idx = point_idx

                if (numpy.max(prev_probability, axis=0) < min_probability):
                    e_point_idx -= 1
                    break
                
                prev_probability /= numpy.sum(prev_probability) # normalize

                t4 = time.time()
                fuse_probability = numpy.zeros((len(self.road_segment_names), len(self.road_segment_names)))
                for road_idx in range(len(self.road_segment_names)):
                    fuse_probability[road_idx] = self.transform_probability[road_idx] * prev_probability[road_idx]
                t5 = time.time()
                print("take {}s to fuse probability".format(t5 - t4))

                probability[point_idx] = numpy.max(fuse_probability, axis=0) # max probability of a column
                column_max_indices = numpy.argmax(fuse_probability, axis=0) # max probability road index of a column
                optimal_paths.append(column_max_indices)
                t6 = time.time()
                print("take {}s to get path".format(t6 - t5))

                for road_idx, road_segment_name in enumerate(self.road_segment_names):
                    road_segment = self.road_segment_by_name[road_segment_name]
                    observation_probability = 0.0
                    if probability[point_idx, road_idx] > min_probability:
                        curr_traj_point = enu_traj_points[point_idx]
                        observation_probability = self.GetOberservationProbability(road_segment, curr_traj_point)
                    probability[point_idx, road_idx] *= observation_probability
                t7 = time.time()
                print("take {}s to calculate probability, point idx = {}".format(t7 - t6, point_idx))  

            part_state_sequence = self.GetBestStateQueue(probability, optimal_paths, s_point_idx, e_point_idx)
            state_sequence.extend(part_state_sequence)
            s_point_idx = e_point_idx + 1  
        
        return []

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

def ParseTrajectoryKmlData(kml_path, unique_anchor):
    # 解析KML文件
    kml_data = None
    with open(kml_path, 'r') as f:
        kml_data = parser.parse(f).getroot()

    # 定义命名空间
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

    # 查找Placemark元素
    placemarks = kml_data.findall('.//kml:Placemark', namespace)

    # 遍历Placemark元素
    traj_points = []
    for placemark in placemarks:
        # 这里需要特别注意，不同的kml文件，LineString的坐标点的分隔符可能是空格，也可能是换行符
        for coordinate in placemark.LineString.coordinates.text.strip().split(" "):
                parts = coordinate.split(',')
                longitude = float(parts[0])
                latitude = float(parts[1])
                enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, unique_anchor)
                traj_points.append(enu_point)
    return traj_points

if __name__ == "__main__":
    road_kml_file = "./data/road_network/xian_road.kml"
    t1 = time.time()
    road_segment_by_name, unique_anchor = ParseRoadNetworkKmlData(road_kml_file)

    map_matcher = MapMatchingByHMM()
    map_matcher.SetRoadNetwork(road_segment_by_name)
    t2 = time.time()
    print("take {}s to set road network".format(t2 - t1))

    traj_kml_file = "./data/trajectories/ed40697bfde345984ac096bd656df62f.kml"
    t1 = time.time()
    traj_points = ParseTrajectoryKmlData(traj_kml_file, unique_anchor)

    map_matcher.FindMatchedPath(traj_points)
