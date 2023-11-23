import sys
import os
import time
import numpy as np
from openpyxl import load_workbook
from pykml import parser
import xmltodict
import json

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../.."))

from util import lla2enu

PI = 3.1415926535897932384626  # π

def ParseKmlData(kml_path):
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
        road_name = str(place_mark_idx) + "_" + description_dict['Name']
        if len(road_name) == 0:
            road_name = str(place_mark_idx) + "_无名路"

        road_enu_points = []
        # 这里需要特别注意，不同的kml文件，LineString的坐标点的分隔符可能是空格，也可能是换行符
        for coordinate in placemark.LineString.coordinates.text.strip().split("\n"):
                parts = coordinate.split(',')
                longitude = float(parts[0])
                latitude = float(parts[1])
                if len(unique_anchor) == 0:
                    unique_anchor = np.array([longitude * PI / 180, latitude * PI / 180, 0])

                enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, unique_anchor)
                road_enu_points.append(enu_point)
        for segment_idx in range(len(road_enu_points) - 1):
            road_segment_name = road_name + "_" + str(segment_idx)
            if road_segment_name in road_segment_by_name:
                print("Error: road_segment_name {} already exists!".format(road_segment_name))
            road_segment_by_name[road_segment_name] = road_enu_points[segment_idx:segment_idx + 2]
        
    return road_segment_by_name, unique_anchor

def FindConnection(road_segment_by_name):
    road_segments = list(road_segment_by_name.items())
    connection_info = {}
    for road_name, road_segment in road_segments:
        connection_info[road_name] = []

    for i in range(len(road_segments) - 1):
        tgt_name, tgt_segment = road_segments[i]
        tgt_s_point = tgt_segment[0]
        tgt_e_point = tgt_segment[-1]
        for j in range(i + 1, len(road_segments)):
            src_name, src_segment = road_segments[j]
            src_s_point = src_segment[0]
            src_e_point = src_segment[-1]

            if np.linalg.norm(tgt_s_point - src_s_point) < 1e-3:
                connection_info[tgt_name].append(src_name)
                connection_info[src_name].append(tgt_name)
            elif np.linalg.norm(tgt_s_point - src_e_point) < 1e-3:
                connection_info[tgt_name].append(src_name)
                connection_info[src_name].append(tgt_name)
            elif np.linalg.norm(tgt_e_point - src_s_point) < 1e-3:
                connection_info[tgt_name].append(src_name)
                connection_info[src_name].append(tgt_name)
            elif np.linalg.norm(tgt_e_point - src_e_point) < 1e-3:
                connection_info[tgt_name].append(src_name)
                connection_info[src_name].append(tgt_name)
            print("----------------")
    return connection_info

    
def GetTransformProbability(road_segment_names, connection_info):
    # row is current state, column is next state
    transform_probability = np.zeros((len(road_segment_names), len(road_segment_names)))
    for road_idx, road_segment_name in enumerate(road_segment_names):
        neighbor_road_segments = connection_info[road_segment_name]

        total_relation = len(neighbor_road_segments) + 1
        probability = 1.0 / total_relation

        transform_probability[road_idx, road_idx] = probability
        for road_segment_name in neighbor_road_segments:
            neighbor_road_idx = road_segment_names.index(road_segment_name)
            transform_probability[road_idx, neighbor_road_idx] = probability
        
    return transform_probability


if __name__ == "__main__":
    kml_file = "./data/xian_road.kml"
    t1 = time.time()
    road_segment_by_name, unique_anchor = ParseKmlData(kml_file)
    connection_info = FindConnection(road_segment_by_name)
    road_segment_names = list(road_segment_by_name.keys())
    transform_probability = GetTransformProbability(road_segment_names, connection_info)

    t2 = time.time()
    print("t2 - t1 = {}".format(t2 - t1))