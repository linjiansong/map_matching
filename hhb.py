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

    resolution = 0.1
    road_name_by_pixel = {}
    for road_name, road_segment in road_segments:
        start = road_segment[0]
        end = road_segment[-1]
        for point in [start, end]:
            key = tuple((point[0:2] / resolution).astype(np.int32))
            if key in road_name_by_pixel:
                road_name_by_pixel[key].append(road_name)
            else:
                road_name_by_pixel[key] = [road_name]

    return road_name_by_pixel
    
def GetTransformProbability(road_segment_by_name):
    road_segment_names = list(road_segment_by_name.keys())

    road_name_by_pixel = FindConnection(road_segment_by_name)

    resolution = 0.1
    # row is current state, column is next state
    transform_probability = np.zeros((len(road_segment_names), len(road_segment_names)))
    for road_idx, road_name in enumerate(road_segment_names):
        road_segment = road_segment_by_name[road_name]
        start_key = tuple((road_segment[0][0:2] / resolution).astype(np.int32))
        end_key = tuple((road_segment[-1][0:2] / resolution).astype(np.int32))

        neighbor_road_segments = set()
        neighbor_road_segments.update(road_name_by_pixel[start_key])
        neighbor_road_segments.update(road_name_by_pixel[end_key])

        probability = 1.0 / len(neighbor_road_segments)

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
    transform_probability = GetTransformProbability(road_segment_by_name)

    t2 = time.time()
    print("t2 - t1 = {}".format(t2 - t1))