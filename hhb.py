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
    road_segments = {}  # 存储路段连接关系的字典，使用唯一的segment_id作为key

    with open(kml_path, 'r' ,encoding='utf-8') as f:
        kml_data = parser.parse(f).getroot()

    # 定义命名空间
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

    # 查找Placemark元素
    placemarks = kml_data.findall('.//kml:Placemark', namespace)

    # 遍历Placemark元素
    for place_mark_idx, placemark in enumerate(placemarks):
        # description_dict = xmltodict.parse(placemark.description.text.strip())
        description_dict = json.loads(placemark.description.text.strip())
        road_name = description_dict['Name']
        if len(road_name) == 0:
            road_name = "无名路" + str(place_mark_idx)
        segment_id = road_name + "_" + str(place_mark_idx)  # 使用路名和索引组合作为segment_id

        if road_name in road_segments:
            print("Error: road_name {} already exists!".format(road_name))
            continue

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

        road_segments[segment_id] = road_enu_points

    return road_segments, unique_anchor


if __name__ == "__main__":
    kml_file = "./data/xian_road.kml"
    t1 = time.time()
    road_segment_by_name, unique_anchor = ParseKmlData(kml_file)
    t2 = time.time()
    print("t2 - t1 = {}".format(t2 - t1))