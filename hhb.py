import sys
import os
import json
import numpy as np
from openpyxl import load_workbook
from pykml import parser
import requests
from openpyxl import Workbook
import multiprocessing

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../.."))

PI = 3.1415926535897932384626  # π

def ParseKmlData(kml_path):
    # 解析KML文件
    kml_data = None
    anchor = []
    with open(kml_path, 'r') as f:
        kml_data = parser.parse(f).getroot()

    # 定义命名空间
    namespace = {'kml': 'http://www.opengis.net/kml/2.2'}

    # 查找Placemark元素
    placemarks = kml_data.findall('.//kml:Placemark', namespace)

    traj_enu_points = []
    count = 1
    for placemark in placemarks:
        for coordinate in placemark.LineString.coordinates.text.strip().split("\n"):
                parts = coordinate.split(',')
                longitude = float(parts[0])
                latitude = float(parts[1])
                # print(longitude, latitude)
                count += 1
                # if len(anchor) == 0:
                #     anchor = np.array([longitude * PI / 180, latitude * PI / 180, 0])

                # enu_point = lla2enu(longitude * PI / 180, latitude * PI / 180, 0, anchor)[0][0]
                # traj_enu_points.append(enu_point)

    return traj_enu_points, anchor



if __name__ == "__main__":
    kml_file = "./data/xian_road.kml"
    ParseKmlData = ParseKmlData(kml_file)
