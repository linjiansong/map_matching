import numpy as np
import math

def lla2xyz(lon, lat, alt):
    RADIUS = 6378137  # semi-major axis
    e = 1 / 298.257223563  # flattening
    N = RADIUS / np.sqrt(
        np.cos(lat) * np.cos(lat) + (1 - e) ** 2 * np.sin(lat) * np.sin(lat)
    )
    h = N + alt

    x = h * np.cos(lat) * np.cos(lon)
    y = h * np.cos(lat) * np.sin(lon)
    z = (N * (1 - e) ** 2 + alt) * np.sin(lat)
    return x, y, z

def ll2Rne(lon, lat):
    c_lat = np.cos(lat)
    s_lat = np.sin(lat)
    c_lon = np.cos(lon)
    s_lon = np.sin(lon)
    R = np.array(
        [
            [-s_lon, -s_lat * c_lon, c_lat * c_lon],
            [c_lon, -s_lat * s_lon, c_lat * s_lon],
            [0, c_lat, s_lat],
        ],
        dtype=np.float64,
    )
    return R

def xyz2enu(xyz, anchor):
    lon0 = anchor[0]
    lat0 = anchor[1]
    alt0 = anchor[2]
    Rn2e = ll2Rne(lon0, lat0)
    x0, y0, z0 = lla2xyz(lon0, lat0, alt0)
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]
    relX = x - x0
    relY = y - y0
    relZ = z - z0
    enu = np.hstack((relX, relY, relZ)).dot(Rn2e)
    return enu

def lla2enu(lon, lat, alt, anchor):
    x, y, z = lla2xyz(lon, lat, alt)
    xyz = np.hstack((x, y, z))
    enu = xyz2enu(xyz, anchor)
    return enu

def transformlat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
        0.1 * lng * lat + 0.2 * math.sqrt(math.fabs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 *
            math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 *
            math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 *
            math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transformlng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
        0.1 * lng * lat + 0.1 * math.sqrt(math.fabs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 *
            math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 *
            math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 *
            math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def gcj02_to_wgs84(lng, lat):
    """
    GCJ02(火星坐标系)转GPS84
    :param lng:火星坐标系的经度
    :param lat:火星坐标系纬度
    :return:
    """
    a = 6378245.0  # 长半轴
    ee = 0.00669342162296594323  # 偏心率平方

    if not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55): # 判断是否在中国
        return [lng, lat]
    dlat = transformlat(lng - 105.0, lat - 35.0)
    dlng = transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return [lng * 2 - mglng, lat * 2 - mglat]