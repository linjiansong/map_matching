import numpy as np

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