
import numpy as np
from pyproj import Proj, Transformer

# WGS84 -> ECEF 
def wgs84_to_ecef(lat, lon, alt):
    # WGS84 ellipsoid constants
    a = 6378137.0  # semi-major axis (meters)
    e2 = 6.69437999014e-3  # eccentricity squared

    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)  # Prime vertical radius

    X = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    Y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    Z = (N * (1 - e2) + alt) * np.sin(lat_rad)

    return X, Y, Z

# ECEF -> ENU 
def ecef_to_enu(x, y, z, ref_lat, ref_lon, ref_alt):
    ref_x, ref_y, ref_z = wgs84_to_ecef(ref_lat, ref_lon, ref_alt)

    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z

    # 기준점
    ref_lat_rad = np.radians(ref_lat)
    ref_lon_rad = np.radians(ref_lon)

    # ENU 회전 행렬
    t = np.array([
        [-np.sin(ref_lon_rad), np.cos(ref_lon_rad), 0],
        [-np.sin(ref_lat_rad) * np.cos(ref_lon_rad), -np.sin(ref_lat_rad) * np.sin(ref_lon_rad), np.cos(ref_lat_rad)],
        [np.cos(ref_lat_rad) * np.cos(ref_lon_rad), np.cos(ref_lat_rad) * np.sin(ref_lon_rad), np.sin(ref_lat_rad)]
    ])

    # Perform the matrix multiplication
    enu = t @ np.array([dx, dy, dz])
    return enu[0], enu[1], enu[2]

# ENU -> NED 
def enu_to_ned(x_enu, y_enu, z_enu):
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu
    return x_ned, y_ned, z_ned

# WGS84 -> UTM 
def wgs84_to_utm(lat, lon, zone_number=52): #서울 기준(52n)
    proj = Proj(proj="utm", zone=zone_number, ellps="WGS84", datum="WGS84")
    x, y = proj(lon, lat)
    return x, y


if __name__ == "__main__":
    # ex
    lat, lon, alt = map(float,input("WGS84 위도, 경도, 고도를 입력하세요 :").split()) #현재위치 비룡플라자(37.449248 126.656665 0.0) 37.23923625810047 126.77315944371854 0.0 
    # 37.23905007632922 126.77530160074377 0.0
    ref_lat, ref_lon, ref_alt = 37.238838359501933 ,   126.772902206454901  ,  0.000000000000000    # ref

    # WGS84 -> ECEF
    x, y, z = wgs84_to_ecef(lat, lon, alt)
    print(f"WGS84 -> ECEF : X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

    # ECEF -> ENU
    enu_x, enu_y, enu_z = ecef_to_enu(x, y, z, ref_lat, ref_lon, ref_alt)
    print(f"ECEF -> ENU: X={enu_x:.2f}, Y={enu_y:.2f}, Z={enu_z:.2f}")

    # ENU -> NED
    ned_x, ned_y, ned_z = enu_to_ned(enu_x, enu_y, enu_z)
    print(f"ENU -> NED: X={ned_x:.2f}, Y={ned_y:.2f}, Z={ned_z:.2f}")

    # WGS84 -> UTM
    utm_x, utm_y = wgs84_to_utm(lat, lon)
    print(f"WGS84 -> UTM : X={utm_x:.2f}, Y={utm_y:.2f}")