"""
无人机飞行监控与航线规划系统 - 单文件整合版
功能：地图显示、坐标转换、障碍物管理、航线规划、飞行监控、通信拓扑、MAVLink数据流
"""
import streamlit as st
import folium
from streamlit_folium import st_folium
import json
import math
import time
import random
from datetime import datetime

# ==================== 页面配置 ====================
st.set_page_config(
    page_title="无人机飞行监控与航线规划系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# ==================== 样式 ====================
st.markdown("""
<style>
    .main-header {font-size: 2rem; font-weight: bold; color: #1f77b4; text-align: center; margin-bottom: 0.5rem;}
    .sub-header {font-size: 1rem; color: #666; text-align: center; margin-bottom: 1rem;}
    .metric-card {background: #f0f2f6; border-radius: 10px; padding: 12px; margin: 4px 0;}
    .status-online {color: #00cc00; font-weight: bold;}
    .status-offline {color: #cc0000; font-weight: bold;}
    .log-info {color: #0066cc;}
    .log-warning {color: #ff9900;}
    .log-error {color: #cc0000;}
    .log-success {color: #00cc00;}
</style>
""", unsafe_allow_html=True)

# ==================== 坐标转换模块 ====================
def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

def _transformlat(lng, lat):
    ret = -100.0 + 2.0*lng + 3.0*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    ret += (20.0*math.sin(6.0*lng*math.pi) + 20.0*math.sin(2.0*lng*math.pi)) * 2.0/3.0
    ret += (20.0*math.sin(lat*math.pi) + 40.0*math.sin(lat/3.0*math.pi)) * 2.0/3.0
    ret += (160.0*math.sin(lat/12.0*math.pi) + 320*math.sin(lat*math.pi/30.0)) * 2.0/3.0
    return ret

def _transformlng(lng, lat):
    ret = 300.0 + lng + 2.0*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    ret += (20.0*math.sin(6.0*lng*math.pi) + 20.0*math.sin(2.0*lng*math.pi)) * 2.0/3.0
    ret += (20.0*math.sin(lng*math.pi) + 40.0*math.sin(lng/3.0*math.pi)) * 2.0/3.0
    ret += (150.0*math.sin(lng/12.0*math.pi) + 300.0*math.sin(lng/30.0*math.pi)) * 2.0/3.0
    return ret

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = _transformlat(lng - 105.0, lat - 35.0)
    dlng = _transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - 0.00669342162296594323 * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((6378245.0 * (1 - 0.00669342162296594323)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (6378245.0 / sqrtmagic * math.cos(radlat) * math.pi)
    return lng + dlng, lat + dlat

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = _transformlat(lng - 105.0, lat - 35.0)
    dlng = _transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - 0.00669342162296594323 * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((6378245.0 * (1 - 0.00669342162296594323)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (6378245.0 / sqrtmagic * math.cos(radlat) * math.pi)
    return lng - dlng, lat - dlat

def haversine_distance(lng1, lat1, lng2, lat2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lng2 - lng1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

# ==================== 障碍物管理模块 ====================
def load_obstacles():
    try:
        with open("obstacles.json", 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return []

def save_obstacles(obs_list):
    try:
        with open("obstacles.json", 'w', encoding='utf-8') as f:
            json.dump(obs_list, f, ensure_ascii=False, indent=2)
        return True
    except Exception:
        return False

def point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def get_polygon_center(polygon):
    if not polygon:
        return None
    return [sum(p[0] for p in polygon)/len(polygon), sum(p[1] for p in polygon)/len(polygon)]

def get_polygon_bounds(polygon):
    if not polygon:
        return None
    lats = [p[1] for p in polygon]
    lngs = [p[0] for p in polygon]
    return {"min_lat": min(lats), "max_lat": max(lats), "min_lng": min(lngs), "max_lng": max(lngs)}

def line_intersects_polygon(start, end, polygon):
    def ccw(A, B, C):
        return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    n = len(polygon)
    for i in range(n):
        if ccw(start, polygon[i], polygon[(i+1)%n]) != ccw(end, polygon[i], polygon[(i+1)%n]) and \
           ccw(start, end, polygon[i]) != ccw(start, end, polygon[(i+1)%n]):
            return True
    return False

# ==================== 航线规划模块 ====================
def get_tangent_points(center, radius, point):
    cx, cy = center
    px, py = point
    dx, dy = px - cx, py - cy
    d = math.sqrt(dx*dx + dy*dy)
    if d <= radius:
        return []
    base_angle = math.atan2(dy, dx)
    offset_angle = math.acos(radius / d)
    return [
        [cx + radius*math.cos(base_angle+offset_angle), cy + radius*math.sin(base_angle+offset_angle)],
        [cx + radius*math.cos(base_angle-offset_angle), cy + radius*math.sin(base_angle-offset_angle)]
    ]

def generate_arc_points(center, radius, start_point, end_point, num=8):
    cx, cy = center
    sa = math.atan2(start_point[1]-cy, start_point[0]-cx)
    ea = math.atan2(end_point[1]-cy, end_point[0]-cx)
    if ea < sa:
        ea += 2*math.pi
    return [[cx + radius*math.cos(sa + t*(ea-sa)), cy + radius*math.sin(sa + t*(ea-sa))]
            for t in [i/(num+1) for i in range(1, num+1)]]

def plan_all_paths(start, end, obstacles, flight_height, safety_radius):
    colliding = []
    for obs in obstacles:
        if flight_height <= obs["height"] and line_intersects_polygon(start, end, obs["polygon"]):
            colliding.append(obs)
    
    direct = [start, end]
    if not colliding:
        return {"direct": direct, "left": direct, "right": direct, "optimal": direct,
                "distances": {"direct": haversine_distance(start[0],start[1],end[0],end[1]),
                             "left": haversine_distance(start[0],start[1],end[0],end[1]),
                             "right": haversine_distance(start[0],start[1],end[0],end[1]),
                             "optimal": haversine_distance(start[0],start[1],end[0],end[1])}}
    
    # 左绕飞
    left = [start]
    for obs in colliding:
        for v in obs["polygon"]:
            left.append(v)
    left.append(end)
    
    # 右绕飞
    right = [start]
    for obs in colliding:
        for v in reversed(obs["polygon"]):
            right.append(v)
    right.append(end)
    
    # 最优路径（圆弧）
    optimal = [start]
    for obs in colliding:
        center = get_polygon_center(obs["polygon"])
        bounds = get_polygon_bounds(obs["polygon"])
        if center and bounds:
            w = haversine_distance(bounds["min_lng"],bounds["min_lat"],bounds["max_lng"],bounds["min_lat"])
            h = haversine_distance(bounds["min_lng"],bounds["min_lat"],bounds["min_lng"],bounds["max_lat"])
            r = max(w, h) / 2 + safety_radius
            r_deg = r / 111000
            tangents = get_tangent_points(center, r_deg, start)
            if tangents:
                d1 = haversine_distance(tangents[0][0],tangents[0][1],end[0],end[1])
                d2 = haversine_distance(tangents[1][0],tangents[1][1],end[0],end[1])
                chosen = tangents[0] if d1 < d2 else tangents[1]
                optimal.append(chosen)
                optimal.extend(generate_arc_points(center, r_deg, chosen, end, 8))
    optimal.append(end)
    
    def path_len(p):
        return sum(haversine_distance(p[i][0],p[i][1],p[i+1][0],p[i+1][1]) for i in range(len(p)-1))
    
    return {"direct": direct, "left": left, "right": right, "optimal": optimal,
            "distances": {"direct": path_len(direct), "left": path_len(left),
                         "right": path_len(right), "optimal": path_len(optimal)}}

# ==================== MAVLink模拟模块 ====================
def init_mavlink_state():
    return {
        "seq": 0,
        "base_lat": 32.234097,
        "base_lon": 118.749413,
        "base_alt": 50.0,
        "lat": 32.234097,
        "lon": 118.749413,
        "alt": 50.0,
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "battery_voltage": 12.6,
        "battery_remaining": 85,
        "satellites": 10,
        "groundspeed": 0.0,
        "airspeed": 0.0,
        "heading": 0,
        "throttle": 0,
        "climb_rate": 0.0,
        "message_count": 0,
        "history": [],
        "stats": {}
    }

def update_mavlink(state):
    state["seq"] = (state["seq"] + 1) % 256
    state["lat"] = state["base_lat"] + random.uniform(-0.0005, 0.0005)
    state["lon"] = state["base_lon"] + random.uniform(-0.0005, 0.0005)
    state["alt"] = state["base_alt"] + random.uniform(-5, 5)
    state["roll"] = random.uniform(-0.3, 0.3)
    state["pitch"] = random.uniform(-0.3, 0.3)
    state["yaw"] = random.uniform(0, 6.28)
    state["groundspeed"] = random.uniform(0, 15)
    state["airspeed"] = random.uniform(0, 15)
    state["heading"] = random.randint(0, 360)
    state["throttle"] = random.randint(30, 80)
    state["climb_rate"] = random.uniform(-2, 2)
    state["battery_remaining"] = max(10, state["battery_remaining"] + random.uniform(-0.5, 0.3))
    state["satellites"] = random.randint(8, 12)
    state["message_count"] += 5
    
    ts = datetime.now().strftime("%H:%M:%S") + f".{datetime.now().microsecond//1000:03d}"
    msgs = [
        ("HEARTBEAT", f"HEARTBEAT seq={state['seq']} sys=1 comp=1"),
        ("GPS_RAW_INT", f"GPS lat={state['lat']:.6f} lon={state['lon']:.6f} alt={state['alt']:.1f}m sats={state['satellites']}"),
        ("ATTITUDE", f"ATT roll={state['roll']:.3f} pitch={state['pitch']:.3f} yaw={state['yaw']:.3f}"),
        ("BATTERY_STATUS", f"BATT remain={int(state['battery_remaining'])}%"),
        ("VFR_HUD", f"VFR spd={state['groundspeed']:.1f}m/s hdg={state['heading']} alt={state['alt']:.1f}m"),
    ]
    for msg_type, msg_str in msgs:
        state["history"].append({"time": ts, "type": msg_type, "text": msg_str})
        state["stats"][msg_type] = state["stats"].get(msg_type, 0) + 1
    if len(state["history"]) > 100:
        state["history"] = state["history"][-100:]

# ==================== 通信拓扑模块 ====================
COMM_NODES = {
    "GCS": {"name": "地面站 (GCS)", "type": "ground_station", "status": "online", "ip": "192.168.1.100", "port": 14550},
    "OBC": {"name": "机载计算机 (OBC)", "type": "onboard_computer", "status": "online", "ip": "192.168.1.101", "port": 14540},
    "FCU": {"name": "飞控单元 (FCU)", "type": "flight_controller", "status": "online", "ip": "192.168.1.102", "port": 14560},
}

COMM_LINKS = {
    "GCS_OBC": {"source": "GCS", "target": "OBC", "protocol": "UDP", "latency": 15, "packet_loss": 0.5, "status": "active"},
    "OBC_FCU": {"source": "OBC", "target": "FCU", "protocol": "MAVLink", "latency": 5, "packet_loss": 0.1, "status": "active"},
    "GCS_FCU": {"source": "GCS", "target": "FCU", "protocol": "MAVLink", "latency": 25, "packet_loss": 1.2, "status": "standby"},
}

def update_comm():
    for link in COMM_LINKS.values():
        link["latency"] = max(1, link["latency"] + random.randint(-2, 2))
        link["packet_loss"] = max(0, link["packet_loss"] + random.uniform(-0.1, 0.1))

def init_flight_state():
    return {
        "status": "idle",          # idle, flying, paused, landing, emergency
        "mode": "AUTO",            # AUTO, MANUAL, LOITER
        "path_index": 0,
        "path": [],
        "progress": 0.0,
        "flight_time": 0,
        "distance_flown": 0,
        "start_time": None,
    }

def update_flight_position(state, mavlink, speed=5.0):
    """根据航线更新飞行器位置，speed单位m/s"""
    if not state["path"] or state["path_index"] >= len(state["path"]) - 1:
        if state["status"] == "flying":
            state["status"] = "idle"
        return
    
    p1 = state["path"][state["path_index"]]
    p2 = state["path"][state["path_index"] + 1]
    
    seg_dist = haversine_distance(p1[0], p1[1], p2[0], p2[1])
    if seg_dist < 1:
        state["path_index"] += 1
        return
    
    # 每帧前进 speed 米（约0.5秒一帧 -> speed*0.5）
    step = speed * 0.5
    state["progress"] += step / seg_dist
    state["distance_flown"] += step
    
    if state["progress"] >= 1.0:
        state["progress"] = 0.0
        state["path_index"] += 1
        if state["path_index"] >= len(state["path"]) - 1:
            state["status"] = "idle"
        return
    
    t = state["progress"]
    lng = p1[0] + t * (p2[0] - p1[0])
    lat = p1[1] + t * (p2[1] - p1[1])
    
    mavlink["lon"] = lng
    mavlink["lat"] = lat
    
    # 航向
    heading = math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
    if heading < 0: heading += 360
    mavlink["heading"] = int(heading)
    mavlink["yaw"] = math.radians(heading)
    
    # 姿态微调（模拟飞行抖动）
    mavlink["roll"] = random.uniform(-0.05, 0.05)
    mavlink["pitch"] = random.uniform(-0.05, 0.05)

def set_flight_altitude(mavlink, target_alt, rate=2.0):
    """调整高度，rate爬升率m/s"""
    diff = target_alt - mavlink["alt"]
    step = rate * 0.5
    if abs(diff) <= step:
        mavlink["alt"] = target_alt
        mavlink["climb_rate"] = 0
    elif diff > 0:
        mavlink["alt"] += step
        mavlink["climb_rate"] = rate
    else:
        mavlink["alt"] -= step
        mavlink["climb_rate"] = -rate

# ==================== Session State 初始化 ====================
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = load_obstacles()

if 'point_a' not in st.session_state:
    st.session_state.point_a = [118.749413, 32.234097]

if 'point_b' not in st.session_state:
    st.session_state.point_b = [118.751413, 32.236097]

if 'flight_height' not in st.session_state:
    st.session_state.flight_height = 50.0

if 'safety_radius' not in st.session_state:
    st.session_state.safety_radius = 15.0

if 'mavlink' not in st.session_state:
    st.session_state.mavlink = init_mavlink_state()

if 'flight' not in st.session_state:
    st.session_state.flight = init_flight_state()

if 'flight_speed' not in st.session_state:
    st.session_state.flight_speed = 8.0

if 'map_center' not in st.session_state:
    st.session_state.map_center = [32.234097, 118.749413]

# 地图中心坐标
CENTER_LNG = 118.749413
CENTER_LAT = 32.234097
BOUNDS = {"north": 32.2370, "south": 32.2310, "east": 118.7530, "west": 118.7450}

# ==================== 侧边栏 ====================
st.sidebar.markdown("## 🚁 系统导航")
page = st.sidebar.radio(
    "选择模块",
    ["📍 地图与航线规划", "🌐 3D地图", "🚧 障碍物管理", "🔄 坐标转换工具", "📊 飞行监控", "📡 通信链路", "📻 MAVLink数据流"]
)

st.sidebar.markdown("---")
st.sidebar.markdown("### ⚙️ 飞行参数")
with st.sidebar.expander("参数设置", expanded=False):
    st.session_state.flight_height = st.number_input(
        "飞行高度 (米)", min_value=10.0, max_value=500.0,
        value=st.session_state.flight_height, step=5.0)
    st.session_state.safety_radius = st.number_input(
        "安全半径 (米)", min_value=5.0, max_value=100.0,
        value=st.session_state.safety_radius, step=5.0)

st.sidebar.markdown("---")
st.sidebar.info("**无人机飞行监控系统 v1.0**\n\n坐标系: GCJ-02\n地图: OSM/卫星")

# ==================== 页面1：地图与航线规划 ====================
if page.startswith("📍"):
    st.markdown('<div class="main-header">🗺️ 地图与航线规划</div>', unsafe_allow_html=True)
    st.markdown('<div class="sub-header">无人机航线规划与障碍物避让</div>', unsafe_allow_html=True)
    
    col1, col2 = st.columns([3, 1])
    
    with col2:
        st.markdown("### 📍 航点设置")
        st.markdown("**起点 A (GCJ-02)**")
        a_lng = st.number_input("A点经度", value=st.session_state.point_a[0], format="%.6f", key="a_lng")
        a_lat = st.number_input("A点纬度", value=st.session_state.point_a[1], format="%.6f", key="a_lat")
        st.session_state.point_a = [a_lng, a_lat]
        
        st.markdown("**终点 B (GCJ-02)**")
        b_lng = st.number_input("B点经度", value=st.session_state.point_b[0], format="%.6f", key="b_lng")
        b_lat = st.number_input("B点纬度", value=st.session_state.point_b[1], format="%.6f", key="b_lat")
        st.session_state.point_b = [b_lng, b_lat]
        
        st.markdown("---")
        st.markdown("### ✈️ 航线规划")
        
        if st.button("🔄 规划航线", use_container_width=True):
            with st.spinner("规划中..."):
                st.session_state.planned_paths = plan_all_paths(
                    st.session_state.point_a, st.session_state.point_b,
                    st.session_state.obstacles,
                    st.session_state.flight_height, st.session_state.safety_radius)
                st.success("规划完成！")
        
        if 'planned_paths' in st.session_state:
            paths = st.session_state.planned_paths
            st.markdown("**路径长度对比**")
            pdata = []
            for pt, name in [("direct","直飞"),("left","左绕飞"),("right","右绕飞"),("optimal","最优路径")]:
                pdata.append({"类型": name, "距离(m)": round(paths["distances"][pt], 1)})
            st.table(pdata)
            
            st.markdown("**显示路径**")
            show_direct = st.checkbox("直飞", True, key="sd")
            show_left = st.checkbox("左绕飞", False, key="sl")
            show_right = st.checkbox("右绕飞", False, key="sr")
            show_optimal = st.checkbox("最优路径", True, key="so")
    
    with col1:
        m = folium.Map(location=st.session_state.map_center, zoom_start=16, tiles="OpenStreetMap")
        folium.TileLayer(
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Esri", name="卫星地图").add_to(m)
        folium.LayerControl(position='topright').add_to(m)
        
        # 边界
        bbox = [[BOUNDS["south"],BOUNDS["west"]],[BOUNDS["south"],BOUNDS["east"]],
                [BOUNDS["north"],BOUNDS["east"]],[BOUNDS["north"],BOUNDS["west"]],
                [BOUNDS["south"],BOUNDS["west"]]]
        folium.Polygon(locations=bbox, color="blue", weight=2, fill=True,
                       fillColor="blue", fillOpacity=0.1, popup="作业区域").add_to(m)
        
        # 障碍物
        for obs in st.session_state.obstacles:
            poly = [[p[1], p[0]] for p in obs["polygon"]]
            folium.Polygon(locations=poly, color="red", weight=2, fill=True,
                           fillColor="red", fillOpacity=0.3,
                           popup=f"{obs['name']}<br>高:{obs['height']}m<br>安全半径:{obs['safety_radius']}m").add_to(m)
            c = get_polygon_center(obs["polygon"])
            if c:
                folium.Marker(location=[c[1], c[0]],
                    icon=folium.DivIcon(html=f'<div style="font-size:10px;color:red;font-weight:bold;background:white;padding:2px;">{obs["name"]}</div>')).add_to(m)
        
        # A、B点
        folium.Marker(location=[st.session_state.point_a[1], st.session_state.point_a[0]],
            popup=f"起点 A<br>{st.session_state.point_a[0]:.6f}, {st.session_state.point_a[1]:.6f}",
            icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
        folium.Marker(location=[st.session_state.point_b[1], st.session_state.point_b[0]],
            popup=f"终点 B<br>{st.session_state.point_b[0]:.6f}, {st.session_state.point_b[1]:.6f}",
            icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
        
        # 规划路径
        if 'planned_paths' in st.session_state:
            paths = st.session_state.planned_paths
            color_map = {"direct":"gray","left":"orange","right":"purple","optimal":"green"}
            name_map = {"direct":"直飞","left":"左绕飞","right":"右绕飞","optimal":"最优路径"}
            show_map = {"direct": show_direct if 'show_direct' in dir() else True,
                       "left": show_left if 'show_left' in dir() else False,
                       "right": show_right if 'show_right' in dir() else False,
                       "optimal": show_optimal if 'show_optimal' in dir() else True}
            for pt, pp in paths.items():
                if pt == "distances": continue
                if show_map.get(pt, False):
                    folium.PolyLine(locations=[[p[1],p[0]] for p in pp],
                        color=color_map.get(pt,"blue"), weight=3, opacity=0.8,
                        popup=name_map.get(pt,pt)).add_to(m)
        
        map_data = st_folium(m, width=800, height=600, key="map1")

# ==================== 页面2：3D地图 ====================
elif page.startswith("🌐"):
    st.markdown('<div class="main-header">🌐 3D地图视图</div>', unsafe_allow_html=True)
    st.markdown('<div class="sub-header">三维地形与航线可视化</div>', unsafe_allow_html=True)
    
    col1, col2 = st.columns([3, 1])
    with col2:
        st.markdown("### 🎨 显示选项")
        show_p = st.checkbox("显示航点", True)
        show_o = st.checkbox("显示障碍物", True)
        show_r = st.checkbox("显示航线", True)
        h3d = st.number_input("飞行高度(m)", value=st.session_state.flight_height, step=5.0, key="h3d")
    
    with col1:
        try:
            import pydeck as pdk
            points = []
            if show_p:
                points.append({"lng": st.session_state.point_a[0], "lat": st.session_state.point_a[1],
                              "alt": 0, "name": "起点A", "color": [0,255,0]})
                points.append({"lng": st.session_state.point_b[0], "lat": st.session_state.point_b[1],
                              "alt": h3d, "name": "终点B", "color": [255,0,0]})
            if show_o:
                for obs in st.session_state.obstacles:
                    c = get_polygon_center(obs["polygon"])
                    if c:
                        points.append({"lng": c[0], "lat": c[1], "alt": obs["height"],
                                      "name": obs["name"], "color": [255,100,100]})
            
            path_data = []
            if show_r and 'planned_paths' in st.session_state:
                for pt, pp in st.session_state.planned_paths.items():
                    if pt == "distances": continue
                    path_data.append({"path": [[p[0],p[1],h3d] for p in pp], "name": pt,
                                     "color": [0,255,0] if pt=="optimal" else [100,100,255]})
            elif show_r:
                path_data.append({"path": [[st.session_state.point_a[0],st.session_state.point_a[1],h3d],
                                        [st.session_state.point_b[0],st.session_state.point_b[1],h3d]],
                                 "name": "AB连线", "color": [0,100,255]})
            
            layers = []
            if points:
                layers.append(pdk.Layer("ScatterplotLayer", data=points,
                    get_position=["lng","lat","alt"], get_color="color", get_radius=20, pickable=True))
                layers.append(pdk.Layer("TextLayer", data=points,
                    get_position=["lng","lat","alt"], get_text="name", get_size=14, get_color=[0,0,0]))
            if path_data:
                layers.append(pdk.Layer("PathLayer", data=path_data,
                    get_path="path", get_color="color", get_width=5, pickable=True))
            
            view = pdk.ViewState(longitude=CENTER_LNG, latitude=CENTER_LAT, zoom=16, pitch=45)
            r = pdk.Deck(layers=layers, initial_view_state=view,
                        tooltip={"text": "{name}\n高度: {alt}m"},
                        map_style="mapbox://styles/mapbox/satellite-v9")
            st.pydeck_chart(r)
        except Exception as e:
            st.warning(f"3D地图加载失败: {e}")
            st.info("请确保安装了pydeck并支持WebGL")

# ==================== 页面3：障碍物管理 ====================
elif page.startswith("🚧"):
    st.markdown('<div class="main-header">🚧 障碍物管理</div>', unsafe_allow_html=True)
    
    col1, col2 = st.columns([2, 1])
    
    with col2:
        st.markdown("### ➕ 添加障碍物")
        obs_name = st.text_input("名称", value="新障碍物")
        obs_height = st.number_input("高度(m)", 1.0, 200.0, 30.0, 5.0)
        obs_safety = st.number_input("安全半径(m)", 5.0, 100.0, 15.0, 5.0)
        
        st.markdown("**多边形顶点 (JSON)**")
        poly_json = st.text_area("输入顶点 [[lng,lat],...]",
            value='[[118.7500, 32.2335],[118.7505, 32.2335],[118.7505, 32.2340],[118.7500, 32.2340]]',
            height=100)
        
        if st.button("✅ 添加", use_container_width=True):
            try:
                poly = json.loads(poly_json)
                if isinstance(poly, list) and len(poly) >= 3:
                    new_id = len(st.session_state.obstacles)
                    st.session_state.obstacles.append({
                        "id": new_id, "name": obs_name, "polygon": poly,
                        "height": obs_height, "safety_radius": obs_safety
                    })
                    save_obstacles(st.session_state.obstacles)
                    st.success(f"已添加: {obs_name}")
                    st.rerun()
                else:
                    st.error("至少需要3个顶点")
            except json.JSONDecodeError:
                st.error("JSON格式错误")
        
        st.markdown("---")
        st.markdown("### ⚡ 快速添加")
        if st.button("添加示例障碍1", use_container_width=True):
            poly = [[118.7498,32.2338],[118.7503,32.2338],[118.7503,32.2343],[118.7498,32.2343]]
            st.session_state.obstacles.append({
                "id": len(st.session_state.obstacles), "name": "教学楼A",
                "polygon": poly, "height": 35.0, "safety_radius": 20.0})
            save_obstacles(st.session_state.obstacles)
            st.rerun()
        
        if st.button("添加示例障碍2", use_container_width=True):
            poly = [[118.7506,32.2348],[118.7510,32.2348],[118.7510,32.2352],[118.7506,32.2352]]
            st.session_state.obstacles.append({
                "id": len(st.session_state.obstacles), "name": "实验楼B",
                "polygon": poly, "height": 25.0, "safety_radius": 15.0})
            save_obstacles(st.session_state.obstacles)
            st.rerun()
        
        if st.button("🗑️ 全部清除", use_container_width=True):
            st.session_state.obstacles = []
            save_obstacles([])
            st.rerun()
    
    with col1:
        st.markdown("### 🗺️ 障碍物地图")
        m = folium.Map(location=[CENTER_LAT, CENTER_LNG], zoom_start=16)
        folium.TileLayer(
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Esri", name="卫星地图").add_to(m)
        folium.LayerControl().add_to(m)
        
        for obs in st.session_state.obstacles:
            poly = [[p[1],p[0]] for p in obs["polygon"]]
            folium.Polygon(locations=poly, color="red", weight=2, fill=True,
                fillColor="red", fillOpacity=0.3,
                popup=f"{obs['name']}<br>高:{obs['height']}m").add_to(m)
            c = get_polygon_center(obs["polygon"])
            if c:
                folium.Marker(location=[c[1], c[0]],
                    icon=folium.DivIcon(html=f'<div style="font-size:12px;color:red;font-weight:bold;background:white;padding:2px;">{obs["name"]}</div>')).add_to(m)
        
        st_folium(m, width=700, height=400, key="map_obs")
        
        st.markdown("### 📋 障碍物列表")
        if st.session_state.obstacles:
            data = []
            for o in st.session_state.obstacles:
                c = get_polygon_center(o["polygon"])
                data.append({"ID": o["id"], "名称": o["name"], "高度(m)": o["height"],
                            "安全半径(m)": o["safety_radius"], "顶点数": len(o["polygon"]),
                            "中心经度": round(c[0],6) if c else "-",
                            "中心纬度": round(c[1],6) if c else "-"})
            st.table(data)
            
            del_id = st.number_input("删除ID", 0, len(st.session_state.obstacles)-1, 0)
            if st.button("🗑️ 删除"):
                st.session_state.obstacles = [o for o in st.session_state.obstacles if o["id"] != del_id]
                for i, o in enumerate(st.session_state.obstacles):
                    o["id"] = i
                save_obstacles(st.session_state.obstacles)
                st.rerun()
        else:
            st.info("暂无障碍物")

# ==================== 页面4：坐标转换工具 ====================
elif page.startswith("🔄"):
    st.markdown('<div class="main-header">🔄 坐标转换工具</div>', unsafe_allow_html=True)
    st.markdown('<div class="sub-header">WGS-84 / GCJ-02 坐标系转换</div>', unsafe_allow_html=True)
    
    col1, col2 = st.columns(2)
    with col1:
        st.markdown("**输入坐标**")
        in_lng = st.number_input("经度", value=CENTER_LNG, format="%.6f")
        in_lat = st.number_input("纬度", value=CENTER_LAT, format="%.6f")
        in_sys = st.selectbox("输入坐标系", ["WGS-84", "GCJ-02"])
        out_sys = st.selectbox("输出坐标系", ["GCJ-02", "WGS-84"])
        
        if st.button("🔄 转换"):
            rlng, rlat = in_lng, in_lat
            if in_sys == "WGS-84" and out_sys == "GCJ-02":
                rlng, rlat = wgs84_to_gcj02(in_lng, in_lat)
            elif in_sys == "GCJ-02" and out_sys == "WGS-84":
                rlng, rlat = gcj02_to_wgs84(in_lng, in_lat)
            st.session_state.ct_result = (rlng, rlat)
    
    with col2:
        st.markdown("**转换结果**")
        if 'ct_result' in st.session_state:
            rlng, rlat = st.session_state.ct_result
            st.success(f"经度: {rlng:.6f}")
            st.success(f"纬度: {rlat:.6f}")
            m = folium.Map(location=[rlat, rlng], zoom_start=17)
            folium.Marker([rlat, rlng]).add_to(m)
            st_folium(m, width=400, height=300, key="map_ct")
    
    st.markdown("---")
    st.markdown("### 📏 距离计算")
    d1, d2 = st.columns(2)
    with d1:
        d1lng = st.number_input("点1经度", value=CENTER_LNG, format="%.6f", key="d1l")
        d1lat = st.number_input("点1纬度", value=CENTER_LAT, format="%.6f", key="d1a")
    with d2:
        d2lng = st.number_input("点2经度", value=CENTER_LNG+0.002, format="%.6f", key="d2l")
        d2lat = st.number_input("点2纬度", value=CENTER_LAT+0.002, format="%.6f", key="d2a")
    
    if st.button("📏 计算"):
        d = haversine_distance(d1lng, d1lat, d2lng, d2lat)
        st.success(f"距离: {d:.2f} 米 ({d/1000:.3f} 公里)")

# ==================== 页面5：飞行监控 ====================
elif page.startswith("📊"):
    st.markdown('<div class="main-header">📊 飞行监控</div>', unsafe_allow_html=True)
    
    m = st.session_state.mavlink
    f = st.session_state.flight
    
    # 状态标签
    status_map = {
        "idle": ("待飞", "gray"),
        "flying": ("飞行中", "green"),
        "paused": ("已暂停", "orange"),
        "landing": ("降落中", "blue"),
        "emergency": ("紧急！", "red"),
    }
    status_text, status_color = status_map.get(f["status"], ("未知", "gray"))
    st.markdown(f'<div style="text-align:center;margin-bottom:10px;">'
                f'<span style="background:{status_color};color:white;padding:6px 20px;'
                f'border-radius:20px;font-weight:bold;">● {status_text}</span> '
                f'<span style="margin-left:20px;">模式: <b>{f["mode"]}</b></span> '
                f'<span style="margin-left:20px;">航段: <b>{f["path_index"]+1}/{max(len(f["path"])-1,1)}</b></span>'
                f'</div>', unsafe_allow_html=True)
    
    # 控制按钮区
    c0, c1, c2, c3, c4, c5, c6 = st.columns([1,1,1,1,1,1,2])
    with c0:
        if st.button("▶️ 起飞", use_container_width=True):
            if f["status"] == "idle":
                # 从A点起飞，使用最优路径或直飞
                if 'planned_paths' in st.session_state and st.session_state.planned_paths.get("optimal"):
                    f["path"] = st.session_state.planned_paths["optimal"]
                else:
                    f["path"] = [st.session_state.point_a, st.session_state.point_b]
                f["path_index"] = 0
                f["progress"] = 0.0
                f["status"] = "flying"
                f["mode"] = "AUTO"
                f["start_time"] = time.time()
                f["distance_flown"] = 0
                m["lon"] = f["path"][0][0]
                m["lat"] = f["path"][0][1]
                m["alt"] = st.session_state.flight_height
                st.rerun()
    with c1:
        if st.button("⏸️ 暂停", use_container_width=True):
            if f["status"] == "flying":
                f["status"] = "paused"
                st.rerun()
    with c2:
        if st.button("▶️ 继续", use_container_width=True):
            if f["status"] == "paused":
                f["status"] = "flying"
                st.rerun()
    with c3:
        if st.button("⬇️ 降落", use_container_width=True):
            if f["status"] in ("flying", "paused"):
                f["status"] = "landing"
                st.rerun()
    with c4:
        if st.button("🚨 紧急", use_container_width=True):
            f["status"] = "emergency"
            f["mode"] = "RTL"
            st.rerun()
    with c5:
        if st.button("🔄 复位", use_container_width=True):
            f["status"] = "idle"
            f["mode"] = "AUTO"
            f["path_index"] = 0
            f["progress"] = 0.0
            f["distance_flown"] = 0
            f["path"] = []
            m["lon"] = st.session_state.point_a[0]
            m["lat"] = st.session_state.point_a[1]
            m["alt"] = st.session_state.flight_height
            m["groundspeed"] = 0
            m["airspeed"] = 0
            m["heading"] = 0
            m["throttle"] = 0
            m["climb_rate"] = 0
            st.rerun()
    with c6:
        st.session_state.flight_speed = st.slider("飞行速度 (m/s)", 1.0, 20.0, st.session_state.flight_speed, 0.5)
    
    # 更新飞行状态
    if f["status"] == "flying":
        update_flight_position(f, m, speed=st.session_state.flight_speed)
        m["groundspeed"] = st.session_state.flight_speed + random.uniform(-0.3, 0.3)
        m["airspeed"] = st.session_state.flight_speed + random.uniform(-0.5, 0.5)
        m["throttle"] = random.randint(50, 70)
        m["battery_remaining"] = max(5, m["battery_remaining"] - 0.02)
        f["flight_time"] = int(time.time() - f["start_time"]) if f["start_time"] else 0
    elif f["status"] == "paused":
        m["groundspeed"] = 0
        m["airspeed"] = 0
        m["throttle"] = 30
        m["climb_rate"] = 0
    elif f["status"] == "landing":
        set_flight_altitude(m, 5.0, rate=1.5)
        if m["alt"] <= 5.1:
            f["status"] = "idle"
            m["groundspeed"] = 0
            m["airspeed"] = 0
            m["throttle"] = 0
    elif f["status"] == "emergency":
        m["groundspeed"] = 0
        m["airspeed"] = 0
        m["throttle"] = 0
        set_flight_altitude(m, 2.0, rate=3.0)
    
    # 飞行参数显示
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("地速", f"{m['groundspeed']:.1f} m/s")
        st.metric("空速", f"{m['airspeed']:.1f} m/s")
    with c2:
        st.metric("相对高度", f"{m['alt']:.1f} m")
        st.metric("爬升率", f"{m['climb_rate']:.1f} m/s")
    with c3:
        st.metric("航向", f"{m['heading']}°")
        st.metric("油门", f"{m['throttle']}%")
    with c4:
        st.metric("电压", f"{m['battery_voltage']:.1f}V")
        st.metric("电量", f"{int(m['battery_remaining'])}%")
    
    st.markdown("---")
    
    c1, c2 = st.columns(2)
    with c1:
        st.markdown("### 📍 位置信息")
        st.write(f"**纬度**: {m['lat']:.6f}°")
        st.write(f"**经度**: {m['lon']:.6f}°")
        st.write(f"**海拔**: {m['alt']:.1f} m")
        st.write(f"**飞行时间**: {f['flight_time']} 秒")
        st.write(f"**已飞距离**: {f['distance_flown']:.1f} m")
    with c2:
        st.markdown("### 🔄 姿态信息")
        st.write(f"**横滚**: {math.degrees(m['roll']):.1f}°")
        st.write(f"**俯仰**: {math.degrees(m['pitch']):.1f}°")
        st.write(f"**偏航**: {math.degrees(m['yaw']):.1f}°")
    
    st.markdown("---")
    st.markdown("### 🛰️ GPS状态")
    g1, g2, g3 = st.columns(3)
    with g1: st.metric("可见卫星", m['satellites'])
    with g2: st.metric("HDOP", "1.2")
    with g3: st.write("**质量**: 良好" if m['satellites'] >= 8 else "一般")
    
    st.markdown("---")
    st.markdown("### 🗺️ 实时位置")
    mmap = folium.Map(location=[m['lat'], m['lon']], zoom_start=17)
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri", name="卫星地图").add_to(mmap)
    folium.LayerControl().add_to(mmap)
    
    # 绘制航线
    if f["path"]:
        folium.PolyLine(locations=[[p[1],p[0]] for p in f["path"]],
            color="blue", weight=2, opacity=0.5, dash_array="5,5").add_to(mmap)
        # 起点终点
        folium.Marker([f["path"][0][1], f["path"][0][0]],
            popup="起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(mmap)
        folium.Marker([f["path"][-1][1], f["path"][-1][0]],
            popup="终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(mmap)
    
    # 无人机位置
    folium.Marker(location=[m['lat'], m['lon']],
        popup=f"高度: {m['alt']:.1f}m<br>航向: {m['heading']}°<br>速度: {m['groundspeed']:.1f}m/s",
        icon=folium.Icon(color="blue", icon="plane", prefix="fa")).add_to(mmap)
    
    st_folium(mmap, width=800, height=450, key="map_flight")
    
    # 自动刷新（飞行中每0.8秒刷新一次）
    if f["status"] in ("flying", "landing", "emergency"):
        time.sleep(0.8)
        st.rerun()
    
    if st.button("🔄 手动刷新"):
        st.rerun()

# ==================== 页面6：通信链路 ====================
elif page.startswith("📡"):
    st.markdown('<div class="main-header">📡 通信链路展示</div>', unsafe_allow_html=True)
    
    update_comm()
    
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.markdown("### 🌐 网络拓扑图")
        
        # 尝试graphviz
        try:
            import graphviz
            dot = graphviz.Digraph()
            dot.attr(rankdir='LR')
            dot.attr('node', shape='box', style='rounded,filled')
            for nid, node in COMM_NODES.items():
                color = '#90EE90' if node['status']=='online' else '#FFB6C1'
                dot.node(nid, f"{node['name']}\n{node['ip']}", fillcolor=color)
            for lid, link in COMM_LINKS.items():
                style = 'solid' if link['status']=='active' else 'dashed'
                color = 'green' if link['status']=='active' else 'gray'
                dot.edge(link['source'], link['target'],
                        label=f"{link['protocol']}\n{link['latency']}ms", style=style, color=color)
            st.graphviz_chart(dot.source)
        except Exception:
            st.info("拓扑图（表格展示）")
            tdata = []
            for lid, link in COMM_LINKS.items():
                tdata.append({"链路": f"{link['source']}→{link['target']}",
                             "协议": link['protocol'], "延迟(ms)": link['latency'],
                             "丢包(%)": round(link['packet_loss'],2), "状态": link['status']})
            st.table(tdata)
    
    with col2:
        st.markdown("### 📊 节点状态")
        for nid, node in COMM_NODES.items():
            sc = "status-online" if node['status']=='online' else "status-offline"
            st.markdown(f"""
            <div class="metric-card">
                <b>{node['name']}</b><br>
                IP: {node['ip']}:{node['port']}<br>
                状态: <span class="{sc}">{node['status']}</span>
            </div>
            """, unsafe_allow_html=True)
        
        st.markdown("---")
        st.markdown("### 📈 链路质量")
        for lid, link in COMM_LINKS.items():
            if link['latency'] < 10 and link['packet_loss'] < 0.5:
                q, c = "优秀", "green"
            elif link['latency'] < 20 and link['packet_loss'] < 1.0:
                q, c = "良好", "orange"
            else:
                q, c = "一般", "red"
            st.markdown(f"""
            <div class="metric-card">
                <b>{link['source']} ↔ {link['target']}</b><br>
                延迟: {link['latency']}ms | 丢包: {link['packet_loss']:.1f}%<br>
                质量: <span style="color:{c}">{q}</span>
            </div>
            """, unsafe_allow_html=True)
    
    if st.button("🔄 刷新"):
        st.rerun()

# ==================== 页面7：MAVLink数据流 ====================
elif page.startswith("📻"):
    st.markdown('<div class="main-header">📻 MAVLink数据流</div>', unsafe_allow_html=True)
    
    for _ in range(5):
        update_mavlink(st.session_state.mavlink)
    m = st.session_state.mavlink
    
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.markdown("### 📨 实时报文")
        for msg in reversed(m["history"][-30:]):
            if msg["type"] == "HEARTBEAT":
                icon, cls = "💓", "log-success"
            elif msg["type"] == "GPS_RAW_INT":
                icon, cls = "🛰️", "log-info"
            elif msg["type"] == "ATTITUDE":
                icon, cls = "🔄", "log-info"
            elif msg["type"] == "BATTERY_STATUS":
                icon, cls = "🔋", "log-warning"
            else:
                icon, cls = "📄", "log-info"
            st.markdown(f"""
            <div style="font-family:monospace;font-size:12px;margin:2px 0;">
                <span style="color:#888;">[{msg['time']}]</span>
                {icon} <span class="{cls}">{msg['text']}</span>
            </div>
            """, unsafe_allow_html=True)
    
    with col2:
        st.markdown("### 📊 消息统计")
        if m["stats"]:
            import pandas as pd
            df = pd.DataFrame([{"类型": k, "数量": v} for k, v in m["stats"].items()])
            st.bar_chart(df.set_index("类型"))
        
        st.markdown("---")
        st.markdown("### 📋 当前状态")
        st.write(f"**心跳序号**: {m['seq']}")
        st.write(f"**纬度**: {m['lat']:.6f}°")
        st.write(f"**经度**: {m['lon']:.6f}°")
        st.write(f"**高度**: {m['alt']:.1f}m")
        st.write(f"**电量**: {int(m['battery_remaining'])}%")
        st.write(f"**消息总数**: {m['message_count']}")
        
        st.markdown("---")
        st.markdown("### 🔌 连接状态")
        st.markdown('<span class="status-online">● 已连接 (模拟)</span>', unsafe_allow_html=True)
    
    if st.button("🔄 刷新"):
        st.rerun()
