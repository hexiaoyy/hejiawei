# 🚁 无人机飞行监控与航线规划系统

基于 Streamlit + Folium 的无人机可视化监控系统，支持地图显示、障碍物管理、航线规划、飞行监控和通信链路展示。

## ✨ 功能特性

### 📍 地图与航线规划
- OpenStreetMap 街道图 + ArcGIS 卫星图 双图层切换
- 起点A / 终点B 坐标设置（GCJ-02坐标系）
- 四种航线规划：直飞、左绕飞、右绕飞、最优路径（弧线）
- 路径长度对比与可视化显示

### 🌐 3D地图
- pydeck 三维地形可视化
- 航点、障碍物、航线的3D高度显示
- 卫星地形图层切换

### 🚧 障碍物管理
- 多边形圈选障碍物（JSON顶点输入）
- 障碍物高度与安全半径设置
- 记忆功能：自动保存到 `obstacles.json`，重启不丢失
- 快速添加示例障碍物
- 障碍物列表与删除管理

### 🔄 坐标转换工具
- WGS-84 ↔ GCJ-02 双向转换
- 两点间球面距离计算
- 地图点位验证

### 📊 飞行监控
- 实时飞行参数显示（地速、空速、高度、航向、油门、电量）
- 飞行器姿态（横滚/俯仰/偏航）
- GPS卫星状态
- 实时位置地图追踪

### 📡 通信链路
- GCS-OBC-FCU 通信拓扑图（Graphviz / 表格双模式）
- 节点状态监控（在线/离线）
- 链路延迟、丢包率、质量评级

### 📻 MAVLink数据流
- 实时报文显示（心跳包、GPS、姿态、电池、VFR_HUD）
- 消息类型统计图表
- 飞行器状态总览

## 📁 项目结构

```
uav_flight_system/
├── app.py              # 主程序（所有功能整合）
├── requirements.txt    # Python依赖
├── packages.txt        # 系统依赖（Streamlit Cloud用）
├── .streamlit/
│   └── config.toml     # Streamlit配置
└── README.md           # 说明文档
```

## 🚀 快速开始

### 本地运行

```bash
# 安装依赖
pip install -r requirements.txt

# 运行应用
streamlit run app.py
```

浏览器自动打开 `http://localhost:8501`

### Streamlit Cloud 部署

1. 将项目推送到 GitHub 仓库
2. 访问 [share.streamlit.io](https://share.streamlit.io)
3. 点击 **New app**，选择你的仓库
4. 主文件路径填 `app.py`
5. 点击 **Deploy**

> 注意：`packages.txt` 会告知 Streamlit Cloud 安装 graphviz 系统包。

## 🌍 坐标系说明

| 坐标系 | 用途 |
|--------|------|
| WGS-84 | GPS原始坐标，全球通用 |
| GCJ-02 | 中国国测局加密坐标，高德/腾讯地图使用 |
| OpenStreetMap | 使用 WGS-84 坐标 |
| ArcGIS卫星图 | 使用 WGS-84 坐标 |

系统内部地图坐标使用 GCJ-02 火星坐标，以匹配中国境内地图显示。

## ⚙️ 默认参数

| 参数 | 默认值 |
|------|--------|
| 地图中心 | 118.749413°E, 32.234097°N |
| 飞行高度 | 50 米 |
| 安全半径 | 15 米 |
| 障碍物高度 | 30 米 |

## 🧱 技术栈

- **前端框架**: Streamlit
- **地图渲染**: Folium + streamlit-folium
- **3D地图**: pydeck
- **拓扑图**: graphviz
- **数据处理**: pandas, numpy

## 📄 许可证

MIT License
