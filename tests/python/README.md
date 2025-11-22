# Data Replay

## 0. 准备环境 (TODO)
### 0.1 piper_ros (ROS2 功能包)
### 0.2 robo_orchard_lab (python库)

## 1. 修改数据集路径

在 `data_replay.py` main函数中将下面的data路径换成你保存 (meta.mdb, depth.mdb, image.mdb, index.mdb) 的实际路径

```python
dataset = RoboTwinLmdbDataset("/data/", valid_transforms)
```

## 2. 运行 data_replay.py

```bash
python data_replay.py
```

## 3. 运行 piper urdf 可视化启动文件

### 3.1 进入之前下载的 piper_ros
```bash
cd /root/ros2_ws/piper_ros
```

### 3.2 激活 ROS2 环境
```bash
source install/setup.bash
```

### 3.3 启动可视化脚本
```bash
ros2 launch piper_description my.launch.py
```
