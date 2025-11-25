# RoboOrchard Data Recorder Toolkit

RoboOrchard 数据记录器是一套全面而强大的工具套件，专为机器人领域的高保真数据采集而设计，主要面向 ROS 2 生态系统。它提供了一个功能强大且可配置的后端，用于将数据记录为 MCAP 格式，并配备了一个用户友好的 Web 应用程序，用于交互式控制和可视化。

该项目由[Horizon Robotics](https://github.com/HorizonRobotics/RoboOrchardLab/)开发旨在应对复杂、真实的数据采集场景所面临的挑战，提供对数据流的精细控制，确保数据完整性，并简化操作人员的工作流程。

在此基础上有做了些许的改动。

RoboOrchard Data Recorder 从零搭建、部署、依赖包等介绍可查阅 [RoboOrchard Data Recorder Toolkit](README_origin.md) 

RoboOrchard Data Recorder 从零配置修改等介绍可查阅 [Challenge Cup](example/challenge_cup/) 

该文档是用于快速在已有平台上采集数据，无需从零搭建环境。

## 1. 启动 3个 相机

**打开 3个 终端**，用于启动三台 RealSense D435i（左右 + 中间）深度相机。
每台相机需要单独开一个终端启动。

### 1.1 进入 d435_ws2 目录 并激活环境 (三个都要)

```bash
cd /root/ros2_ws/d435_ws2
source install/setup.bash
```
确保环境正确加载后再运行相机驱动。

### 1.2 分别 运行下面三个命令

下面三个命令分别启动 middle / left / right 相机。
其中 `serial_no` 必须与现有的设备一致，用于确保 ROS 绑定到正确的物理相机。

``` bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:='agilex' \
    camera_name:='middle_camera' \
    align_depth.enable:=true \
    initial_reset:=true \
    diagnostics_period:=2.0 \
    serial_no:="'405622072486'"
```

``` bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:='agilex' \
    camera_name:='left_camera' \
    align_depth.enable:=true \
    initial_reset:=true \
    diagnostics_period:=2.0 \
    serial_no:="'401622072506'"
```

``` bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:='agilex' \
    camera_name:='right_camera' \
    align_depth.enable:=true \
    initial_reset:=true \
    diagnostics_period:=2.0 \
    serial_no:="'405622074908'"
```

> 说明：
> - align_depth.enable=true 用于对齐深度图和 RGB 图。
> - initial_reset=true 可避免相机异常状态导致的启动失败。
> - 若相机连接顺序变动，可通过 rs-enumerate-devices 查看序列号。


## 2. 启动 2个 图像压缩 (RGB + 深度)

这个工具使用内部编码器（image encoder）将图像压缩 color/image_raw → color/image_raw/compressed_data
这会极大减少 MCAP 文件体积。

**打开 2 个终端**，分别启动 RGB 编码器 和 深度编码器。

### 2.1 进入 robo_orchard_data_recorder/example/challenge_cup 目录
```bash
cd /root/ros2_ws/robo_orchard_data_recorder/example/
```

### 2.2 激活环境 (两个都要)
```bash
source /root/ros2_ws/robo_orchard_data_recorder/ros2_package/install/setup.bash
```

### 2.3 分别运行下面两个指令
**RGB编码器：**
```bash
bash challenge_cup/launch_image_encoder.sh
```
**深度编码器：**
```bash
bash challenge_cup/launch_depth_encoder.sh
```
> 说明：
> 这两个脚本会自动订阅之前启动的所有相机。

## 3. 启动 2个 机械臂
机械臂部分用于采集操作数据，包括关节状态、末端位姿等。
如果重启主机或重新插拔机械臂，必须重新识别 CAN 口。

**打开 1个 终端**

### 3.1 进入 piper_ros 目录并激活环境
```bash
cd /root/ros2_ws/piper_ros
source install/setup.bash
```

### 3.2 找到所有can口 (如果有重新 插拔机械臂 或者 重启主机)
```bash
bash find_all_can_port.sh
```
> 说明：该脚本会自动扫描已连接的机械臂并输出设备名及端口号，如 `can0, can1, ...`

### 3.3 激活所有 3.2 脚本找到的can口 (如果重启了 机械臂)
```bash
bash can_muti_activate.sh
```
> 说明：如果脚本内与上面的获取的设备及端口号不一致，可以根据上面脚本的输出内容来修改 `can_muti_activate.sh`

### 3.4 运行机械臂启动脚本
```bash
ros2 launch piper start_multi_arms.launch.py
```

## 4. 启动 静态 TF 发布器

这个工具需要统一各相机、机械臂、末端执行器之间的坐标关系。
启动现有的 ROS2 静态 TF 发布功能包用于发布固定变换。

**打开 1个 终端**

### 4.1 进入 robo_orchard_data_recorder/example/challenge_cup 目录
```bash
cd /root/ros2_ws/robo_orchard_data_recorder/example/
```

### 4.2 激活环境 (两个都要)
```bash
source /root/ros2_ws/robo_orchard_data_recorder/ros2_package/install/setup.bash
```

### 4.3 运行静态TF发布脚本

```bash
bash challenge_cup/launch_static_tf_publisher.sh
```



## 5. 启动 数据采集平台 （Web UI）
这是 RoboOrchard Data Recorder 的核心界面，用于：
- 设置采集用户
- 设置任务标签
- 配置 episode
- 启动 / 停止录制
- 实时查看多相机与机械臂数据流

**打开 1个 终端**

### 5.1 进入 robo_orchard_data_recorder 目录下 
```bash
cd /root/ros2_ws/robo_orchard_data_recorder/example
```

### 5.2 激活环境
```bash
source /root/ros2_ws/robo_orchard_data_recorder/ros2_package/install/setup.bash
```

### 5.3 启动 采集平台
```bash
bash challenge_cup/launch_app.sh
```
启动后会自动打开 Web 界面（通常运行在 http://localhost:8501 或 :8502）
![alt text](docs/_static/image.png)

### 5.4 设置本轮采集的用户名

> 说明：设置"采集者身份"，用于后续数据管理。

![alt text](docs/_static/image-1.png)

### 5.5 设置本轮采集的数据标签（非常重要）

标签会写入 `dataset/meta.mdb` 中，后续训练会根据此标签加载数据。

支持：
- 文本格式标签（如动作类型）
- 自定义字段
- 每次采集前都可修改

![alt text](docs/_static/image-2.png)

![alt text](docs/_static/image-3.png)

![alt text](docs/_static/image-4.png)

### 5.6 点击 "Confirm collecting config"

![alt text](docs/_static/image-5.png)

### 5.6 开始采集

- 点击 Start 记录一个 episode
- 点击 Stop 结束
- 每个 episode 会生成一个文件夹与一个 MCAP 文件

## 6. 数据集转换：MCAP -> LMDB
RoboOrchard Data Recorder原始数据为 MCAP，为了训练模型需要转换为 LMDB。

### 6.1 进入 robo_orchard_data_recorder/example/workspace 
```bash
cd /root/ros2_ws/robo_orchard_data_recorder/example/workspace
```

### 6.2 根据所需转换的实际数据集信息修改以下配置
```bash
# 参考 workspace/mcap_to_lmdb.sh
python3 -m robo_orchard_lab.dataset.horizon_manipulation.packer.mcap_lmdb_packer \
    --input_path "/root/ros2_ws/robo_orchard_data_recorder/example/workspace/challenge_cup/2025_10_27-11_17_36/data/JK/put_bottles_dustbin/episode*/*.mcap" \
    --output_path ./put_bottles_dustbin_1027_2 \
    --urdf piper_description_dualarm_180.urdf \
    --image_scale_factor 0.5
```

> 参数说明：
> - `--input_path` 要转换的 mcap 文件路径，可用 episode* 通配符
> - `--output_path` lmdb 输出路径
> - `--urdf` 指定机械臂模型，用于带入 camera extrinsic（将相机位姿写入数据集）
> - `--image_scale_factor` 相机长宽压缩比，保持0.5即可

### 6.3 运行转换脚本
```bash
bash mcap_to_lmdb.sh
```
转换完成后，会将所有 MCAP 数据集写成一个结构化的 LMDB 数据集。

> LMDB 数据集结构说明：
> - image.mdb 图像数据文件
> - depth.mdb 深度数据文件
> - meta.mdb 数据集元信息
> - lock.mdb 锁文件

## 7. 数据集可视化
我们有自己的可视化工具，用于检查转换结果和调试数据。

具体操作请查看 [tests/python/](tests/python/) 

> 可视化包含：
> - 图像与深度可视化
> - 机械臂轨迹回放
> - episode 汇总检查

---

# 📫 Editor:
 
ASC-RCS
- John Kho (22920232204199@stu.xmu.edu.cn)
- Zhengxuan L. (laizhengxuan@stu.xmu.edu.cn)
