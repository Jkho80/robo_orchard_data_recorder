import cv2
import geometry_msgs.msg
import numpy as np
import rclpy
import tf2_ros
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from robo_orchard_lab.dataset.robotwin.robotwin_lmdb_dataset import (
    RoboTwinLmdbDataset,
)
from sensor_msgs.msg import CameraInfo, Image, JointState

# from robo_orchard_lab.projects.sem.robotwin.config_sem_robotwin import build_transforms


config = dict(
    hist_steps=1,
    pred_steps=64,
    chunk_size=8,
    embed_dims=256,
    with_depth=True,
    with_depth_loss=True,
    min_depth=0.01,
    max_depth=1.2,
    num_depth=128,
    batch_size=20,
    max_step=int(1e5),
    step_log_freq=25,
    save_step_freq=5000,
    num_workers=8,
    lr=1e-4,
    checkpoint="path/to/model.safetensors",
    bert_checkpoint="./ckpt/bert-base-uncased",
    data_path="/data/lmdb_dataset_empty_cup_place_2025_09_09",
    urdf="./piper_description_dualarm.urdf",
    multi_task=False,
    task_names=["empty_cup_place"],
)

config.update(
    calibration=dict(
        right={
            "position": [
                -0.06467007281411405,
                0.017047962629280943,
                0.05807131939724936,
            ],
            "orientation": [
                -0.14645412055790097,
                0.14440677147612505,
                -0.6642611908788346,
                0.7186479981296034,
            ],
        },
        left={
            "position": [
                -0.0708666828613833,
                0.009987417684242016,
                0.05807131939724936,
            ],
            "orientation": [
                -0.14505635828798102,
                0.15000731441384266,
                -0.6895885882124545,
                0.6934868690535743,
            ],
        },
        middle={
            "position": [
                0.070783568385050412,
                -0.277,
                0.5173197227547938,
            ],
            "orientation": [
                0.9238795325,
                0,
                0.3826834324,
                0,
            ],
        },
    ),
    img_channel_flip=True,
    scale_shift=[
        [1.478021398, 0.10237011399999996],
        [1.453678296, 1.4043815520000003],
        [1.553963852, -1.5014923],
        [1.86969153, -0.0010728060000000372],
        [1.3381379620000002, -0.012585846000000012],
        [3.086157592, -0.06803160000000008],
        [0.03857, 0.036329999999999994],
        [1.478021398, 0.10237011399999996],
        [1.453678296, 1.4043815520000003],
        [1.553963852, -1.5014923],
        [1.86969153, -0.0010728060000000372],
        [1.3381379620000002, -0.012585846000000012],
        [3.086157592, -0.06803160000000008],
        [0.03857, 0.036329999999999994],
    ],
    kinematics_config=dict(
        left_arm_joint_id=[0, 1, 2, 3, 4, 5],
        right_arm_joint_id=[8, 9, 10, 11, 12, 13],
        left_arm_link_keys=[
            "fl_link1",
            "fl_link2",
            "fl_link3",
            "fl_link4",
            "fl_link5",
            "fl_link6",
        ],
        right_arm_link_keys=[
            "fr_link1",
            "fr_link2",
            "fr_link3",
            "fr_link4",
            "fr_link5",
            "fr_link6",
        ],
        left_finger_keys=["fl_link7"],
        right_finger_keys=["fr_link7"],
    ),
)


def build_transforms(config, calibration=None):
    from robo_orchard_lab.dataset.robotwin.transforms import (
        AddItems,
        AddScaleShift,
        CalibrationToExtrinsic,
        ConvertDataType,
        DualArmKinematics,
        GetProjectionMat,
        IdentityTransform,
        ImageChannelFlip,
        ItemSelection,
        JointStateNoise,
        Resize,
        SimpleStateSampling,
        ToTensor,
    )

    add_data_relative_items = dict(
        type=AddItems,
        T_base2world=config.get(
            "T_base2world",
            [
                # ALOHA
                # [0, -1, 0, 0],
                # [1, 0, 0, -0.65],
                # [0, 0, 1, 0],
                # [0, 0, 0, 1],
                # PiPER
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1.0],
            ],
        ),
    )
    state_sampling = dict(
        type=SimpleStateSampling,
        hist_steps=config["hist_steps"],
        pred_steps=config["pred_steps"],
    )
    resize = dict(
        type=Resize,
        dst_wh=(320, 256),
        dst_intrinsic=[
            [358.6422, 0.0000, 160.0000, 0.0000],
            [0.0000, 382.5517, 128.0000, 0.0000],
            [0.0000, 0.0000, 1.0000, 0.0000],
            [0.0000, 0.0000, 0.0000, 1.0000],
        ],
    )
    if config.get("img_channel_flip"):
        img_channel_flip = dict(
            type=ImageChannelFlip, output_channel=[2, 1, 0]
        )
    else:
        img_channel_flip = dict(type=IdentityTransform)
    to_tensor = dict(type=ToTensor)
    projection_mat = dict(type=GetProjectionMat, target_coordinate="base")
    convert_dtype = dict(
        type=ConvertDataType,
        convert_map=dict(
            imgs="float32",
            depths="float32",
            image_wh="float32",
            projection_mat="float32",
            embodiedment_mat="float32",
        ),
    )
    scale_shift_list = config.get(
        "scale_shift",
        [
            [0.82115489, 0.00280333],
            [1.26673863, 1.26673677],
            [1.38083194, 1.38080194],
            [0.94487381, -0.94485653],
            [0.13566405, 0.05423572],
            [0.90747011, 0.05119371],
            [0.425, 0.575],
            [0.82115489, 0.00280333],
            [1.26673863, 1.26673677],
            [1.38083194, 1.38080194],
            [0.94487381, -0.94485653],
            [0.13566405, 0.05423572],
            [0.90747011, 0.05119371],
            [0.425, 0.575],
        ],  # defalut scale shift for robotwin2.0 example single task
    )

    kinematics = dict(
        type=DualArmKinematics,
        urdf=config["urdf"],
        **config.get("kinematics_config", {}),
    )
    scale_shift = dict(
        type=AddScaleShift,
        scale_shift=scale_shift_list,
    )
    joint_state_noise = dict(
        type=JointStateNoise,
        noise_range=[
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.0, 0.0],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.02, 0.02],
            [-0.0, 0.0],
        ],
    )
    item_selection = dict(
        type=ItemSelection,
        keys=[
            "imgs",
            "depths",
            "image_wh",
            "projection_mat",
            "embodiedment_mat",
            "hist_robot_state",
            "pred_robot_state",
            "joint_relative_pos",
            "joint_scale_shift",
            "kinematics",
            "text",
            "uuid",
            "intrinsic",
            "T_world2cam",
            "joint_state",
            "ee_state",
            "hist_joint_state",
            "pred_joint_state",
        ],
    )

    if calibration is not None:
        calib_to_ext = dict(
            type=CalibrationToExtrinsic,
            urdf=config["urdf"],  # public1
            calibration=calibration,
            **config.get("kinematics_config", {}),
        )
    else:
        calib_to_ext = dict(type=IdentityTransform)

    train_transforms = [
        add_data_relative_items,
        state_sampling,
        resize,
        img_channel_flip,
        to_tensor,
        calib_to_ext,
        projection_mat,
        scale_shift,
        joint_state_noise,
        convert_dtype,
        kinematics,
        item_selection,
    ]
    val_transforms = [
        add_data_relative_items,
        state_sampling,
        resize,
        img_channel_flip,
        to_tensor,
        calib_to_ext,
        projection_mat,
        scale_shift,
        convert_dtype,
        kinematics,
        item_selection,
    ]
    return train_transforms, val_transforms


def build_dataset(config, lazy_init=False):
    from robo_orchard_lab.dataset.robotwin.robotwin_lmdb_dataset import (
        RoboTwinLmdbDataset,
    )

    train_transforms, val_transforms = build_transforms(config)
    train_dataset = RoboTwinLmdbDataset(
        paths=config["data_path"],
        task_names=config["task_names"],
        lazy_init=lazy_init,
        transforms=train_transforms,
    )

    val_dataset = RoboTwinLmdbDataset(
        paths=None,
        task_names=config["task_names"],
        lazy_init=True,
        transforms=val_transforms,
        instruction_keys=("seen",),
    )
    return train_dataset, val_dataset


def invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rotation_matrix_to_rpy(R):
    """R: 3x3 rotation matrix -> roll, pitch, yaw (radians)"""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    return np.degrees([roll, pitch, yaw])


class RoboTwinPublisher(Node):
    def __init__(self, dataset):
        super().__init__("robo_twin_publisher")

        self.dataset = dataset  # 数据集
        print("dataset len:", len(self.dataset))
        self.current_index = 0  # 当前发布的样本索引

        self.image_pubs = [None] * 4  # 字典存储每个相机的图像发布器
        self.depth_pubs = [None] * 4  # 字典存储每个相机的深度图发布器
        self.camera_info_pubs = [None] * 4  # 字典存储每个相机的相机信息发布器

        self.joint_state_pub = self.create_publisher(
            JointState, "joint_states", 10
        )

        self.br = tf2_ros.TransformBroadcaster(self)

        # OpenCV与ROS2消息转换桥
        self.bridge = CvBridge()

        self.setup_publishers()

        self.joint_states = []
        self.ee_states = None

        self.got_js = False

        self.joint_state_idx = 0

        self.selected_indices = np.linspace(0, 250, 10, dtype=int)

        self.cal = dict(
            middle={
                "position": [
                    -0.010783568385050412,
                    -0.2559182030838615,
                    0.5173197227547938,
                ],
                "orientation": [
                    -0.6344593881273598,
                    0.6670669773214551,
                    -0.2848079166270871,
                    0.2671467447131103,
                ],
            },
            left={
                "position": [-0.0693628, 0.04614798, 0.02938585],
                "orientation": [-0.6916, 0.6979, -0.1345, 0.1458],
            },
            right={
                "position": [-0.0693628, 0.04614798, 0.02938585],
                "orientation": [-0.6916, 0.6979, -0.1345, 0.1458],
            },
        )

        self.cam_base = dict(
            middle="base_link",
            left="fl_link6",
            right="fr_link6",
        )

        self.vel = [10.0] * 16
        self.effort = [4.0] * 16

    def setup_publishers(self):
        names = ["front", "left", "right", "under"]
        # print(len(names))
        for cam_idx in [0, 1, 2, 3]:
            self.image_pubs[cam_idx] = self.create_publisher(
                Image, f"/cam_{names[cam_idx]}/image", 10
            )
            # 创建每个相机的深度图发布器
            self.depth_pubs[cam_idx] = self.create_publisher(
                Image, f"/cam_{names[cam_idx]}/depth", 10
            )
            # 创建每个相机的内参发布器
            self.camera_info_pubs[cam_idx] = self.create_publisher(
                CameraInfo, f"/cam_{names[cam_idx]}/camera_info", 10
            )

    def publish_data(self):
        # 如果数据集已经发布完毕，停止发布
        if self.current_index >= len(self.dataset):
            self.get_logger().info("All data has been published.")
            return False

        # 获取当前样本数据
        data = self.dataset[self.current_index]
        # print(data)
        if not self.got_js:
            for k, v in data.items():
                if isinstance(v, (np.ndarray, torch.Tensor)):
                    print(f"data[{k}].shape:", v.shape)
                else:
                    print(f"data[{k}]:", v)
            print(f"data[text]: {data['text']}")

        joint_state = data["hist_joint_state"][0].cpu().numpy()
        joint_state_msg = JointState()
        joint_state_msg.name = [
            "fl_joint1",
            "fl_joint2",
            "fl_joint3",
            "fl_joint4",
            "fl_joint5",
            "fl_joint6",
            "fl_joint7",
            "fl_joint8",
            "fr_joint1",
            "fr_joint2",
            "fr_joint3",
            "fr_joint4",
            "fr_joint5",
            "fr_joint6",
            "fr_joint7",
            "fr_joint8",
        ]
        arr = np.zeros(16)
        arr[0:6] = joint_state[0:6]
        arr[6] = joint_state[6] / 2
        arr[7] = -joint_state[6] / 2
        arr[8:14] = joint_state[7:13]
        arr[14] = joint_state[13] / 2
        arr[15] = -joint_state[13] / 2
        arr2 = [0.0] * 16
        joint_state_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )  # 设置时间戳

        joint_state_msg.header.frame_id = "base_link"
        joint_state_msg.position = arr.tolist()
        joint_state_msg.velocity = self.vel
        joint_state_msg.effort = self.effort

        self.joint_state_pub.publish(joint_state_msg)

        self.got_js = True

        T_world2cam = data["T_world2cam"]
        T_cam2world = []
        for T in T_world2cam:
            T_inv = invert_transform(T)
            T_cam2world.append(T_inv)

        T_cam2world = np.array(T_cam2world)

        # Loop to publish the transforms for each camera
        for i in range(3):  # Three cameras
            T = T_cam2world[i]
            R = T[:3, :3]
            t = T[:3, 3]

            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = f"camera{i}"

            transform.transform.translation.x = float(t[0])
            transform.transform.translation.y = float(t[1])
            transform.transform.translation.z = float(t[2])

            # Convert rotation matrix to quaternion
            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)
            transform.transform.rotation.x = float(qx)
            transform.transform.rotation.y = float(qy)
            transform.transform.rotation.z = float(qz)
            transform.transform.rotation.w = float(qw)

            self.br.sendTransform(transform)

        for i in self.cal.keys():  # Three cameras
            T = self.cal[i]
            x, y, z = T["position"]
            qx, qy, qz, qw = T["orientation"]

            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.cam_base[i]
            transform.child_frame_id = f"camera_{i}"

            transform.transform.translation.x = float(x)
            transform.transform.translation.y = float(y)
            transform.transform.translation.z = float(z)

            transform.transform.rotation.x = float(qx)
            transform.transform.rotation.y = float(qy)
            transform.transform.rotation.z = float(qz)
            transform.transform.rotation.w = float(qw)

        name = ["middle", "left", "right", "under"]

        for camera_idx in range(len(data["imgs"])):
            image = data["imgs"][camera_idx]  # 获取当前相机图像
            depth = data["depths"][camera_idx]  # 获取当前相机深度图
            intrinsic = data["intrinsic"][camera_idx]  # 获取当前相机内参
            world2cam = data["T_world2cam"][camera_idx]
            base2world = np.array(
                [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            )

            # 发布图像
            try:
                projection_mat = intrinsic @ world2cam @ base2world
                vis_img = self.dataset.get_vis_imgs(
                    torch.unsqueeze(image, axis=0),
                    torch.unsqueeze(projection_mat, axis=0),
                    data["hist_robot_state"][-1],
                    ee_indices=(6, 13),
                )
                # Now you can display the image using OpenCV
                cv2.imshow(name[camera_idx], vis_img)
                cv2.waitKey(1)  # Wait for a key press to close the window
                ros_image = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")

                self.image_pubs[camera_idx].publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Failed to convert image: {e}")

            # 发布深度图
            try:
                depth_normalized = cv2.normalize(
                    depth.cpu().numpy(), None, 0, 255, cv2.NORM_MINMAX
                )  # 归一化到0~255
                depth_uint8 = depth_normalized.astype(np.uint8)  # 转成 uint8

                ros_depth = self.bridge.cv2_to_imgmsg(
                    depth_uint8, encoding="mono8"
                )
                self.depth_pubs[camera_idx].publish(ros_depth)
            except Exception as e:
                self.get_logger().error(f"Failed to convert depth image: {e}")
        self.current_index += 1
        if self.current_index % 100 == 0:
            self.get_logger().info(
                f"Published data sample {self.current_index}/{len(self.dataset)}. JointState.Shape :{len(self.joint_states)}"
            )
        return True

    def rotation_matrix_to_quaternion(self, R):
        """Convert a rotation matrix to a quaternion"""
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                qw = (R[2, 1] - R[1, 2]) / S
                qx = 0.25 * S
                qy = (R[0, 1] + R[1, 0]) / S
                qz = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                qw = (R[0, 2] - R[2, 0]) / S
                qx = (R[0, 1] + R[1, 0]) / S
                qy = 0.25 * S
                qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                qw = (R[1, 0] - R[0, 1]) / S
                qx = (R[0, 2] + R[2, 0]) / S
                qy = (R[1, 2] + R[2, 1]) / S
                qz = 0.25 * S
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)

    train_transforms, valid_transforms = build_transforms(config)
    dataset = RoboTwinLmdbDataset("/data/", valid_transforms)

    print(
        f"dataset length: {dataset.__len__()}, "
        f"number of episode: {dataset.num_episode}"
    )

    publisher = RoboTwinPublisher(dataset)

    while rclpy.ok():
        if not publisher.publish_data():
            break

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
