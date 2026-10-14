#!/usr/bin/env python3
"""
魚眼ステレオ画像を「左右が平行に並んだピンホールカメラ」の画像に補正するノード。

入力 : 左右の魚眼画像（トピック名はキャリブレーションファイルの rostopic から読む）
出力 : /stereo/left/image_rect,  /stereo/left/camera_info
       /stereo/right/image_rect, /stereo/right/camera_info
       TF: cam0 -> cam0_rect（補正後の仮想カメラの向き）
"""
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import message_filters
from tf2_ros import StaticTransformBroadcaster


# ------------------------------------------------------------
# 補助関数
# ------------------------------------------------------------
def load_kalibr_yaml(path):
    """OpenVINS形式のyamlを読む。先頭の %YAML:1.0 はPythonのyamlで読めないので飛ばす。"""
    with open(path) as f:
        text = ''.join(line for line in f if not line.startswith('%YAML'))
    return yaml.safe_load(text)


def parse_camera(cam):
    """yamlの1カメラ分の情報を、OpenCVが使う形（numpy配列）に変換する。"""
    if cam.get('distortion_model') != 'equidistant':
        raise ValueError('このノードは equidistant（魚眼）モデル専用です')

    fu, fv, cu, cv_ = cam['intrinsics']
    K = np.array([[fu, 0.0, cu],
                  [0.0, fv, cv_],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array(cam['distortion_coeffs'], dtype=np.float64).reshape(4, 1)
    size = tuple(int(v) for v in cam['resolution'])  # (幅, 高さ)

    # T_imu_cam: カメラ座標の点 -> IMU座標の点 に変換する4x4行列
    if 'T_imu_cam' in cam:
        T_imu_cam = np.array(cam['T_imu_cam'], dtype=np.float64)
    else:  # Kalibrの生出力は逆向き（T_cam_imu）なので逆行列をとる
        T_imu_cam = np.linalg.inv(np.array(cam['T_cam_imu'], dtype=np.float64))

    return K, D, size, T_imu_cam, cam.get('rostopic')


def rotation_to_quaternion(R):
    """回転行列 -> クォータニオン (x, y, z, w)。"""
    R = np.ascontiguousarray(R, dtype=np.float64)
    rvec, _ = cv2.Rodrigues(R)
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    axis = rvec.flatten() / angle
    s = np.sin(angle / 2.0)
    return axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0)


def make_camera_info(P, size, frame_id):
    """補正後の仮想カメラの camera_info を作る（歪みはもう無いので d は全部0）。"""
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = size
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5
    info.k = P[:3, :3].flatten().tolist()
    info.r = np.eye(3).flatten().tolist()
    info.p = P.flatten().tolist()     # 右カメラはここの4列目に基線長の情報が入る
    return info


def make_tf(parent, child, R, t, stamp):
    """parent から見た child の位置・向きを表すTFを作る。"""
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
        (float(v) for v in t)
    qx, qy, qz, qw = rotation_to_quaternion(R)
    tf.transform.rotation.x, tf.transform.rotation.y = float(qx), float(qy)
    tf.transform.rotation.z, tf.transform.rotation.w = float(qz), float(qw)
    return tf


def to_mono8(img):
    """画像を8bitモノクロにそろえる。"""
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.dtype == np.uint16:
        img = (img >> 8).astype(np.uint8)
    return img


# ------------------------------------------------------------
# ノード本体
# ------------------------------------------------------------
class StereoRectifyNode(Node):

    def __init__(self):
        super().__init__('stereo_rectify')

        # ---- パラメータ（launchファイルやコマンドラインから変更できる） ----
        self.declare_parameter('calib_file', '')
        self.declare_parameter('left_cam', 'cam0')
        self.declare_parameter('right_cam', 'cam1')
        self.declare_parameter('balance', 0.0)       # 0:余白なし(狭い) 〜 1:視野広め(余白あり)
        self.declare_parameter('fov_scale', 1.0)
        self.declare_parameter('output_prefix', '/stereo')
        self.declare_parameter('imu_frame', 'imu')
        self.declare_parameter('cam_frame', 'cam0')
        self.declare_parameter('rect_frame', 'cam0_rect')
        self.declare_parameter('publish_imu_to_cam_tf', False)  # imu->cam0 のTFが無いときだけ true

        p = lambda name: self.get_parameter(name).value
        calib_file = p('calib_file')
        if not calib_file:
            raise RuntimeError('パラメータ calib_file を指定してください')
        self.rect_frame = p('rect_frame')

        # ---- 1. キャリブレーションを読む ----
        calib = load_kalibr_yaml(calib_file)
        K0, D0, size, T_imu_c0, topic0 = parse_camera(calib[p('left_cam')])
        K1, D1, _, T_imu_c1, topic1 = parse_camera(calib[p('right_cam')])
        self.size = size

        # ---- 2. 左右カメラの相対位置（cam0の点 -> cam1の点） ----
        T_c1_c0 = np.linalg.inv(T_imu_c1) @ T_imu_c0
        R = np.ascontiguousarray(T_c1_c0[:3, :3])
        T = np.ascontiguousarray(T_c1_c0[:3, 3]).reshape(3, 1)
        self.get_logger().info(f'基線長: {np.linalg.norm(T) * 100:.2f} cm')

        # ---- 3. 補正パラメータと補正マップ（起動時に1回だけ計算） ----
        R1, R2, P1, P2, _Q = cv2.fisheye.stereoRectify(
            K0, D0, K1, D1, size, R, T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            balance=float(p('balance')), fov_scale=float(p('fov_scale')))
        self.map0 = cv2.fisheye.initUndistortRectifyMap(K0, D0, R1, P1, size, cv2.CV_16SC2)
        self.map1 = cv2.fisheye.initUndistortRectifyMap(K1, D1, R2, P2, size, cv2.CV_16SC2)
        self.get_logger().info(f'補正後の焦点距離: {P1[0, 0]:.1f} px')

        # camera_info は中身が変わらないので先に作っておく（左右とも同じ座標系）
        self.info0 = make_camera_info(P1, size, self.rect_frame)
        self.info1 = make_camera_info(P2, size, self.rect_frame)

        # ---- TF: cam0 -> cam0_rect（R1 は cam0 -> rect なので、TFには転置を使う） ----
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        tfs = [make_tf(p('cam_frame'), self.rect_frame, R1.T, [0.0, 0.0, 0.0], now)]
        if p('publish_imu_to_cam_tf'):
            tfs.append(make_tf(p('imu_frame'), p('cam_frame'),
                               T_imu_c0[:3, :3], T_imu_c0[:3, 3], now))
        self.tf_broadcaster.sendTransform(tfs)

        # ---- 4. publisher / subscriber ----
        prefix = p('output_prefix')
        self.pub_img0 = self.create_publisher(Image, f'{prefix}/left/image_rect', 10)
        self.pub_img1 = self.create_publisher(Image, f'{prefix}/right/image_rect', 10)
        self.pub_info0 = self.create_publisher(CameraInfo, f'{prefix}/left/camera_info', 10)
        self.pub_info1 = self.create_publisher(CameraInfo, f'{prefix}/right/camera_info', 10)

        self.bridge = CvBridge()
        self.sub0 = message_filters.Subscriber(self, Image, topic0)
        self.sub1 = message_filters.Subscriber(self, Image, topic1)
        # 左右の画像を「ほぼ同じ時刻（差0.01秒以内）」のペアにしてから callback を呼ぶ
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub0, self.sub1], queue_size=10, slop=0.01)
        self.sync.registerCallback(self.callback)

        self.logged_encoding = False
        self.get_logger().info(f'入力: {topic0}, {topic1} / 出力: {prefix}/...')

    # ---- 5〜7. 毎フレームの処理 ----
    def callback(self, msg0, msg1):
        if not self.logged_encoding:
            self.get_logger().info(f'入力画像のエンコーディング: {msg0.encoding}')
            self.logged_encoding = True

        img0 = to_mono8(self.bridge.imgmsg_to_cv2(msg0, 'passthrough'))
        img1 = to_mono8(self.bridge.imgmsg_to_cv2(msg1, 'passthrough'))

        rect0 = cv2.remap(img0, self.map0[0], self.map0[1], cv2.INTER_LINEAR)
        rect1 = cv2.remap(img1, self.map1[0], self.map1[1], cv2.INTER_LINEAR)

        # タイムスタンプは元画像のものを使う（左右で完全に一致させるため左の時刻にそろえる）
        stamp = msg0.header.stamp
        for rect, pub_img, info, pub_info in (
                (rect0, self.pub_img0, self.info0, self.pub_info0),
                (rect1, self.pub_img1, self.info1, self.pub_info1)):
            out = self.bridge.cv2_to_imgmsg(rect, encoding='mono8')
            out.header.stamp = stamp
            out.header.frame_id = self.rect_frame
            info.header.stamp = stamp
            pub_img.publish(out)
            pub_info.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = StereoRectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()