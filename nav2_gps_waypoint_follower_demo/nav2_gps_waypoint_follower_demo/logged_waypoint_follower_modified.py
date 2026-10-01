import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
import math
import threading 
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class YamlWaypointParser:
    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        [{"latitude": 緯度, "longitude": 経度, "action": タスク名, "value": 引数}] のリストを返す
        """
        waypoints = []
        for wp in self.wps_dict["waypoints"]:
            # attribute項目が辞書型形式（actionとvalue）になっているかチェック
            attr = wp.get("attribute", {})
            action = attr.get("action", "none")
            value = attr.get("value", 0)

            waypoints.append({
                "latitude": wp["latitude"],
                "longitude": wp["longitude"],
                "action": action,
                "value": value
            })
        return waypoints


class GpsWpCommander(Node):
    """
    最新のGPS（/gps/fix）を購読し、現在地からの相対距離を毎回到着時に計算して走るノード
    """
    def __init__(self, wps_file_path):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        
        # 現在のGPSデータを保持する変数
        self.current_gps = None

        # /gps/fix トピックの購読設定
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.get_logger().info('Subscribed to /gps/fix')

    def gps_callback(self, msg: NavSatFix):
        """
        GPSトピックを受信したときに現在地を更新するコールバック
        """
        # 有効なデータが来ているかチェック
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.current_gps = msg

    def calculate_relative_xy(self, target_lat, target_lon):
        """
        現在のGPS位置からターゲットGPS位置への相対的なX(東)とY(北)の距離(メートル)を計算（ヒュベニの公式）
        """
        if self.current_gps is None:
            self.get_logger().error("Current GPS data is not available yet!")
            return None, None

        # 地球の赤道半径と極半径 (WGS84)
        A = 6378137.0
        B = 6356752.314245
        
        # 緯度経度をラジアンに変換
        lat1 = math.radians(self.current_gps.latitude)
        lon1 = math.radians(self.current_gps.longitude)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)

        # 差分
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # 平均緯度
        lat_ave = (lat1 + lat2) / 2.0

        # 第一離心率の2乗
        e2 = (A**2 - B**2) / (A**2)
        
        # 子午線曲率半径 (M) と 卯酉線曲率半径 (N)
        W = math.sqrt(1.0 - e2 * math.sin(lat_ave)**2)
        M = A * (1.0 - e2) / (W**3)
        N = A / W

        # ロボット中心（base_footprint）から見た相対座標
        # X軸を進行方向、Y軸を左方向と仮定するため、東(East)・北(North)のベクトルからNav2形式へ
        # 注意: Nav2のベースフレーム座標系にそのまま合わせるため、ここでは地図上の相対差分(メートル)を計算
        dy = dlat * M       # 北方向への距離 (メートル)
        dx = dlon * N * math.cos(lat_ave)  # 東方向への距離 (メートル)

        return dx, dy

    def start_wpf(self):
        """
        ウェイポイントを1つずつ逐次処理するメインループ
        """
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        wps = self.wp_parser.get_wps()

        self.get_logger().info(f'Loaded {len(wps)} GPS waypoints. Starting sequential calculation...')

        for i, wp in enumerate(wps):
            self.get_logger().info(f'--- Processing Waypoint {i+1}/{len(wps)} ---')

            # GPSのデータが届くまで待機
            while self.current_gps is None:
                self.get_logger().info("Waiting for first GPS fix...")
                rclpy.spin_once(self, timeout_sec=0.5)

            # 最新の情報を更新
            rclpy.spin_once(self, timeout_sec=0.1)

            # 1. 相対距離(dx, dy)を計算
            target_lat = wp["latitude"]
            target_lon = wp["longitude"]
            target_yaw = 0

            dx, dy = self.calculate_relative_xy(target_lat, target_lon)
            if dx is None:
                continue

            # 2. 【TFを利用】現在の odom -> base_link の位置（tf2_echo と同じ処理）を取得
            current_x = 0.0
            current_y = 0.0
            
            try:
                # 最新の座標変換を取得（タイムスタンプ 0 は最新のデータを意味します）
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    'map',          # ターゲット（基準にする親フレーム）
                    'base_link',     # ソース（ロボットの現在のフレーム）
                    now,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                current_x = trans.transform.translation.x
                current_y = trans.transform.translation.y
                self.get_logger().info(f"Retrieved current pose via TF (odom->base_link): X={current_x:.2f}, Y={current_y:.2f}")
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"Could not lookup TF: {e}. Using (0,0) as fallback.")

            # 3. ターゲットの座標を「odomフレーム」ベースで作成
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = rclpy.time.Time().to_msg()
            
            # TFから得た確実な現在位置に、GPSの相対距離を足し算
            target_pose.pose.position.x = current_x - dx
            target_pose.pose.position.y = current_y - dy
            target_pose.pose.position.z = 0.0

            self.get_logger().info(f"Calculated Odom Target : X={target_pose.pose.position.x:.2f}, Y={target_pose.pose.position.y:.2f}")

            # 4. ナビゲーション命令を発行
            self.navigator.goToPose(target_pose)

            # 5. 到着までループ監視
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)

            # 6. 到着ステータスの確認
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Waypoint {i+1} succeeded!')
                self.execute_custom_task(wp["action"], wp["value"])
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Waypoint {i+1} was canceled.')
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error(f'Waypoint {i+1} failed.')
            
            time.sleep(2.0)



        print("All GPS waypoints processed.")

    def execute_custom_task(self, action, value):
        """
        到着したウェイポイントのactionとvalue（引数）に応じてタスクを実行する関数
        """
        self.get_logger().info(f"Executing task: Action=[{action}], Value=[{value}]")

        if action == "wait":
            # 引数で渡された数値（秒）だけ動的に待機する
            wait_time = float(value)
            self.get_logger().info(f"[Task] Waiting for {wait_time} seconds...")
            time.sleep(wait_time)
            self.get_logger().info("Wait completed!")
            
        elif action == "spin":
            # 引数で渡されたラジアン角だけ旋回する
            spin_angle = float(value)
            self.get_logger().info(f"[Task] Spinning for {spin_angle} radians...")
            self.navigator.spin(spin_dist=spin_angle, time_allowance=15)
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)
            self.get_logger().info("Spin completed!")
            
        elif action == "none":
            self.get_logger().info("Moving to next waypoint.")
            
        else:
            self.get_logger().warn(f"Unknown action [{action}]. No action taken.")



def main():
    rclpy.init()

    default_yaml_file_path = os.path.join(
        get_package_share_directory("nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)

    # 【重要】GPSの購読（コールバック）を裏で常に動かすため、マルチスレッドでspinを開始
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(gps_wpf)
    
    # spinを別スレッドで開始
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # メインスレッドでナビゲーションのループを実行
    try:
        gps_wpf.start_wpf()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
