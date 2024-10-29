-- Cartographerの設定ファイル。各パラメータはSLAMの動作を調整します。

include "map_builder.lua"  -- マップ構築に必要なモジュールを含む
include "trajectory_builder.lua"  -- 軌道（Trajectory）構築に必要なモジュールを含む

options = {
  -- 基本設定
  map_builder = MAP_BUILDER,  -- マップビルダー設定の選択
  trajectory_builder = TRAJECTORY_BUILDER,  -- 軌道ビルダー設定の選択
  map_frame = "map",  -- マップフレームの名前
  tracking_frame = "base_link",  -- ロボットの基準フレーム（位置追跡用）
  published_frame = "base_footprint",  -- 公開されるフレーム
  odom_frame = "odom",  -- オドメトリフレームの名前
  provide_odom_frame = true,  -- オドメトリフレームを提供するかどうか
  publish_frame_projected_to_2d = false,  -- フレームを2Dに投影するか
  use_pose_extrapolator = true,  -- 姿勢推定の外挿器を使用するか
  use_odometry = true,  -- オドメトリを使用するか
  use_nav_sat = false,  -- ナビゲーション用衛星データを使用するか
  use_landmarks = false,  -- ランドマークを使用するか
  num_laser_scans = 1,  -- 使用するレーザースキャンの数
  num_multi_echo_laser_scans = 0,  -- 使用するマルチエコーレーザースキャンの数
  num_subdivisions_per_laser_scan = 1,  -- レーザースキャンの細分化数
  num_point_clouds = 0,  -- 使用するポイントクラウドの数
  lookup_transform_timeout_sec = 0.5,  -- トランスフォームの検索タイムアウト（秒）。短くすると失敗しやすく、長くすると遅延が増加します。
  submap_publish_period_sec = 0.5,  -- サブマップの公開頻度（秒）。短くすると最新情報が得られやすいが、通信量が増加します。
  pose_publish_period_sec = 5e-3,  -- 姿勢の公開頻度（秒）。頻度を高くすると追跡がスムーズだが、計算負荷が増えます。
  trajectory_publish_period_sec = 30e-3,  -- 軌道の公開頻度（秒）。高くするとリアルタイム性が向上しますが、CPU負荷が増加します。
  rangefinder_sampling_ratio = 1.0,  -- レンジファインダーのサンプリング割合。低くすると計算負荷が減少するが、精度が落ちる。
  odometry_sampling_ratio = 1.0,  -- オドメトリのサンプリング割合。同上。
  fixed_frame_pose_sampling_ratio = 1.0,  -- 固定フレームのサンプリング割合。同上。
  imu_sampling_ratio = 1.0,  -- IMUのサンプリング割合。低くすると負荷軽減。
  landmarks_sampling_ratio = 1.0,  -- ランドマークのサンプリング割合。同上。
}

-- ---------------------------------------------------------------------------------------------
-- GLOBAL SLAM設定
-- ---------------------------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 50  -- 90 この数のノードごとに大域SLAMを実行。小さくすると更新頻度が上がり精度が上がりますが、計算負荷が増します。
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 制約サンプリング割合。高くするとより多くの制約が生成され精度が上がるが、負荷が増加します。
POSE_GRAPH.constraint_builder.max_constraint_distance = 15  -- 制約を追加する際の最大距離。大きくすると精度が上がりますが、処理が重くなります。
POSE_GRAPH.constraint_builder.min_score = 0.55  -- スキャンマッチングの最小スコア。小さくすると許容範囲が広がりノイズも増えやすい。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  -- グローバルローカライゼーション時の最小スコア。小さくすると位置合わせが緩和され、ループ検出が増えることも。
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4  -- ループクロージャーの並進重み。大きくすると位置合わせが厳密になり、精度が上がるが負荷が増えます。
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- ループクロージャーの回転重み。大きくすると回転精度が上がりますが、処理時間も増加します。
POSE_GRAPH.constraint_builder.log_matches = true  -- ループクロージャーのデバッグログ出力。オンにするとデバッグが容易になります。

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7  -- 線形探索ウィンドウ。大きくすると精度が増しますが、計算時間が増加します。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30)  -- 角度探索ウィンドウ。大きくすると精度向上、計算負荷も増加します。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 10  -- 探索のグリッド数。深いほど精度が上がりますが、計算時間が長くなります。

POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20  -- 占有スペースの重み。高くすると障害物の認識が強化されますが、計算コストも増えます。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 90  -- 並進の重み。高くすると並進の精度が上がりますが、処理時間が増加。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 50  -- 回転の重み。大きくすると回転の精度が上がりますが、処理が重くなります。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true  -- Ceresソルバーで非単調ステップを使用。探索の柔軟性が増す。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- ソルバーの最大反復数。増やすと精度が上がりますが、計算コストも増します。
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 18  -- スレッド数。増やすと並列処理が行われ、速度が上がります。

POSE_GRAPH.matcher_translation_weight = 5e2  -- 並進の最適化重み。高くすると並進を厳密に扱いますが、計算負荷が増えます。
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- 回転の最適化重み。大きくすると回転の厳密さが上がり、負荷が増加します。

POSE_GRAPH.optimization_problem.huber_scale = 1e1  -- Huber損失のスケーリング値。高くすると誤差をなだらかに扱い、ノイズ耐性が増します。
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2  -- IMU加速度の重み。大きくすると加速度が重要視されるが、IMUが必須。
POSE_GRAPH.optimization_problem.rotation_weight = 1.4e3  -- IMU回転の重み。高くすると回転精度が上がるが、負荷も増加。
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- ローカルSLAM並進重み。大きくすると並進精度が上がり、負荷が増えます。
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5  -- ローカルSLAM回転重み。回転精度が向上しますが、負荷も増します。
POSE_GRAPH.optimization_problem.odometry_translation_weight = 4.0e4  -- オドメトリ並進重み。増やすと並進の精度が上がりますが、負荷が増えます。
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 3.0e4  -- オドメトリ回転重み。同上。
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1  -- 固定フレーム並進重み。固定フレームの精度を上げたい場合に増やします。
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2  -- 固定フレーム回転重み。同上。

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- 最適化問題の反復回数。高くすると精度が上がりますが、負荷も増加します。
POSE_GRAPH.max_num_final_iterations = 200  -- 最後の最適化の最大反復数。反復数が多いと精度が上がりますが、計算負荷が増えます。
POSE_GRAPH.global_sampling_ratio = 0.003  -- グローバルサンプリング割合。低くすると計算負荷が減りますが、精度が落ちる可能性があります。
POSE_GRAPH.global_constraint_search_after_n_seconds = 10  -- 制約探索のタイミング（秒）。短くすると制約が早く追加されますが、計算負荷が増えます。

-- ---------------------------------------------------------------------------------------------
-- LOCAL SLAM設定
-- ---------------------------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.use_imu_data = false  -- IMUデータの使用。trueにすると、IMUの影響で安定するが、IMUが必須。
TRAJECTORY_BUILDER_2D.min_range = 0.5  -- レンジファインダーの最小距離。短くすると近距離のデータも取得できますが、ノイズが増える可能性も。
TRAJECTORY_BUILDER_2D.max_range = 200  -- レンジファインダーの最大距離。長くすると視野が広がりますが、遠距離データに対する信頼性が低下することも。
TRAJECTORY_BUILDER_2D.min_z = -0.1  -- z座標の最小値。値を増やすと床のノイズを減らせますが、全体のスキャン範囲が狭まります。
TRAJECTORY_BUILDER_2D.max_z = 30  -- z座標の最大値。大きくすると高い障害物を検知できますが、処理が重くなる可能性も。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 累積するスキャン数。大きくすると平滑化されますが、リアルタイム性が低下します。
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025  -- ボクセルフィルタのサイズ。小さくすると詳細を保持しますが、処理が重くなります。

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5  -- 適応ボクセルフィルタの最大長さ。大きくすると情報量が増えますが、負荷も増えます。

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- オンラインスキャンマッチングの使用。trueにすると、精度が上がりますが負荷が増します。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 線形探索ウィンドウ。大きくすると精度が上がりますが、計算時間が増加します。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20)  -- 角度探索ウィンドウ。大きくすると探索範囲が増えますが、負荷も増加します。
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 90  -- 並進重み。高くすると並進の精度が上がりますが、負荷も増えます。
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50  -- 回転重み。高くすると回転精度が上がりますが、処理が重くなります。

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5  -- モーションフィルタの時間閾値。小さくすると更新頻度が上がりますが、負荷が増加します。
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2  -- モーションフィルタの距離閾値。小さくすると精度が上がりますが、負荷も増加します。

-- ---------------------------------------------------------------------------------------------
-- その他の重要な設定
-- ---------------------------------------------------------------------------------------------

MAP_BUILDER.use_trajectory_builder_2d = true  -- 2D軌道ビルダーを使用
MAP_BUILDER.num_background_threads = 18  -- バックグラウンドスレッド数。多くすると処理が並列化され速度が向上しますが、リソースが多く必要。

return options  -- オプション設定の返却
