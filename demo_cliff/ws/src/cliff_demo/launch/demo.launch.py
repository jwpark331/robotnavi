"""한 회차: Gazebo + Nav2(AMCL 위치추정 + 주행) + 조건별 costmap filter + 시나리오 실행기.

    ros2 launch cliff_demo demo.launch.py scenario:=S2 condition:=P1 seed:=1 out_dir:=/tmp/run gui:=false

조건 (geometry.CONDITIONS):
  P0  SLAM 지도만                       — filter 없음
  P1  + Keepout                         — keepout_filter (운영자가 그린 마스크)
  P2  + 깊이카메라 단일 프레임          — keepout_filter + cliff_filter(cliff_detector mode=frame)
  P3  + 깊이카메라 베이즈 필터 (제안)   — keepout_filter + cliff_filter(cliff_detector mode=bayes)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from cliff_demo import geometry as G


class NoAlias(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def setup(context):
    pkg = get_package_share_directory("cliff_demo")
    bringup = get_package_share_directory("nav2_bringup")
    a = {k: context.launch_configurations[k]
         for k in ("scenario", "condition", "seed", "out_dir", "gui", "timeout", "push_dx", "mat_dropout", "void_dropout", "floor_mu", "pitch_k",
                   "keepout_ring", "max_vel")}
    scen, cond = G.SCENARIOS[a["scenario"]], G.CONDITIONS[a["condition"]]
    out = a["out_dir"]
    os.makedirs(out, exist_ok=True)

    p = yaml.safe_load(open(os.path.join(pkg, "params", "nav2.yaml")))
    max_vel = float(a["max_vel"])
    if max_vel > 0:                                     # v3 속도 시험: 최고 속도 세 값만 바꾼다 (가감속 한계 · 비용지도 · 카메라 상수는 그대로)
        fp = p["controller_server"]["ros__parameters"]["FollowPath"]
        fp["max_vel_x"] = fp["max_speed_xy"] = max_vel
        p["velocity_smoother"]["ros__parameters"]["max_velocity"][0] = max_vel
    filters = (["keepout_filter"] if cond["keepout"] else []) + (["cliff_filter"] if cond["cliff"] else [])
    for cm in ("local_costmap", "global_costmap"):
        if filters:                       # rcl 파라미터 파서는 빈 리스트를 못 읽는다
            p[cm][cm]["ros__parameters"]["filters"] = list(filters)
    ring = bool(cond.get("keepout_ring")) or a["keepout_ring"].lower() == "true"   # v2는 모든 방식이 띠 keepout
    mask = G.MASK_PREFIX + "keepout_" + "".join(scen["keepout"]) + ("_ring" if ring else "")
    p["filter_mask_server"]["ros__parameters"]["yaml_filename"] = os.path.join(pkg, "maps", mask + ".yaml")
    p["amcl"]["ros__parameters"]["initial_pose"].update(x=G.START[0], y=G.START[1], yaw=G.START[2])   # 월드 2 (월드 1은 같은 값)
    params = os.path.join(out, "nav2_params.yaml")
    with open(params, "w") as f:
        yaml.dump(p, f, Dumper=NoAlias, allow_unicode=True, sort_keys=False)   # rcl 파서는 YAML 별칭(&id)을 못 읽는다

    world = os.path.join(pkg, "worlds", G.WORLD_FILE)
    mu = float(a["floor_mu"])
    if mu < 1.0 or os.environ.get("CLIFF_W2_MAT"):      # 젖은 바닥: 계단 A 앞 모퉁이만 마찰을 낮춘 월드를 이 회차용으로 만든다 (H8 탐색: 옮긴 매트도 회차용 월드로)
        from cliff_demo.make_world import world as make_world
        world = os.path.join(out, "world.sdf")
        with open(world, "w") as f:
            f.write(make_world(mu if mu < 1.0 else None))

    nav_args = {"use_sim_time": "true", "params_file": params, "autostart": "true", "use_composition": "False"}
    acts = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "sim.launch.py")),
                                 launch_arguments={"gui": a["gui"], "world": world, "ir": "true" if cond.get("ir") else "false",
                                                   "ir_plus": "true" if cond.get("plus") else "false",
                                                   "ir_rear": "true" if cond.get("ir_rear") else "false",
                                                   **({"spawn_x": str(G.START_S3J[0]), "spawn_y": str(G.START_S3J[1]),
                                                       "spawn_yaw": str(G.START_S3J[2])} if scen.get("init_err") else {}),
                                                   "mat_dropout": a["mat_dropout"], "seed": a["seed"],
                                                   "ir_log": os.path.join(out, "ir.jsonl") if cond.get("ir") else ""}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "localization_launch.py")),
                                 launch_arguments={**nav_args, "map": os.path.join(pkg, "maps", G.MAP_NAME + ".yaml")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation_launch.py")),
                                 launch_arguments=nav_args.items()),
    ]
    managed = []
    if cond["keepout"]:
        acts += [Node(package="nav2_map_server", executable="map_server", name="filter_mask_server", output="log",
                      parameters=[params]),
                 Node(package="nav2_map_server", executable="costmap_filter_info_server",
                      name="costmap_filter_info_server", output="log", parameters=[params])]
        managed += ["filter_mask_server", "costmap_filter_info_server"]
    if cond["cliff"]:
        acts += [Node(package="nav2_map_server", executable="costmap_filter_info_server",
                      name="cliff_filter_info_server", output="log", parameters=[params]),
                 Node(package="cliff_demo", executable="cliff_detector", output="log",
                      parameters=[{"use_sim_time": True, "mode": cond["cliff"], "frame": cond["frame"], "artifacts": True,
                                   "seed": int(a["seed"]), "mat_dropout": float(a["mat_dropout"]),
                                   "void_dropout": float(a["void_dropout"]), "caution_cost": int(cond.get("caution", 0)),
                                   "pitch_k_deg": float(a["pitch_k"]),
                                   **({"memory": cond["memory"]} if "memory" in cond else {}),
                                   **({"nan_neutral": True} if cond.get("nan_neutral") else {}),
                                   **({"nan_neutral": True, "view_gate": True, "ground_plane": True, "sigma_phi_deg": 1.0,
                                       "three_state": True, "ir_memory": True} if cond.get("plus") else {}),
                                   "log_path": os.path.join(out, "cliff.jsonl")}])]
        managed += ["cliff_filter_info_server"]
    if managed:
        acts.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                         name="lifecycle_manager_filters", output="log",
                         parameters=[{"use_sim_time": True, "autostart": True, "node_names": managed}]))
    runner = Node(package="cliff_demo", executable="scenario", output="screen",
                  parameters=[{"use_sim_time": True, "scenario": a["scenario"], "condition": a["condition"],
                               "seed": int(a["seed"]), "out_dir": out, "timeout": float(a["timeout"]),
                               "push_dx": float(a["push_dx"]), "mat_dropout": float(a["mat_dropout"]),
                               "void_dropout": float(a["void_dropout"]), "floor_mu": float(a["floor_mu"]),
                               "pitch_k": float(a["pitch_k"]), "keepout_ring": ring, "max_vel": max_vel}])
    acts += [runner, RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())]))]
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="S1"),
        DeclareLaunchArgument("condition", default_value="P0"),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("out_dir", default_value="/tmp/cliff_run"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("timeout", default_value="150"),
        DeclareLaunchArgument("push_dx", default_value="-1", description="S3 밀림 거리 고정 [m] (음수면 시드로 0.8–1.0)"),
        DeclareLaunchArgument("mat_dropout", default_value=str(G.MAT_DROPOUT), description="검은 매트 결측률"),
        DeclareLaunchArgument("pitch_k", default_value="0.0", description="실험: 가감속 연동 기울기 오차 [deg/(m/s²)] (0 = 끔)"),
        DeclareLaunchArgument("floor_mu", default_value=str(G.FLOOR_MU), description="젖은 바닥: 계단 A 앞 모퉁이 마찰계수 (1 = 기본 월드)"),
        DeclareLaunchArgument("keepout_ring", default_value="false", description="v2: 모든 방식의 운영자 keepout을 비용 띠 마스크로"),
        DeclareLaunchArgument("void_dropout", default_value=str(G.VOID_DROPOUT), description="검은 계단: 계단 구멍 광선 결측률"),
        DeclareLaunchArgument("max_vel", default_value="0", description="v3: 최고 속도 [m/s] (0 = nav2.yaml 그대로)"),
        OpaqueFunction(function=setup),
    ])
