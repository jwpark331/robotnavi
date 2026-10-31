"""geometry.py → worlds/city_hall.world (Gazebo Classic SDF).

    python3 -m cliff_demo.make_world <출력 경로>
"""
import math
import sys

from cliff_demo import geometry as G


def box(name, cx, cy, cz, sx, sy, sz, rgba, yaw=0.0, collide=True, mu=None):
    surf = (f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>' if mu is not None else "")
    col = (f'<collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>{surf}</collision>'
           if collide else "")
    r, g, b, a = rgba
    return (f'<link name="{name}"><pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 {yaw:.5f}</pose>{col}'
            f'<visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
            f'<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular></material></visual></link>')


def _touches_void(c, a0, a1, side):
    """판의 한 변(좌표 c, 구간 a0–a1)이 계단 구멍과 맞닿는가. 맞닿으면 겹치지 않는다 (구멍을 덮지 않게)."""
    for x0, y0, x1, y1, _ in G.VOIDS.values():
        if side in ("x0", "x1"):
            edge = x1 if side == "x0" else x0
            if abs(c - edge) < 1e-9 and a0 < y1 and a1 > y0:
                return True
        else:
            edge = y1 if side == "y0" else y0
            if abs(c - edge) < 1e-9 and a0 < x1 and a1 > x0:
                return True
    return False


def building(wet_mu=None):
    """wet_mu가 있으면 G.WET_PATCH 구역을 따로 깔고 그 판만 마찰계수를 wet_mu로 둔다 ('젖은 바닥' 실험)."""
    links = []
    z0, z1 = G.WALL_Z
    k = 0
    for line in G.WALLS:
        for (x1, y1), (x2, y2) in zip(line[:-1], line[1:]):
            L = math.hypot(x2 - x1, y2 - y1)
            links.append(box(f"wall{k}", (x1 + x2) / 2, (y1 + y2) / 2, (z0 + z1) / 2, L + G.WALL_T, G.WALL_T, z1 - z0,
                             (0.55, 0.55, 0.58, 1), math.atan2(y2 - y1, x2 - x1)))
            k += 1
    rects = [(r, None) for r in G.slabs(cut_wet=wet_mu is not None)] + ([(G.WET_PATCH, wet_mu)] if wet_mu is not None else [])
    for i, ((x0, y0, x1, y1), mu) in enumerate(rects):
        # 이웃 판과 맞닿는 변은 2 cm씩 겹친다 (이음매 옆면을 묻어 바퀴가 걸리지 않게). 계단 구멍 쪽 변은 그대로.
        ex0 = x0 - (0.02 if not _touches_void(x0, y0, y1, "x0") and x0 > G.FLOOR_RECT[0] else 0.0)
        ex1 = x1 + (0.02 if not _touches_void(x1, y0, y1, "x1") and x1 < G.FLOOR_RECT[2] else 0.0)
        ey0 = y0 - (0.02 if not _touches_void(y0, x0, x1, "y0") and y0 > G.FLOOR_RECT[1] else 0.0)
        ey1 = y1 + (0.02 if not _touches_void(y1, x0, x1, "y1") and y1 < G.FLOOR_RECT[3] else 0.0)
        x0, x1, y0, y1 = ex0, ex1, ey0, ey1
        links.append(box(f"slab{i}", (x0 + x1) / 2, (y0 + y1) / 2, -G.SLAB_T / 2, x1 - x0, y1 - y0, G.SLAB_T,
                         (0.80, 0.79, 0.76, 1) if mu is None else (0.62, 0.74, 0.86, 1), mu=mu))
    for name, (x0, y0, x1, y1, d) in G.VOIDS.items():
        run = (x1 - x0) if d == "+x" else (y1 - y0)
        n = int(math.ceil(run / G.TREAD - 1e-9))
        for i in range(n):
            top = -G.RISER * (i + 1)
            a = i * G.TREAD
            b = min(run, (i + 1) * G.TREAD) if i < n - 1 else run
            shade = 0.62 - 0.04 * i
            if d == "+x":
                links.append(box(f"stair{name}{i}", x0 + (a + b) / 2, (y0 + y1) / 2, (top + G.LOWER_Z) / 2,
                                 b - a, y1 - y0, top - G.LOWER_Z, (shade, 0.42, 0.36, 1)))
            else:
                links.append(box(f"stair{name}{i}", (x0 + x1) / 2, y0 + (a + b) / 2, (top + G.LOWER_Z) / 2,
                                 x1 - x0, b - a, top - G.LOWER_Z, (shade, 0.42, 0.36, 1)))
    x0, y0, x1, y1 = G.DARK_MAT
    links.append(box("dark_mat", (x0 + x1) / 2, (y0 + y1) / 2, 0.0005, x1 - x0, y1 - y0, 0.001, (0.05, 0.05, 0.06, 1),
                     collide=False))   # 3 mm 상자는 Gazebo에서 바퀴가 걸려 멈춘다
    x0, y0, x1, y1 = G.SUN_PATCH
    links.append(box("sun_patch", (x0 + x1) / 2, (y0 + y1) / 2, 0.0005, x1 - x0, y1 - y0, 0.001, (1.0, 0.93, 0.62, 1),
                     collide=False))
    return f'<model name="city_hall"><static>true</static>{"".join(links)}</model>'


def overhead_camera():
    o = G.OVERHEAD
    return f"""<model name="overhead_cam"><static>true</static>
  <link name="link"><pose>{o['x']} {o['y']} {o['z']} 0 {math.pi / 2:.6f} {math.pi / 2:.6f}</pose>
    <sensor name="overhead" type="camera"><update_rate>4</update_rate><always_on>true</always_on>
      <camera><horizontal_fov>{o['hfov']}</horizontal_fov>
        <image><width>{o['w']}</width><height>{o['h']}</height><format>R8G8B8</format></image>
        <clip><near>1.0</near><far>40</far></clip></camera>
      <plugin name="overhead_cam" filename="libgazebo_ros_camera.so">
        <ros><namespace>/overhead</namespace></ros><camera_name>overhead</camera_name><frame_name>overhead</frame_name>
      </plugin>
    </sensor>
  </link></model>"""


def world(wet_mu=None):
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
<world name="city_hall">
  <physics type="ode"><max_step_size>0.002</max_step_size><real_time_update_rate>500</real_time_update_rate>
    <real_time_factor>1</real_time_factor></physics>
  <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.75 0.78 0.82 1</background><shadows>false</shadows></scene>
  <gui><camera name="user_camera"><pose>5.75 0.8 13.5 0 1.12 1.5708</pose></camera></gui>
  <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose>
    <diffuse>0.85 0.85 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -1</direction></light>
  <model name="lower_floor"><static>true</static><pose>5.75 6.9 {G.LOWER_Z - 0.05} 0 0 0</pose>
    <link name="l"><collision name="c"><geometry><box><size>30 30 0.1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>30 30 0.1</size></box></geometry>
        <material><ambient>0.3 0.3 0.32 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse></material></visual></link></model>
  {building(wet_mu)}
  {overhead_camera()}
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros><namespace>/</namespace></ros><update_rate>20.0</update_rate>
  </plugin>
</world>
</sdf>
"""


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else G.WORLD_FILE
    with open(out, "w") as f:
        f.write(world())
    print(out)
