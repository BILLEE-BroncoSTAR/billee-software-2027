#!/usr/bin/env python3
"""Run one Nav2 scenario in the sim and record what happened, as one JSON line.

Needs a running sim with full Nav2 in worlds/nav_test.sdf, spawned at the scenario's lane:

    ros2 launch chassis_bringup sim_gz.launch.py gui:=false joy_control:=false \\
        nav_mode:=full world_file:=worlds/nav_test.sdf y:=20
    ros2 run navigation nav_scenario_test.py --scenario detour --out results.jsonl

(run_nav_tests.sh does both, for several scenarios and runs.)

Scenarios (lane y in nav_test.sdf; the goal is map (15, 0), straight ahead of the start):
    open     y=0    clear route
    detour   y=20   block across the line
    narrow   y=40   1.6 m gap in a wall
    blocked  y=60   goal inside a closed pen (expected: ABORTED, controlled)
    cancel   y=0    goal, then cancel after --cancel-after sim seconds

Measured, against Gazebo ground truth (/gz/odom) and the obs_* boxes in the world file:
    result               SUCCEEDED / ABORTED / CANCELED / REJECTED / TIMEOUT
    t_first_cmd_*        goal sent -> first non-zero command into diff_drive_controller
    t_total_*            goal sent -> result (sim and wall seconds; the sim runs < 1x real time)
    min_clearance_m      closest gap between the rover's footprint (unpadded, from the URDF) and
                         any obstacle; <= 0 is a collision
    final_error_m        distance from the goal at the end (truth)
    path_length_m        distance driven (truth)
    recoveries           Spin / BackUp / Wait / costmap-clear actions started by the behavior tree
    plans                global plans published (replanning at 1 Hz)
    cancel_*             (cancel) time from cancel to the last non-zero command
    nodes_not_active     Nav2 servers not 'active' afterwards (a crash, not a navigation failure)
"""

import argparse
import json
import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import BehaviorTreeLog
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter

LANES = {"open": 0.0, "detour": 20.0, "narrow": 40.0, "blocked": 60.0, "cancel": 0.0}
# Rover footprint in base_link without padding (nav2_params.yaml / URDF collision meshes).
FOOTPRINT = [(0.505, 0.391), (0.505, -0.391), (-0.615, -0.391), (-0.615, 0.391)]
NAV2_NODES = ["controller_server", "planner_server", "bt_navigator", "behavior_server",
              "velocity_smoother", "smoother_server", "waypoint_follower"]
RECOVERY_NODES = ("Spin", "BackUp", "Wait", "ClearLocalCostmap-Subtree",
                  "ClearGlobalCostmap-Subtree")
RESULTS = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED",
           TaskResult.FAILED: "ABORTED"}


def world_boxes(world):
    """Axis-aligned obs_* boxes in the world file as (xmin, xmax, ymin, ymax)."""
    boxes = []
    for model in ET.parse(world).getroot().iter("model"):
        if not model.get("name", "").startswith("obs_"):
            continue
        x, y = [float(v) for v in model.find("pose").text.split()[:2]]
        sx, sy = [float(v) for v in model.find(".//collision//box/size").text.split()[:2]]
        boxes.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
    return boxes


def seg_dist(p, q, a, b):
    """Shortest distance between segments pq and ab (2D)."""
    def pt_seg(c, u, v):
        dx, dy = v[0] - u[0], v[1] - u[1]
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((c[0] - u[0]) * dx + (c[1] - u[1]) * dy)
                                                   / (dx * dx + dy * dy)))
        return math.hypot(c[0] - u[0] - t * dx, c[1] - u[1] - t * dy)
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


def inside(pt, poly):
    """Point in convex polygon (any winding)."""
    sign = 0
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        c = (x2 - x1) * (pt[1] - y1) - (y2 - y1) * (pt[0] - x1)
        if c != 0:
            if sign and (c > 0) != (sign > 0):
                return False
            sign = c
    return True


def clearance(x, y, yaw, boxes):
    """Min distance from the footprint at (x, y, yaw) to any box; 0 if they overlap."""
    c, s = math.cos(yaw), math.sin(yaw)
    fp = [(x + c * px - s * py, y + s * px + c * py) for px, py in FOOTPRINT]
    best = math.inf
    for x0, x1, y0, y1 in boxes:
        bx = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if any(inside(p, bx) for p in fp) or any(inside(p, fp) for p in bx):
            return 0.0
        for i in range(4):
            for j in range(4):
                best = min(best, seg_dist(fp[i], fp[(i + 1) % 4], bx[j], bx[(j + 1) % 4]))
    return best


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder:
    def __init__(self, nav, lane_y, boxes):
        self.nav, self.lane_y, self.boxes = nav, lane_y, boxes
        self.truth0 = None
        self.last_xy = None
        self.path_len = 0.0
        self.min_clear = math.inf
        self.collision_samples = 0
        self.pose = None
        self.cmd_times = []          # (sim, wall) of every non-zero command to the wheels
        self.recoveries = {}
        self.plans = 0
        nav.create_subscription(Odometry, "/gz/odom", self.on_truth, 50)
        nav.create_subscription(Twist, "/diff_drive_controller/cmd_vel_unstamped", self.on_cmd, 50)
        nav.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self.on_bt, 50)
        nav.create_subscription(Path, "/plan", self.on_plan, 10)

    def now(self):
        return self.nav.get_clock().now().nanoseconds * 1e-9, time.monotonic()

    def on_truth(self, m):
        p = m.pose.pose.position
        if self.truth0 is None:
            # /gz/odom is either world coordinates or relative to the spawn; detect which.
            self.truth0 = (p.x, p.y)
        # World coordinates: the spawn is at (0, lane_y).
        world = abs(self.truth0[1] - self.lane_y) < 1.0
        x, y = p.x, p.y + (0.0 if world else self.lane_y)
        yaw = yaw_of(m.pose.pose.orientation)
        if self.last_xy:
            self.path_len += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.pose = (x, y, yaw)
        if self.boxes:
            cl = clearance(x, y, yaw, self.boxes)
            self.min_clear = min(self.min_clear, cl)
            self.collision_samples += cl <= 0.0

    def on_cmd(self, m):
        if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3:
            self.cmd_times.append(self.now())

    def on_bt(self, m):
        for e in m.event_log:
            if e.node_name in RECOVERY_NODES and e.current_status == "RUNNING" \
                    and e.previous_status == "IDLE":
                self.recoveries[e.node_name] = self.recoveries.get(e.node_name, 0) + 1

    def on_plan(self, _):
        self.plans += 1


def node_states(nav):
    bad = []
    for name in NAV2_NODES:
        cli = nav.create_client(GetState, f"/{name}/get_state")
        if not cli.wait_for_service(timeout_sec=5.0):
            bad.append(f"{name}:no_service")
            continue
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=5.0)
        label = fut.result().current_state.label if fut.result() else "no_reply"
        if label != "active":
            bad.append(f"{name}:{label}")
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scenario", choices=sorted(LANES), required=True)
    ap.add_argument("--goal-x", type=float, default=15.0, help="map-frame goal x (m)")
    ap.add_argument("--goal-y", type=float, default=0.0, help="map-frame goal y (m)")
    ap.add_argument("--timeout", type=float, default=300.0, help="sim seconds before giving up")
    ap.add_argument("--cancel-after", type=float, default=8.0, help="cancel: sim seconds")
    ap.add_argument("--world", default=os.path.join(
        get_package_share_directory("robot_description"), "worlds", "nav_test.sdf"))
    ap.add_argument("--run", default="", help="label stored with the result")
    ap.add_argument("--out", default="", help="append the JSON line to this file")
    a = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter("use_sim_time", value=True)])
    lane_y = LANES[a.scenario]
    rec = Recorder(nav, lane_y, world_boxes(a.world))

    # Not nav.waitUntilNav2Active(): in Humble it always waits for a localizer's lifecycle
    # (AMCL); robot_localization has none. Wait for the Nav2 servers themselves instead.
    deadline = time.monotonic() + 600.0
    while node_states(nav):
        if time.monotonic() > deadline:
            print(json.dumps({"scenario": a.scenario, "run": a.run, "result": "NAV2_NOT_ACTIVE",
                              "nodes_not_active": node_states(nav)}))
            raise SystemExit(1)
        time.sleep(2.0)
    while rec.pose is None or nav.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(nav, timeout_sec=0.1)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = a.goal_x, a.goal_y
    goal.pose.orientation.w = 1.0

    t0 = rec.now()
    accepted = nav.goToPose(goal)
    out = {"scenario": a.scenario, "run": a.run, "goal_map": [a.goal_x, a.goal_y]}
    cancel_t = None
    if not accepted:
        out["result"] = "REJECTED"
    else:
        while not nav.isTaskComplete():
            now = rec.now()
            if a.scenario == "cancel" and cancel_t is None and now[0] - t0[0] >= a.cancel_after:
                cancel_t = now
                nav.cancelTask()
            if now[0] - t0[0] > a.timeout:
                nav.cancelTask()
                out["result"] = "TIMEOUT"
                break
        out.setdefault("result", RESULTS.get(nav.getResult(), str(nav.getResult())))
    t1 = rec.now()

    # Let the last commands and truth samples arrive (2 sim seconds) before measuring.
    end = rec.now()[0] + 2.0
    while rec.now()[0] < end:
        rclpy.spin_once(nav, timeout_sec=0.05)

    first = next((c for c in rec.cmd_times if c[0] >= t0[0]), None)
    gx, gy = a.goal_x, a.goal_y + lane_y  # goal in world coordinates
    x, y, _ = rec.pose
    out.update({
        "t_first_cmd_sim_s": round(first[0] - t0[0], 2) if first else None,
        "t_first_cmd_wall_s": round(first[1] - t0[1], 2) if first else None,
        "t_total_sim_s": round(t1[0] - t0[0], 1),
        "t_total_wall_s": round(t1[1] - t0[1], 1),
        "real_time_factor": round((t1[0] - t0[0]) / max(t1[1] - t0[1], 1e-6), 2),
        "final_error_m": round(math.hypot(gx - x, gy - y), 2),
        "path_length_m": round(rec.path_len, 2),
        "min_clearance_m": round(rec.min_clear, 3) if rec.min_clear != math.inf else None,
        "collision": rec.collision_samples > 0,
        "recoveries": rec.recoveries,
        "plans": rec.plans,
    })
    if cancel_t:
        after = [c for c in rec.cmd_times if c[0] >= cancel_t[0]]
        out["cancel_to_last_cmd_sim_s"] = round(after[-1][0] - cancel_t[0], 2) if after else 0.0
        out["cancel_to_last_cmd_wall_s"] = round(after[-1][1] - cancel_t[1], 2) if after else 0.0
    out["nodes_not_active"] = node_states(nav)

    line = json.dumps(out)
    print(line)
    if a.out:
        with open(a.out, "a") as fh:
            fh.write(line + "\n")
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
