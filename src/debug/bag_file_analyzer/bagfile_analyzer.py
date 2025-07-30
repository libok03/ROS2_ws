#!/usr/bin/env python3
import os
import argparse
import numpy as np

# NumPy 2.0 compatibility patch
if not hasattr(np, 'maximum_sctype'):
    np.maximum_sctype = lambda t: np.dtype(t).type
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sqlite3
import matplotlib.pyplot as plt
import mpld3  # for interactive HTML export

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle around Z axis."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny, cosy)

class BagfileAnalyzer(Node):
    def __init__(self, db_path: str):
        super().__init__('bagfile_analyzer')
        if not os.path.isfile(db_path):
            raise FileNotFoundError(f"DB file not found: {db_path}")
        self.db_path = db_path
        self.db_pids, self.db_x, self.db_y, self.db_yaw, self.db_missions = self._load_db()
        self.get_logger().info(f"Loaded {len(self.db_x)} points from DB Path table.")

        self.prev_pid = None
        self.start_time = None
        self.lap_times = {}

        self.ts = []
        self.x = []
        self.y = []
        self.yaw = []
        self.speed = []
        self.err_xy = []
        self.err_yaw = []

        self.pid_indices = {}

        plt.ion()
        self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 15))
        self.cb_yaw = None
        self.cb_speed = None

        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self._callback,
            10
        )
        self.get_logger().info(f"Subscribed to '/localization/kinematic_state', DB: {self.db_path}")

    def _load_db(self):
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute("SELECT path_id, x, y, yaw FROM Path ORDER BY idx")
        rows = cur.fetchall()
        cur.execute("SELECT path_id, mission FROM Node")
        node_rows = cur.fetchall()
        conn.close()
        if not rows:
            self.get_logger().warn("No rows in DB 'Path' table.")
            return (np.array([]), np.array([]), np.array([]), np.array([]), {})
        pids = np.array([r[0] for r in rows])
        xs = np.array([r[1] for r in rows])
        ys = np.array([r[2] for r in rows])
        yaws = np.array([r[3] for r in rows])
        missions = {}
        for r in node_rows:
            missions[r[0]] = r[1]
        return pids, xs, ys, yaws, missions

    def _compute_error(self, x: float, y: float, yaw: float):
        if len(self.db_x) == 0:
            return 0.0, 0.0
        d = np.hypot(self.db_x - x, self.db_y - y)
        i = int(np.argmin(d))
        err_xy = float(d[i])
        delta = yaw - self.db_yaw[i]
        err_yaw = float((delta + np.pi) % (2*np.pi) - np.pi)
        return err_xy, err_yaw

    def save_analysis_figure(self, save_path, ts, x, y, yaw, speed, err_xy, err_yaw,
                             db_x=None, db_y=None, db_pids=None, db_yaw=None, db_missions=None,
                             pid=None, lap_times=None):
        fig, axs = plt.subplots(3, 2, figsize=(12, 15))

        # 1) Lap time per path_id
        ax = axs[0, 0]
        ax.axis('off')
        if lap_times:
            items = list(lap_times.items())
            nrows = 15
            chunk = (len(items) + nrows - 1) // nrows
            lines = []
            for i in range(nrows):
                part = items[i*chunk:(i+1)*chunk]
                if not part:
                    break
                def fmt(pid, lap):
                    mission = db_missions.get(pid, "None") if db_missions else "None"
                    return f"{pid}({mission}):{lap:.2f}s"
                lines.append(' | '.join([fmt(pid, lap) for pid, lap in part]))
            text_str = "\n".join(lines)
            ax.text(0.02, 0.95, text_str, transform=ax.transAxes, va='top', ha='left', fontsize=10)
        else:
            ax.text(0.5, 0.5, 'No laps yet', ha='center', va='center', transform=ax.transAxes)

        # 2) Trajectory colored by speed
        ax = axs[0, 1]
        sc2 = ax.scatter(x, y, c=speed, s=8)
        ax.set_title('Trajectory (speed)')
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.grid(True)
        fig.colorbar(sc2, ax=ax, label='Speed [m/s]')

        # 3) Speed vs Curvature
        ax = axs[1, 0]
        ax.set_title('Speed vs Curvature')
        ax.set_xlabel('Curvature [1/m]'); ax.set_ylabel('Speed [m/s]'); ax.grid(True)
        if len(x) >= 3:
            dx = np.gradient(x, ts)
            dy = np.gradient(y, ts)
            ddx = np.gradient(dx, ts)
            ddy = np.gradient(dy, ts)
            curv = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
            ax.scatter(curv, speed, s=8)

        # 4) Overlay DB vs Local
        ax = axs[1, 1]
        if db_x is not None and db_y is not None:
            if pid is not None and db_pids is not None:
                db_mask = (db_pids == pid)
                ax.plot(db_x[db_mask], db_y[db_mask], 'r--', label='DB')
            else:
                ax.plot(db_x, db_y, 'r--', label='DB')
            ax.plot(x, y, 'b-', label='Local')
            ax.legend()
        else:
            ax.text(0.5, 0.5, 'No DB data', ha='center', va='center', transform=ax.transAxes)
        title_pid = f' (path_id={pid})' if pid is not None else ''
        ax.set_title('DB vs Local Path' + title_pid)
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.grid(True)

        # 5) Position RMSE over time
        ax = axs[2, 0]
        if err_xy is not None and len(err_xy) > 0:
            err_arr = np.array(err_xy)
            rmse_pos = np.sqrt(np.cumsum(err_arr**2) / np.arange(1, len(err_arr) + 1))
            ax.plot(ts, rmse_pos, color='tab:red', label='Pos RMSE')
            ax.legend()
        else:
            ax.text(0.5, 0.5, 'No position data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Position RMSE')
        ax.set_xlabel('Time [s]'); ax.set_ylabel('RMSE [m]'); ax.grid(True)

        # 6) Yaw RMSE over time
        ax = axs[2, 1]
        if err_yaw is not None and len(err_yaw) > 0:
            yaw_arr = np.array(err_yaw)
            rmse_yaw = np.sqrt(np.cumsum(yaw_arr**2) / np.arange(1, len(yaw_arr) + 1))
            ax.plot(ts, rmse_yaw, color='tab:blue', label='Yaw RMSE')
            ax.legend()
        else:
            ax.text(0.5, 0.5, 'No yaw data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Yaw RMSE')
        ax.set_xlabel('Time [s]'); ax.set_ylabel('RMSE [rad]'); ax.grid(True)

        mission = ''
        if pid is not None and db_missions:
            mission = db_missions.get(pid, '')
        fig.suptitle(f'Path ID: {pid} {mission}', fontsize=16)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        fig.savefig(save_path)
        plt.close(fig)
        print(f'Saved {save_path}')

    def save_all_path_figures(self):
        for pid, idxs in self.pid_indices.items():
            if not idxs: continue
            idxs = np.array(idxs)
            ts = np.array(self.ts)[idxs]
            x = np.array(self.x)[idxs]
            y = np.array(self.y)[idxs]
            yaw = np.array(self.yaw)[idxs]
            speed = np.array(self.speed)[idxs]
            err_xy = np.array(self.err_xy)[idxs]
            err_yaw = np.array(self.err_yaw)[idxs]
            lap_times = {}
            if self.lap_times and pid in self.lap_times:
                lap_times[pid] = self.lap_times[pid]
            save_path = f'path_{pid}.png'
            self.save_analysis_figure(
                save_path, ts, x, y, yaw, speed, err_xy, err_yaw,
                db_x=self.db_x, db_y=self.db_y,
                db_pids=self.db_pids, db_yaw=self.db_yaw, db_missions=self.db_missions,
                pid=pid, lap_times=lap_times
            )

    def _callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        spd = float(np.hypot(vx, vy))

        idx = len(self.x)
        self.ts.append(t)
        self.x.append(x)
        self.y.append(y)
        self.yaw.append(yaw)
        self.speed.append(spd)

        if len(self.db_x) > 0:
            dists = np.hypot(self.db_x - x, self.db_y - y)
            i = int(np.argmin(dists))
            pid = self.db_pids[i]
            self.pid_indices.setdefault(pid, []).append(idx)
            if self.prev_pid is None:
                self.prev_pid = pid
                self.start_time = t
            elif pid != self.prev_pid:
                lap = t - self.start_time
                self.lap_times[self.prev_pid] = lap
                self.prev_pid = pid
                self.start_time = t

        e_xy, e_yaw = self._compute_error(x, y, yaw)
        self.err_xy.append(e_xy)
        self.err_yaw.append(e_yaw)

        self._update_plots()

    def _update_plots(self):
        for ax in self.axs.flatten():
            ax.cla()

        # 1) Lap time per path_id
        ax = self.axs[0, 0]
        ax.axis('off')
        if self.lap_times:
            items = list(self.lap_times.items())
            nrows = 15
            chunk = (len(items) + nrows - 1) // nrows
            lines = []
            for i in range(nrows):
                part = items[i*chunk:(i+1)*chunk]
                if not part:
                    break
                def fmt(pid, lap):
                    mission = self.db_missions.get(pid, "None")
                    return f"{pid}({mission}):{lap:.2f}s"
                lines.append(' | '.join([fmt(pid, lap) for pid, lap in part]))
            text_str = "\n".join(lines)
            ax.text(0.02, 0.95, text_str, transform=ax.transAxes, va='top', ha='left', fontsize=10)
        else:
            ax.text(0.5, 0.5, 'No laps yet', ha='center', va='center', transform=ax.transAxes)

        # 2) Trajectory colored by speed
        ax = self.axs[0, 1]
        sc2 = ax.scatter(self.x, self.y, c=self.speed, s=8)
        ax.set_title('Trajectory (speed)')
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.grid(True)
        if self.cb_speed is None:
            self.cb_speed = self.fig.colorbar(sc2, ax=ax)
            self.cb_speed.set_label('Speed [m/s]')
        else:
            self.cb_speed.update_normal(sc2)

        # 3) Speed vs Curvature
        ax = self.axs[1, 0]
        ax.set_title('Speed vs Curvature')
        ax.set_xlabel('Curvature [1/m]'); ax.set_ylabel('Speed [m/s]'); ax.grid(True)
        if len(self.x) >= 3:
            dx = np.gradient(self.x, self.ts)
            dy = np.gradient(self.y, self.ts)
            ddx = np.gradient(dx, self.ts)
            ddy = np.gradient(dy, self.ts)
            curv = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
            ax.scatter(curv, self.speed, s=8)

        # 4) Overlay DB vs Local
        ax = self.axs[1, 1]
        if len(self.db_x) > 0:
            ax.plot(self.db_x, self.db_y, 'r--', label='DB')
            ax.plot(self.x, self.y, 'b-', label='Local')
            ax.legend()
        else:
            ax.text(0.5, 0.5, 'No DB data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title('DB vs Local Path')
        ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.grid(True)

        # 5) Position RMSE over time
        ax = self.axs[2, 0]
        if self.err_xy is not None and len(self.err_xy) > 0:
            err_arr = np.array(self.err_xy)
            rmse_pos = np.sqrt(np.cumsum(err_arr**2) / np.arange(1, len(err_arr) + 1))
            ax.plot(self.ts, rmse_pos, color='tab:red', label='Pos RMSE')
            ax.legend()
        else:
            ax.text(0.5, 0.5, 'No position data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Position RMSE')
        ax.set_xlabel('Time [s]'); ax.set_ylabel('RMSE [m]'); ax.grid(True)

        # 6) Yaw RMSE over time
        ax = self.axs[2, 1]
        if self.err_yaw is not None and len(self.err_yaw) > 0:
            yaw_arr = np.array(self.err_yaw)
            rmse_yaw = np.sqrt(np.cumsum(yaw_arr**2) / np.arange(1, len(yaw_arr) + 1))
            ax.plot(self.ts, rmse_yaw, color='tab:blue', label='Yaw RMSE')
            ax.legend()
        else:
            ax.text(0.5, 0.5, 'No yaw data', ha='center', va='center', transform=ax.transAxes)
        ax.set_title('Yaw RMSE')
        ax.set_xlabel('Time [s]'); ax.set_ylabel('RMSE [rad]'); ax.grid(True)

        self.fig.tight_layout()
        plt.pause(0.001)

def main():
    parser = argparse.ArgumentParser(description='Bagfile Analyzer')
    parser.add_argument('--db', '-d', default='/home/libok/db_file/bunsudae_v1.db', help='SQLite DB path')
    args = parser.parse_args()

    rclpy.init()
    node = BagfileAnalyzer(args.db)
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info('Saving figures...')
        node.save_analysis_figure(
            'analysis.png',
            node.ts, node.x, node.y, node.yaw, node.speed, node.err_xy, node.err_yaw,
            db_x=node.db_x, db_y=node.db_y,
            db_pids=node.db_pids, db_yaw=node.db_yaw, db_missions=node.db_missions,
            pid=None, lap_times=node.lap_times
        )
        node.save_all_path_figures()
        mpld3.save_html(node.fig, 'analysis.html')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
