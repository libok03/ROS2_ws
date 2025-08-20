import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class DWA:
    def __init__(self):
        self.max_speed = 20.0  # [m/s]
        self.min_speed = -0.1  # [m/s]
        self.max_yaw_rate = 28.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.01  # [m/ss]
        self.max_delta_yaw_rate = 80.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 1.3  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.2  # [s] Time tick for motion prediction
        self.predict_time = 0.8  # [s]
        self.to_goal_cost_gain = 0.001
        self.speed_cost_gain = 5.0
        self.obstacle_cost_gain = 0.2
        self.robot_stuck_flag_cons = 0.1  # constant to prevent robot stucked

        # Also used to check if goal is reached in both types
        self.robot_radius = 3.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 1.0  # [m] for collision check
        self.robot_length = 2.0  # [m] for collision check
        self.start = []
        self.goal = []

        self.ob = np.array(
            [
                # [-87.66267, 36.81775],
                [-86.18295, 40.51861],
                # [-83.41397, 43.80231],
                [-79.83937, 46.19911],
                # [-75.92238, 47.30313],
                [-71.75175, 46.96906],
                # [-68.17937, 45.46938],
                [-65.27208, 43.29974],
                # [-62.69083, 40.49252],
                [-61.39494, 36.38269],
                # [-74.7186, 31.20725],
                [-74.56528, 26.9605],
                [-74.65209, 22.10976],
                [-74.58686, 18.36543],
                # [-74.73897, 14.00041],
                [-74.68031, 9.516319],
                # [-74.72476, 5.370415],
                [-74.80842, 1.542259],
            ]
        )

    def dwa_control(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)

        return u, trajectory

    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    def calc_dynamic_window(self, x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [
            x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt,
        ]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3]),
        ]

        return dw

    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory

    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(
                    trajectory, goal
                )
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(
                    trajectory, ob
                )

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if (
                        abs(best_u[0]) < self.robot_stuck_flag_cons
                        and abs(x[3]) < self.robot_stuck_flag_cons
                    ):
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.max_delta_yaw_rate
        return best_u, best_trajectory

    def calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        # 장애물 배열이 비어 있는지 체크
        if ob.size == 0:
            return float("Inf")

        # 궤적 배열이 비어 있는지 체크
        if trajectory.shape[0] == 0:
            return float("Inf")

        # 장애물 좌표 추출
        ox = ob[:, 0]
        oy = ob[:, 1]

        # 장애물과 궤적 점 사이의 거리 계산
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        # 거리 계산 결과 확인
        # print("Distances:", r)  # 디버깅용

        # 최소 거리 계산
        if r.size == 0:
            return float("Inf")

        min_r = np.min(r)

        # 로봇의 회전 각도와 회전 행렬 계산
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])

        # 로봇 좌표계에서 장애물의 위치 계산
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])

        # 로봇 크기 기준으로 장애물 위치 확인
        upper_check = local_ob[:, 0] <= self.robot_length / 2
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2

        # 로봇과 장애물이 충돌하는지 확인
        if (
            np.logical_and(
                np.logical_and(upper_check, right_check),
                np.logical_and(bottom_check, left_check),
            )
        ).any():
            return float("Inf")

        return 1.0 / min_r  # 장애물과의 최소 거리의 역수 반환

    def calc_to_goal_cost(self, trajectory, goal):
        """
        calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(
            x,
            y,
            length * math.cos(yaw),
            length * math.sin(yaw),
            head_length=width,
            head_width=width,
        )
        plt.plot(x, y)

    def plot_robot(self, x, y, yaw):  # pragma: no cover

        outline = np.array(
            [
                [
                    -self.robot_length / 2,
                    self.robot_length / 2,
                    (self.robot_length / 2),
                    -self.robot_length / 2,
                    -self.robot_length / 2,
                ],
                [
                    self.robot_width / 2,
                    self.robot_width / 2,
                    -self.robot_width / 2,
                    -self.robot_width / 2,
                    self.robot_width / 2,
                ],
            ]
        )
        Rot1 = np.array(
            [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]
        )
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(
            np.array(outline[0, :]).flatten(),
            np.array(outline[1, :]).flatten(),
            "-k",
        )


def main():
    dwa = DWA()
    dwa.start = np.array([-82.5, 1.3, math.pi / 02.0, 0.0, 0.0])
    dwa.goal = np.array([-67.5, 14.9])

    dwa.ob = np.array(
        [
            # [-87.66267, 36.81775],
            [-86.18295, 40.51861],
            # [-83.41397, 43.80231],
            [-79.83937, 46.19911],
            # [-75.92238, 47.30313],
            [-71.75175, 46.96906],
            # [-68.17937, 45.46938],
            [-65.27208, 43.29974],
            # [-62.69083, 40.49252],
            [-61.39494, 36.38269],
            # [-74.7186, 31.20725],
            [-74.56528, 26.9605],
            [-74.65209, 22.10976],
            [-74.58686, 18.36543],
            # [-74.73897, 14.00041],
            [-74.68031, 9.516319],
            # [-74.72476, 5.370415],
            [-74.80842, 1.542259],
        ]
    )
    # dwa.ob = np.array([])
    trajectory = np.array(dwa.start)
    x = np.array([-82.5, 1.3, math.pi / 2.0, 0.0, 0.0])
    while True:
        # for i in range(1):
        u, predicted_trajectory = dwa.dwa_control(x, dwa.goal, dwa.ob)
        print(len(predicted_trajectory))
        x = dwa.motion(x, u, dwa.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history
        print(f"x: {x}, x[0]: {x[0]}, x[1]: {x[1]}, x[2]: {x[2]}")
        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(dwa.goal[0], dwa.goal[1], "xb")
            plt.plot(dwa.ob[:, 0], dwa.ob[:, 1], "ok")
            dwa.plot_robot(x[0], x[1], x[2])
            dwa.plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)
            # plt.show()

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - dwa.goal[0], x[1] - dwa.goal[1])
        if dist_to_goal <= 3:  # config.robot_radius:
            print("Goal!!")
            break

    print("Done")


if __name__ == "__main__":
    main()