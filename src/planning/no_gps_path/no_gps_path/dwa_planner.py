#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np

class DWAConfig:
    """DWA 기본 파라미터 설정"""
    max_speed = 2.0            # [m/s]
    min_speed = -0.5           # [m/s]
    max_yaw_rate = math.radians(28.0)    # [rad/s]
    max_accel = 0.2            # [m/s^2]
    max_delta_yaw_rate = math.radians(40.0)  # [rad/s^2]
    v_resolution = 0.01        # [m/s]
    yaw_rate_resolution = math.radians(1.0)  # [rad/s]
    dt = 0.1                   # [s] 예측 시간 간격
    predict_time = 2.0         # [s] 예측 시간
    to_goal_cost_gain = 0.15
    speed_cost_gain = 1.0
    obstacle_cost_gain = 1.0

class DWAPlanner:
    def __init__(self, config=None):
        self.cfg = config or DWAConfig()

    def plan(self, obstacles, x, y, yaw, v=0.0):
        """
        장애물 리스트와 현재 상태를 받아 최적 궤적을 반환
        :param obstacles: [[ox1, oy1], [ox2, oy2], ...]
        :param x: 현재 x 위치
        :param y: 현재 y 위치
        :param yaw: 현재 yaw (rad)
        :param v: 현재 속도 (m/s)
        :return: best_traj: [[x, y, yaw, v], ...] 형태의 리스트
        """
        # state = [x, y, yaw, v]
        state = [0.0, 0.0, yaw, v]  # base_link 기준, (0,0)에서 출발
        
        # 동적 윈도우 계산
        dw = self._calc_dynamic_window(state)

        best_traj = [state[:]]
        best_u = [0.0, 0.0]
        min_cost = float('inf')

        # (v, yaw_rate) 후보 모두 평가
        vs = np.arange(dw[0], dw[1], self.cfg.v_resolution)
        ys = np.arange(dw[2], dw[3], self.cfg.yaw_rate_resolution)
        for vv in vs:
            for y_rate in ys:
                traj = self._predict_trajectory(state, [vv, y_rate])
                cost = (
                    self.cfg.to_goal_cost_gain   * abs(self._heading_cost(traj)) +
                    self.cfg.speed_cost_gain     * (self.cfg.max_speed - traj[-1][3]) +
                    self.cfg.obstacle_cost_gain  * self._obstacle_cost(traj, obstacles)
                )
                if cost < min_cost:
                    min_cost = cost
                    best_u = [vv, y_rate]
                    best_traj = traj

        return best_traj

    def _calc_dynamic_window(self, state):
        """현재 상태에서 허용 가능한 속도·yaw_rate 범위 계산"""
        cfg = self.cfg
        Vs = [cfg.min_speed, cfg.max_speed,
              -cfg.max_yaw_rate, cfg.max_yaw_rate]
        Vd = [state[3] - cfg.max_accel * cfg.dt,
              state[3] + cfg.max_accel * cfg.dt,
              state[3] - cfg.max_delta_yaw_rate * cfg.dt,
              state[3] + cfg.max_delta_yaw_rate * cfg.dt]
        return [
            max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])
        ]

    def _predict_trajectory(self, state, u):
        """주어진 제어 입력 u=[v, yaw_rate]로 궤적 예측"""
        cfg = self.cfg
        t = 0.0
        traj = [state[:]]
        x, y, yaw, _ = state

        while t <= cfg.predict_time:
            x   += u[0] * math.cos(yaw) * cfg.dt
            y   += u[0] * math.sin(yaw) * cfg.dt
            yaw += u[1] * cfg.dt
            traj.append([x, y, yaw, u[0]])
            t += cfg.dt

        return traj

    def _heading_cost(self, traj):
        """궤적 끝점 yaw와 목표(0) 차이"""
        return traj[-1][2]

    def _obstacle_cost(self, traj, obstacles):
        """궤적과 장애물 간 최소 거리 기반 비용"""
        min_dist = float('inf')
        for x, y, _, _ in traj:
            for ox, oy in obstacles:
                d = math.hypot(ox - x, oy - y)
                if d < min_dist:
                    min_dist = d
        if min_dist == 0:
            return float('inf')  # 충돌
        return 1.0 / min_dist


