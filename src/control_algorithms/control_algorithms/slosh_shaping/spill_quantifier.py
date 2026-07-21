"""
Equivalent-Pendulum Spill Quantifier (DRAFT / 초안).

주의: 이 파일은 "초안(draft)" 수준이다. 실측 파라미터(ζ, L, 임계각)가
전혀 검증되지 않았으며, 아래 모델은 소각도(small-angle) 강제진동 근사에
불과하다. Isaac Sim 유체 시뮬레이션 또는 실제 실험 데이터로 반드시
검증/보정(calibration)해야 한다.

물리/수학 배경
--------------
액체 표면 기울기를 "받침대(용기)가 가속되는 단진자의 강제진동 응답"으로
근사한다. 용기가 수평 가속도 a(t)를 받을 때, 진자(액면)에는 관성력
-m*a(t)가 작용하며, 이는 진자 기준계에서 "가상의 수평 중력 성분"처럼
작용한다. 소각도 가정(sinθ≈θ) 하에서 감쇠 강제진동 운동방정식은:

    θ''(t) + 2*ζ*ω_n*θ'(t) + ω_n^2*θ(t) = -a(t) / L

    (표준 감쇠 강제진동 방정식 θ'' + 2ζω_n θ' + ω_n^2 θ = F(t)/m 에서,
     "단위질량 진자의 밑면 가속도 가진"의 등가 힘 F/m = -a(t)/L 로 둔 것.
     L로 나누는 이유: 진자 운동방정식 원형이 θ'' + (g/L)θ = -a(t)/L 이고
     ω_n^2 = g/L 이므로 힘항도 자연스럽게 1/L 스케일을 갖는다.)

여기서:
    a(t) : 캐리어(로봇팔 엔드이펙터/카트)의 수평 가속도 프로파일 [m/s^2]
           (min_jerk_trajectory의 accel 출력을 그대로 사용)
    L    : 등가 진자 길이 [m] (input_shaping.py의 L_PLACEHOLDER와 동일 개념)
    ζ    : 감쇠비
    ω_n  : sqrt(g/L)

이 2차 ODE를 상태공간 [θ, θ'] 으로 변환하여 scipy.integrate.solve_ivp로
수치적분한다.

임계각 판정: θ(t)가 CRITICAL_ANGLE_DEG_PLACEHOLDER를 초과하는 구간과
최댓값을 리포트한다 (실제 "넘침" 여부는 용기 충진율, 형상에 따라 다르므로
15도는 어디까지나 자리표시자).

TODO(실측/검증 필요):
   - CRITICAL_ANGLE_DEG_PLACEHOLDER: 실제 식판 국그릇 등에서 넘침이
     시작되는 각도는 충진율(fill ratio)에 크게 의존. 15도는 placeholder.
   - ζ, L: input_shaping.py와 동일한 미실측 상수를 그대로 참조.
   - 소각도 근사(sinθ≈θ)는 θ가 커질수록(특히 임계각 부근) 부정확해짐.
     추후 비선형 진자 모델 또는 Housner 슬로싱 모델로 교체 검토.
"""

from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp

from input_shaping import G, L_PLACEHOLDER, ZETA_PLACEHOLDER, natural_frequency

# TODO: 실측/실험 필요 — 용기 형상·충진율별 넘침 임계각.
CRITICAL_ANGLE_DEG_PLACEHOLDER = 15.0


def tilt_angle_response(t: np.ndarray, accel: np.ndarray,
                         L: float = L_PLACEHOLDER,
                         zeta: float = ZETA_PLACEHOLDER,
                         theta0: float = 0.0, theta_dot0: float = 0.0):
    """가속도 프로파일 a(t)에 대한 액면 기울기각 θ(t) 응답을 계산.

    Args:
        t: 시간 배열 [s], shape (N,), 오름차순 균일/비균일 모두 허용
        accel: 수평 가속도 프로파일 [m/s^2], shape (N,) (t와 동일 길이)
        L, zeta: 등가 진자 길이 / 감쇠비 (미실측 placeholder 기본값)
        theta0, theta_dot0: 초기 조건 (기본 0 — 정지 상태에서 시작 가정)

    Returns:
        t_eval: solve_ivp가 실제 평가한 시간 배열 (=t)
        theta_deg: θ(t) [degree]
        theta_dot: θ'(t) [rad/s]
    """
    t = np.asarray(t, dtype=float)
    accel = np.asarray(accel, dtype=float)
    if t.shape != accel.shape:
        raise ValueError("t와 accel의 shape이 같아야 합니다.")

    omega_n, _ = natural_frequency(L, zeta)

    # 가속도 프로파일을 시간에 대해 선형보간하여 ODE 우변 함수를 만든다.
    def accel_interp(tt):
        return np.interp(tt, t, accel)

    def rhs(tt, y):
        theta, theta_dot = y
        forcing = -accel_interp(tt) / L
        theta_ddot = forcing - 2.0 * zeta * omega_n * theta_dot - omega_n**2 * theta
        return [theta_dot, theta_ddot]

    sol = solve_ivp(
        rhs, t_span=(t[0], t[-1]), y0=[theta0, theta_dot0],
        t_eval=t, method="RK45", rtol=1e-8, atol=1e-10,
    )

    theta_rad = sol.y[0]
    theta_dot = sol.y[1]
    theta_deg = np.degrees(theta_rad)
    return sol.t, theta_deg, theta_dot


def evaluate_spill_risk(t: np.ndarray, theta_deg: np.ndarray,
                         critical_deg: float = CRITICAL_ANGLE_DEG_PLACEHOLDER):
    """θ(t)가 임계각을 넘는 구간과 최댓값을 요약.

    Returns:
        dict:
            max_angle_deg: 최대 |θ| [deg]
            max_angle_time: 그 시각 [s]
            exceed_mask: 임계각 초과 여부 boolean 배열
            exceed_intervals: [(t_start, t_end), ...] 초과 구간 리스트
            settling_time: |θ|가 임계각의 5% 이하로 계속 유지되기 시작하는
                시각(대략적 정착시간, settling time). 없으면 None.
    """
    t = np.asarray(t, dtype=float)
    theta_abs = np.abs(np.asarray(theta_deg, dtype=float))

    max_idx = int(np.argmax(theta_abs))
    max_angle = float(theta_abs[max_idx])
    max_time = float(t[max_idx])

    exceed_mask = theta_abs > critical_deg

    intervals = []
    in_run = False
    start = None
    for i, flag in enumerate(exceed_mask):
        if flag and not in_run:
            in_run = True
            start = t[i]
        elif not flag and in_run:
            in_run = False
            intervals.append((start, t[i - 1]))
    if in_run:
        intervals.append((start, t[-1]))

    # 정착시간: 임계각의 5% 이내로 "그 이후 계속" 유지되는 첫 시각
    tol = 0.05 * critical_deg
    settling_time = None
    below = theta_abs <= tol
    for i in range(len(below)):
        if np.all(below[i:]):
            settling_time = float(t[i])
            break

    return {
        "max_angle_deg": max_angle,
        "max_angle_time": max_time,
        "exceed_mask": exceed_mask,
        "exceed_intervals": intervals,
        "settling_time": settling_time,
    }


def compare_before_after(t: np.ndarray, accel_raw: np.ndarray,
                          accel_shaped: np.ndarray, t_shaped: np.ndarray = None,
                          L: float = L_PLACEHOLDER, zeta: float = ZETA_PLACEHOLDER,
                          critical_deg: float = CRITICAL_ANGLE_DEG_PLACEHOLDER):
    """Input Shaping 적용 전/후 θ(t) 응답을 나란히 비교 (초안 수준).

    Args:
        t: raw accel의 시간축
        accel_raw: shaping 미적용 가속도 프로파일
        accel_shaped: shaping 적용된 가속도 프로파일 (input_shaping.apply_shaper 출력)
        t_shaped: shaped 프로파일 전용 시간축. None이면 raw와 같은 dt로
            accel_shaped 길이에 맞춰 자동 생성.
        L, zeta, critical_deg: spill 판정 파라미터

    Returns:
        dict with keys "before", "after" — 각각
            {"t":..., "theta_deg":..., "summary": evaluate_spill_risk(...)}
    """
    t = np.asarray(t, dtype=float)

    if t_shaped is None:
        dt = t[1] - t[0] if len(t) > 1 else 1.0
        t_shaped = np.arange(len(accel_shaped)) * dt

    t_b, theta_b, _ = tilt_angle_response(t, accel_raw, L=L, zeta=zeta)
    t_a, theta_a, _ = tilt_angle_response(t_shaped, accel_shaped, L=L, zeta=zeta)

    summary_b = evaluate_spill_risk(t_b, theta_b, critical_deg)
    summary_a = evaluate_spill_risk(t_a, theta_a, critical_deg)

    return {
        "before": {"t": t_b, "theta_deg": theta_b, "summary": summary_b},
        "after": {"t": t_a, "theta_deg": theta_a, "summary": summary_a},
    }


if __name__ == "__main__":
    from min_jerk_trajectory import min_jerk_profile
    from input_shaping import zvd_shaper, apply_shaper

    t, pos, vel, accel, jerk = min_jerk_profile(p0=0.0, pf=0.3, T=1.0, num_samples=500)
    dt = t[1] - t[0]

    zvd_t, zvd_a = zvd_shaper()
    accel_shaped = apply_shaper(accel, dt, zvd_t, zvd_a)

    result = compare_before_after(t, accel, accel_shaped)

    print("[Before shaping]")
    print(f"  max |theta| = {result['before']['summary']['max_angle_deg']:.3f} deg "
          f"at t={result['before']['summary']['max_angle_time']:.3f}s")
    print(f"  settling_time = {result['before']['summary']['settling_time']}")

    print("[After ZVD shaping]")
    print(f"  max |theta| = {result['after']['summary']['max_angle_deg']:.3f} deg "
          f"at t={result['after']['summary']['max_angle_time']:.3f}s")
    print(f"  settling_time = {result['after']['summary']['settling_time']}")
