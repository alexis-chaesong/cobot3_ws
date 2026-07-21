"""
Input Shaping (ZV / ZVD) for liquid-slosh suppression.

물리/수학 배경
--------------
1) 등가 진자 모델 (Equivalent Pendulum Model)
   컵/식판 안의 액체 표면 진동을 "단진자"로 근사한다.
   진자 길이 L (액체 유효 진자 길이, 실측 필요), 중력가속도 g 에 대해

       ω_n = sqrt(g / L)                      (감쇠 없는 고유진동수, rad/s)
       ω_d = ω_n * sqrt(1 - ζ^2)              (감쇠계 고유진동수)

   ζ 는 감쇠비(damping ratio), 0 <= ζ < 1 (부족감쇠 가정).

2) ZV (Zero Vibration) Shaper
   2개의 임펄스로 구성. 잔류진동(residual vibration)을 이론상 0으로 만드는
   최소 임펄스 개수 shaper. 표준 공식 (Singer & Seering, 1990):

       T_d   = π / ω_d                                (임펄스 시간 간격)
       K     = exp(-ζπ / sqrt(1-ζ^2))                 (진폭 감쇠비)
       A1    = 1 / (1+K)
       A2    = K / (1+K)
       t1=0, t2=T_d

   장점: shaper 자체가 가장 짧다(rise time 증가 최소).
   단점: ω_n 추정 오차에 취약 (robustness 낮음).

3) ZVD (Zero Vibration and Derivative) Shaper
   ZV 조건에 더해 "잔류진동 진폭을 ω_n에 대해 미분한 값도 0"이 되도록
   임펄스를 하나 더 추가(3개). ω_n 추정 오차에 더 강건(robust)하다.

       t1=0, t2=T_d, t3=2*T_d
       A1 = 1 / (1+2K+K^2)
       A2 = 2K / (1+2K+K^2)
       A3 = K^2 / (1+2K+K^2)

   대신 shaper 길이가 2*T_d로 늘어나 동작 시간이 그만큼 길어진다
   (ZV 대비 rise time 증가).

4) Convolution 적용
   Input shaping은 "원래 지령 프로파일(가감속 프로파일)"과
   "임펄스 시퀀스(shaper)"를 컨볼루션(convolution)하는 것과 동일하다.
   이산 시계열에서는 각 임펄스를 해당 시간만큼 지연(shift)시킨 뒤
   진폭(amp)만큼 스케일하여 모두 더하는 것으로 구현할 수 있다
   (shift-and-add == discrete convolution with a sparse impulse train).

TODO(실측 필요):
   - ZETA_PLACEHOLDER: 실제 식판/컵 액체의 감쇠비. 현재는 물 사발 슬로싱
     실험값 근사치로 0.02~0.05 정도가 보고되는 경우가 많으나, 우리 시스템
     (식판 위 국그릇/컵, 병원 카트 이동 속도 범위)에 대한 실측 전이므로
     placeholder.
   - L_PLACEHOLDER: "유효 진자 길이"는 실제 진자 길이가 아니라 용기 형상,
     액체 채움 높이에 따라 결정되는 등가값이며, 통상 (Housner 1963 등)
     슬로싱 모드 이론으로 별도 계산하거나 슬로싱 실험(가진 후 감쇠 관찰)으로
     역산해야 한다. 지금은 표준 머그컵 크기 기준 대략적인 값으로 채워둠.
"""

from __future__ import annotations

import numpy as np

# ---------------------------------------------------------------------------
# Placeholder physical constants (실측 전 값 — 반드시 실험으로 교체할 것)
# ---------------------------------------------------------------------------
G = 9.81  # m/s^2, 중력가속도 (고정값, 실측 불필요)

# TODO: 실측 필요 — 액체 종류(물/국물 등), 용기 형상, 충진율에 따라 달라짐.
ZETA_PLACEHOLDER = 0.02  # 감쇠비 ζ (무차원), placeholder

# TODO: 실측 필요 — 등가 진자 길이. 슬로싱 주파수 실험(카트 가진 후 액면
# 진동 주파수 측정) 또는 Housner/Abramson 슬로싱 모델로 역산할 것.
L_PLACEHOLDER = 0.05  # m, placeholder (5 cm 정도의 소형 용기 가정)


def natural_frequency(L: float = L_PLACEHOLDER, zeta: float = ZETA_PLACEHOLDER):
    """등가 진자 모델 기반 고유진동수 계산.

    Args:
        L: 유효 진자 길이 [m]. TODO: 실측 전 placeholder 사용 중.
        zeta: 감쇠비 ζ. TODO: 실측 전 placeholder 사용 중.

    Returns:
        (omega_n, omega_d): 감쇠 없는/있는 고유각진동수 [rad/s]
    """
    if L <= 0:
        raise ValueError("L(진자 길이)은 0보다 커야 합니다.")
    if not (0.0 <= zeta < 1.0):
        raise ValueError("zeta는 [0, 1) 범위의 부족감쇠 조건이어야 합니다.")

    omega_n = np.sqrt(G / L)
    omega_d = omega_n * np.sqrt(1.0 - zeta**2)
    return omega_n, omega_d


def zv_shaper(L: float = L_PLACEHOLDER, zeta: float = ZETA_PLACEHOLDER):
    """ZV (Zero Vibration) shaper의 임펄스 시간/진폭을 계산한다.

    Returns:
        times: np.ndarray, shape (2,) — 임펄스 발생 시각 [s], t1=0
        amps:  np.ndarray, shape (2,) — 임펄스 진폭 (합=1)
    """
    omega_n, omega_d = natural_frequency(L, zeta)

    Td = np.pi / omega_d
    K = np.exp(-zeta * np.pi / np.sqrt(1.0 - zeta**2))

    A1 = 1.0 / (1.0 + K)
    A2 = K / (1.0 + K)

    times = np.array([0.0, Td])
    amps = np.array([A1, A2])
    return times, amps


def zvd_shaper(L: float = L_PLACEHOLDER, zeta: float = ZETA_PLACEHOLDER):
    """ZVD (Zero Vibration Derivative) shaper의 임펄스 시간/진폭을 계산한다.

    ZV 대비 강건성(robustness)이 높다 — ω_n 추정 오차에 덜 민감.
    대신 shaper 길이가 2배(2*Td)로 늘어나 동작이 그만큼 지연된다.

    Returns:
        times: np.ndarray, shape (3,) — 임펄스 발생 시각 [s]
        amps:  np.ndarray, shape (3,) — 임펄스 진폭 (합=1)
    """
    omega_n, omega_d = natural_frequency(L, zeta)

    Td = np.pi / omega_d
    K = np.exp(-zeta * np.pi / np.sqrt(1.0 - zeta**2))
    denom = 1.0 + 2.0 * K + K**2

    A1 = 1.0 / denom
    A2 = 2.0 * K / denom
    A3 = (K**2) / denom

    times = np.array([0.0, Td, 2.0 * Td])
    amps = np.array([A1, A2, A3])
    return times, amps


def apply_shaper(profile: np.ndarray, dt: float, impulse_times: np.ndarray,
                  impulse_amps: np.ndarray) -> np.ndarray:
    """원래 가감속 프로파일에 shaper(임펄스 시퀀스)를 convolution으로 적용.

    구현 방식(shift-and-add):
        shaped(t) = sum_i  A_i * profile(t - t_i)

    각 임펄스를 해당 시간만큼 지연시킨 원본 프로파일을 진폭만큼 스케일하여
    합산 — 이는 이산 신호와 sparse 임펄스 열의 컨볼루션과 수학적으로 동일하다.
    출력 길이는 원본 길이 + 마지막 임펄스의 지연 샘플 수 만큼 늘어난다
    (shaper 적용으로 인해 동작 시간이 T_d(ZV) 또는 2*T_d(ZVD)만큼 늘어남).

    Args:
        profile: 원본 시계열 (가속도/속도 등 어떤 프로파일이든 가능), shape (N,)
        dt: 샘플링 시간 간격 [s]
        impulse_times: shaper 임펄스 시각 [s]
        impulse_amps: shaper 임펄스 진폭 (합이 1이 되어야 정상)

    Returns:
        shaped: np.ndarray, shape (N + max_shift,)
    """
    if dt <= 0:
        raise ValueError("dt는 0보다 커야 합니다.")

    profile = np.asarray(profile, dtype=float)
    n_shifts = np.round(np.asarray(impulse_times) / dt).astype(int)

    out_len = len(profile) + int(n_shifts.max())
    shaped = np.zeros(out_len)

    for amp, shift in zip(impulse_amps, n_shifts):
        shaped[shift:shift + len(profile)] += amp * profile

    return shaped


if __name__ == "__main__":
    # 간단한 자체 점검(smoke test) — 실제 검증은 test_trajectory.py 참고
    wn, wd = natural_frequency()
    print(f"[placeholder] omega_n={wn:.3f} rad/s, omega_d={wd:.3f} rad/s")

    zv_t, zv_a = zv_shaper()
    print(f"ZV  impulses: t={zv_t}, A={zv_a}, sum(A)={zv_a.sum():.4f}")

    zvd_t, zvd_a = zvd_shaper()
    print(f"ZVD impulses: t={zvd_t}, A={zvd_a}, sum(A)={zvd_a.sum():.4f}")
