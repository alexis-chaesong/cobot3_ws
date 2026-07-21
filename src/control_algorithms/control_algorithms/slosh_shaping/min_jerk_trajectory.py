"""
Minimum-Jerk Quintic (5th-order) Trajectory Generator.

물리/수학 배경
--------------
경계조건 (boundary conditions), t=0 ~ t=T:

    p(0)  = p0     p(T)  = pf
    v(0)  = 0      v(T)  = 0
    a(0)  = 0      a(T)  = 0

6개 조건 -> 6개 계수를 갖는 5차 다항식으로 유일해 결정 가능:

    p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5

경계조건을 대입해 풀면 (표준 min-jerk 공식, Flash & Hogan 1985 계열):

    c0 = p0
    c1 = 0
    c2 = 0
    c3 =  10*(pf-p0) / T^3
    c4 = -15*(pf-p0) / T^4
    c5 =   6*(pf-p0) / T^5

정규화 시간 τ = t/T 를 쓰면 더 간단한 형태로도 쓸 수 있다:

    p(t)     = p0 + (pf-p0) * (10τ^3 - 15τ^4 + 6τ^5)
    v(t)     = (pf-p0)/T   * (30τ^2 - 60τ^3 + 30τ^4)
    a(t)     = (pf-p0)/T^2 * (60τ   - 180τ^2 + 120τ^3)
    j(t)     = (pf-p0)/T^3 * (60    - 360τ   + 360τ^2)

이 다항식은 jerk(가가속도)의 시간적분 제곱을 최소화하는 해로 알려져 있으며,
급격한 가속도 변화(=액체 슬로싱을 유발하는 주요 원인 중 하나)를 줄이는
부드러운 궤적을 만든다는 점에서 이 프로젝트(슬로싱 억제)의 베이스라인
프로파일로 적합하다. Input Shaping은 이 위에 추가로 적용되는 필터다.
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt


def min_jerk_coeffs(p0: float, pf: float, T: float) -> np.ndarray:
    """단일 축(단일 관절) min-jerk 5차 다항식 계수 [c0..c5]를 반환.

    v0=vf=0, a0=af=0 경계조건 고정 (요구사항에 명시된 표준 형태).
    """
    if T <= 0:
        raise ValueError("T(이동 시간)는 0보다 커야 합니다.")

    delta = pf - p0
    c0 = p0
    c1 = 0.0
    c2 = 0.0
    c3 = 10.0 * delta / T**3
    c4 = -15.0 * delta / T**4
    c5 = 6.0 * delta / T**5
    return np.array([c0, c1, c2, c3, c4, c5])


def min_jerk_profile(p0: float, pf: float, T: float, num_samples: int = 200):
    """단일 축 min-jerk pos/vel/accel/jerk 프로파일을 계산.

    Returns:
        t:     시간 배열, shape (N,)
        pos, vel, accel, jerk: 각 shape (N,)
    """
    if T <= 0:
        raise ValueError("T(이동 시간)는 0보다 커야 합니다.")

    t = np.linspace(0.0, T, num_samples)
    tau = t / T
    delta = pf - p0

    pos = p0 + delta * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    vel = (delta / T) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    accel = (delta / T**2) * (60 * tau - 180 * tau**2 + 120 * tau**3)
    jerk = (delta / T**3) * (60 - 360 * tau + 360 * tau**2)

    return t, pos, vel, accel, jerk


def min_jerk_joint_space(p0: np.ndarray, pf: np.ndarray, T,
                          num_samples: int = 200):
    """다관절(joint-space) min-jerk 프로파일.

    Args:
        p0, pf: shape (n_joints,) — 각 관절의 시작/끝 위치
        T: 스칼라(모든 관절 공통 이동시간) 또는 shape (n_joints,) 배열
           (관절별로 다른 이동시간을 쓰고 싶을 때)
        num_samples: 관절마다 공통으로 사용할 샘플 수. 서로 다른 T를 쓰더라도
            결과 배열의 shape을 (n_joints, num_samples)로 맞추기 위해
            각 관절을 0~T_j 구간에서 num_samples로 리샘플링한다.

    Returns:
        t:     shape (n_joints, num_samples) — 관절별 시간 배열 (T가 다르면
               각 행의 시간 스케일도 다름)
        pos, vel, accel, jerk: 각 shape (n_joints, num_samples)
    """
    p0 = np.atleast_1d(np.asarray(p0, dtype=float))
    pf = np.atleast_1d(np.asarray(pf, dtype=float))
    if p0.shape != pf.shape:
        raise ValueError("p0와 pf의 shape이 일치해야 합니다.")

    n_joints = p0.shape[0]
    T_arr = np.full(n_joints, float(T)) if np.isscalar(T) else np.asarray(T, dtype=float)
    if T_arr.shape[0] != n_joints:
        raise ValueError("T는 스칼라이거나 관절 수와 같은 길이의 배열이어야 합니다.")

    t = np.zeros((n_joints, num_samples))
    pos = np.zeros((n_joints, num_samples))
    vel = np.zeros((n_joints, num_samples))
    accel = np.zeros((n_joints, num_samples))
    jerk = np.zeros((n_joints, num_samples))

    for j in range(n_joints):
        tj, pj, vj, aj, jj = min_jerk_profile(p0[j], pf[j], T_arr[j], num_samples)
        t[j], pos[j], vel[j], accel[j], jerk[j] = tj, pj, vj, aj, jj

    return t, pos, vel, accel, jerk


def plot_profile(t: np.ndarray, pos: np.ndarray, vel: np.ndarray,
                  accel: np.ndarray, jerk: np.ndarray,
                  title: str = "Minimum Jerk Trajectory", show: bool = True):
    """pos/vel/accel/jerk 4단 그래프 시각화.

    다관절(2D 배열)이 들어와도 각 관절을 같은 축에 겹쳐 그린다.
    """
    fig, axes = plt.subplots(4, 1, figsize=(8, 10), sharex=False)

    def _plot_all(ax, t_, y_, ylabel):
        if y_.ndim == 1:
            ax.plot(t_, y_)
        else:
            for j in range(y_.shape[0]):
                ax.plot(t_[j] if t_.ndim > 1 else t_, y_[j], label=f"joint{j}")
            ax.legend(loc="best", fontsize=8)
        ax.set_ylabel(ylabel)
        ax.grid(True)

    _plot_all(axes[0], t, pos, "Position")
    _plot_all(axes[1], t, vel, "Velocity")
    _plot_all(axes[2], t, accel, "Acceleration")
    _plot_all(axes[3], t, jerk, "Jerk")
    axes[3].set_xlabel("Time [s]")
    fig.suptitle(title)
    fig.tight_layout()

    if show:
        plt.show()
    return fig


if __name__ == "__main__":
    # 단일 축 데모
    t, pos, vel, accel, jerk = min_jerk_profile(p0=0.0, pf=0.5, T=2.0)
    print(f"pos[0]={pos[0]:.4f}, pos[-1]={pos[-1]:.4f}")
    print(f"vel[0]={vel[0]:.4f}, vel[-1]={vel[-1]:.4f}")
    print(f"accel[0]={accel[0]:.4f}, accel[-1]={accel[-1]:.4f}")

    plot_profile(t, pos, vel, accel, jerk, title="Single-axis demo", show=False)

    # 다관절 데모 (예: 6축 협동로봇 조인트 이동)
    p0_j = np.array([0.0, 0.1, -0.2, 0.0, 0.3, 0.0])
    pf_j = np.array([0.4, -0.1, 0.2, 0.5, -0.1, 0.6])
    tj, posj, velj, accj, jerkj = min_jerk_joint_space(p0_j, pf_j, T=1.5)
    plot_profile(tj, posj, velj, accj, jerkj, title="Joint-space demo", show=False)

    print("min_jerk_trajectory.py self-test done (그래프는 show=False로 생성만 확인).")
