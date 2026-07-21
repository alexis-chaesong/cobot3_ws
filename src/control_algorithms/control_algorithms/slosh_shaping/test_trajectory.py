"""
pytest 단위테스트.

실행: pytest test_trajectory.py -v
"""

import numpy as np
import pytest

from min_jerk_trajectory import min_jerk_profile, min_jerk_joint_space
from input_shaping import (
    natural_frequency,
    zv_shaper,
    zvd_shaper,
    apply_shaper,
    G,
)
from spill_quantifier import tilt_angle_response, evaluate_spill_risk


# ---------------------------------------------------------------------------
# min_jerk_trajectory 테스트
# ---------------------------------------------------------------------------
class TestMinJerk:
    def test_boundary_conditions_single_axis(self):
        p0, pf, T = 0.0, 1.2, 2.0
        t, pos, vel, accel, jerk = min_jerk_profile(p0, pf, T, num_samples=500)

        assert pos[0] == pytest.approx(p0, abs=1e-9)
        assert pos[-1] == pytest.approx(pf, abs=1e-9)
        assert vel[0] == pytest.approx(0.0, abs=1e-9)
        assert vel[-1] == pytest.approx(0.0, abs=1e-9)
        assert accel[0] == pytest.approx(0.0, abs=1e-9)
        assert accel[-1] == pytest.approx(0.0, abs=1e-9)

    def test_boundary_conditions_negative_direction(self):
        # pf < p0 인 경우(역방향 이동)도 경계조건 성립해야 함
        t, pos, vel, accel, jerk = min_jerk_profile(p0=0.5, pf=-0.3, T=1.5, num_samples=300)
        assert pos[0] == pytest.approx(0.5, abs=1e-9)
        assert pos[-1] == pytest.approx(-0.3, abs=1e-9)
        assert vel[0] == pytest.approx(0.0, abs=1e-8)
        assert vel[-1] == pytest.approx(0.0, abs=1e-8)
        assert accel[0] == pytest.approx(0.0, abs=1e-8)
        assert accel[-1] == pytest.approx(0.0, abs=1e-8)

    def test_zero_displacement_is_identically_zero(self):
        # p0 == pf 이면 전 구간 pos/vel/accel/jerk 모두 0
        t, pos, vel, accel, jerk = min_jerk_profile(p0=0.4, pf=0.4, T=1.0)
        assert np.allclose(pos, 0.4)
        assert np.allclose(vel, 0.0)
        assert np.allclose(accel, 0.0)
        assert np.allclose(jerk, 0.0)

    def test_joint_space_boundary_conditions(self):
        p0 = np.array([0.0, 0.2, -0.1])
        pf = np.array([1.0, -0.5, 0.3])
        t, pos, vel, accel, jerk = min_jerk_joint_space(p0, pf, T=1.0, num_samples=400)

        for j in range(3):
            assert pos[j, 0] == pytest.approx(p0[j], abs=1e-9)
            assert pos[j, -1] == pytest.approx(pf[j], abs=1e-9)
            assert vel[j, 0] == pytest.approx(0.0, abs=1e-9)
            assert vel[j, -1] == pytest.approx(0.0, abs=1e-9)
            assert accel[j, 0] == pytest.approx(0.0, abs=1e-9)
            assert accel[j, -1] == pytest.approx(0.0, abs=1e-9)

    def test_invalid_T_raises(self):
        with pytest.raises(ValueError):
            min_jerk_profile(p0=0.0, pf=1.0, T=0.0)


# ---------------------------------------------------------------------------
# input_shaping 테스트
# ---------------------------------------------------------------------------
class TestInputShaping:
    def test_natural_frequency_undamped(self):
        L = 0.1
        omega_n, omega_d = natural_frequency(L=L, zeta=0.0)
        assert omega_n == pytest.approx(np.sqrt(G / L))
        # zeta=0 이면 omega_d == omega_n
        assert omega_d == pytest.approx(omega_n)

    def test_natural_frequency_damped(self):
        L, zeta = 0.08, 0.1
        omega_n, omega_d = natural_frequency(L=L, zeta=zeta)
        expected_wd = omega_n * np.sqrt(1 - zeta**2)
        assert omega_d == pytest.approx(expected_wd)
        assert omega_d < omega_n

    def test_natural_frequency_invalid_inputs(self):
        with pytest.raises(ValueError):
            natural_frequency(L=-1.0, zeta=0.05)
        with pytest.raises(ValueError):
            natural_frequency(L=0.1, zeta=1.0)

    def test_zv_shaper_theoretical_values(self):
        L, zeta = 0.1, 0.05
        times, amps = zv_shaper(L=L, zeta=zeta)

        omega_n, omega_d = natural_frequency(L, zeta)
        expected_Td = np.pi / omega_d
        expected_K = np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
        expected_A1 = 1.0 / (1.0 + expected_K)
        expected_A2 = expected_K / (1.0 + expected_K)

        assert times[0] == pytest.approx(0.0)
        assert times[1] == pytest.approx(expected_Td)
        assert amps[0] == pytest.approx(expected_A1)
        assert amps[1] == pytest.approx(expected_A2)
        # 진폭 합은 항상 1 (에너지 보존/정상상태 이득 1 조건)
        assert amps.sum() == pytest.approx(1.0)
        # 진폭 비율 A2/A1 == K (요구사항에 명시된 이론값)
        assert (amps[1] / amps[0]) == pytest.approx(expected_K)

    def test_zvd_shaper_theoretical_values(self):
        L, zeta = 0.1, 0.05
        times, amps = zvd_shaper(L=L, zeta=zeta)

        omega_n, omega_d = natural_frequency(L, zeta)
        expected_Td = np.pi / omega_d
        expected_K = np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
        denom = 1 + 2 * expected_K + expected_K**2
        expected_A = np.array([1.0, 2 * expected_K, expected_K**2]) / denom

        assert times[0] == pytest.approx(0.0)
        assert times[1] == pytest.approx(expected_Td)
        assert times[2] == pytest.approx(2 * expected_Td)
        assert np.allclose(amps, expected_A)
        assert amps.sum() == pytest.approx(1.0)

    def test_zv_shaper_zero_damping_time_matches_pi_over_omega_n(self):
        # zeta=0 이면 Td = pi/omega_n (omega_d == omega_n)
        L = 0.05
        times, amps = zv_shaper(L=L, zeta=0.0)
        omega_n, _ = natural_frequency(L, zeta=0.0)
        assert times[1] == pytest.approx(np.pi / omega_n)

    def test_apply_shaper_impulse_response_reproduces_amplitudes(self):
        # 델타함수(단일 임펄스) 입력을 shaping하면, 출력은 shaper의 진폭
        # 그 자체가 시간축에 그대로 나타나야 한다.
        dt = 0.01
        profile = np.array([1.0])  # 단위 임펄스
        times, amps = zv_shaper(L=0.1, zeta=0.05)

        shaped = apply_shaper(profile, dt, times, amps)

        n_shift = int(round(times[1] / dt))
        assert shaped[0] == pytest.approx(amps[0])
        assert shaped[n_shift] == pytest.approx(amps[1])

    def test_apply_shaper_preserves_steady_state_gain(self):
        # 상수(스텝) 입력에 대해, 모든 임펄스가 겹치는 구간(정상상태)의
        # 이득은 1이어야 한다 (진폭 합이 1이므로).
        # 겹침 구간은 [마지막 임펄스 시작 인덱스, 첫 프로파일 길이-1] 이며,
        # 그 이후(shaped 배열의 맨 끝)는 마지막 임펄스 혼자만 기여하므로
        # 1이 아니라 그 임펄스의 진폭(A2)에 수렴한다.
        dt = 0.001
        N = 2000
        profile = np.ones(N)
        times, amps = zv_shaper(L=0.1, zeta=0.05)

        shaped = apply_shaper(profile, dt, times, amps)

        # 모든 임펄스가 겹치는 마지막 인덱스(N-1)는 정상상태 이득 1
        assert shaped[N - 1] == pytest.approx(1.0, abs=1e-6)
        # shaped 배열의 맨 끝은 마지막 임펄스만 기여 -> 그 임펄스 진폭에 수렴
        assert shaped[-1] == pytest.approx(amps[-1], abs=1e-6)


# ---------------------------------------------------------------------------
# spill_quantifier 테스트
# ---------------------------------------------------------------------------
class TestSpillQuantifier:
    def test_zero_acceleration_gives_zero_theta(self):
        t = np.linspace(0, 2.0, 200)
        accel = np.zeros_like(t)

        t_out, theta_deg, theta_dot = tilt_angle_response(
            t, accel, theta0=0.0, theta_dot0=0.0
        )

        assert np.allclose(theta_deg, 0.0, atol=1e-9)
        assert np.allclose(theta_dot, 0.0, atol=1e-9)

    def test_zero_acceleration_nonzero_initial_condition_decays(self):
        # 초기각이 있어도 가속 입력이 없으면 감쇠 진동 후 0으로 수렴해야 함
        t = np.linspace(0, 20.0, 4000)
        accel = np.zeros_like(t)

        t_out, theta_deg, theta_dot = tilt_angle_response(
            t, accel, theta0=np.radians(5.0), theta_dot0=0.0
        )

        assert abs(theta_deg[-1]) < 0.1  # 충분히 감쇠되어 0 근처

    def test_evaluate_spill_risk_no_exceedance(self):
        t = np.linspace(0, 1.0, 100)
        theta_deg = np.zeros_like(t)  # 항상 0 -> 임계각 절대 초과 안 함

        summary = evaluate_spill_risk(t, theta_deg, critical_deg=15.0)

        assert summary["max_angle_deg"] == pytest.approx(0.0)
        assert not summary["exceed_mask"].any()
        assert summary["exceed_intervals"] == []

    def test_evaluate_spill_risk_detects_exceedance(self):
        t = np.linspace(0, 1.0, 100)
        theta_deg = np.zeros_like(t)
        theta_deg[40:60] = 20.0  # 임계각(15도) 초과 구간 인위 삽입

        summary = evaluate_spill_risk(t, theta_deg, critical_deg=15.0)

        assert summary["max_angle_deg"] == pytest.approx(20.0)
        assert summary["exceed_mask"].sum() == 20
        assert len(summary["exceed_intervals"]) == 1


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
