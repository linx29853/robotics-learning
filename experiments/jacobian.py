"""第三课：关节速度贡献、中心差分验证和奇异性。"""
import numpy as np
from forward_kinematics import forward_kinematics


def jacobian(q1, q2, a1=.30, a2=.25):
    return np.array([
        [-a1*np.sin(q1)-a2*np.sin(q1+q2), -a2*np.sin(q1+q2)],
        [a1*np.cos(q1)+a2*np.cos(q1+q2), a2*np.cos(q1+q2)],
    ])


def main():
    q = np.deg2rad([30., 45.])
    q_dot = np.deg2rad([10., -5.])
    J = jacobian(*q)
    print('Jacobian:\n', J)
    print('Joint 1 contribution:', J[:, 0]*q_dot[0])
    print('Joint 2 contribution:', J[:, 1]*q_dot[1])
    print('End-effector velocity (m/s):', J @ q_dot)
    # 本次整理补充：独立对每一列进行中心差分验证。
    worst = 0.
    h = 1e-6
    for angles in np.random.default_rng(42).uniform(-np.pi, np.pi, (100, 2)):
        numeric = np.column_stack([
            (forward_kinematics(*(angles+h*axis))
             - forward_kinematics(*(angles-h*axis)))/(2*h)
            for axis in np.eye(2)
        ])
        error = np.max(np.abs(numeric-jacobian(*angles)))
        assert error < 1e-8
        worst = max(worst, error)
    for q2 in [0., np.pi]:
        assert np.linalg.matrix_rank(jacobian(0., q2)) == 1
    print(f'PASS: 100 finite-difference Jacobians, max error={worst:.3e}; singular rank=1')


if __name__ == '__main__':
    main()
