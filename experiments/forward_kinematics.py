"""第一课：标准 DH 正运动学与几何公式交叉验证。"""
import numpy as np


def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0., sa, ca, d],
        [0., 0., 0., 1.],
    ])


def forward_kinematics_dh(q1, q2, a1=.30, a2=.25):
    return dh_transform(q1, 0, a1, 0) @ dh_transform(q2, 0, a2, 0)


def forward_kinematics(q1, q2, a1=.30, a2=.25):
    return np.array([a1*np.cos(q1)+a2*np.cos(q1+q2),
                     a1*np.sin(q1)+a2*np.sin(q1+q2)])


def main():
    q = np.deg2rad([30., 45.])
    print('T02:\n', forward_kinematics_dh(*q))
    print('position (m):', forward_kinematics(*q))
    worst = 0.
    for angles in np.random.default_rng(42).uniform(-np.pi, np.pi, (100, 2)):
        error = np.linalg.norm(forward_kinematics_dh(*angles)[:2, 3]
                               - forward_kinematics(*angles))
        assert error < 1e-12
        worst = max(worst, error)
    print(f'PASS: 100 FK comparisons, max error={worst:.3e}')


if __name__ == '__main__':
    main()
