"""第二课：两组解析逆解、可达性与正运动学回代。"""
import numpy as np
from forward_kinematics import forward_kinematics


def solve_q2(x, y, a1, a2):
    if not np.all(np.isfinite([x, y, a1, a2])) or min(a1, a2) <= 0:
        raise ValueError('输入须有限，连杆长度须为正')
    c2 = (x*x+y*y-a1*a1-a2*a2)/(2*a1*a2)
    if c2 < -1-1e-12 or c2 > 1+1e-12:
        raise ValueError('目标点不可达')
    c2 = np.clip(c2, -1., 1.)  # 必须在 sqrt 之前处理浮点误差
    s2 = np.sqrt(max(0., 1-c2*c2))
    return np.arctan2(s2, c2), np.arctan2(-s2, c2)


def solve_q1(x, y, a1, a2, q2):
    return np.arctan2(y, x)-np.arctan2(a2*np.sin(q2), a1+a2*np.cos(q2))


def inverse_kinematics(x, y, a1=.30, a2=.25):
    """一般返回两个分支；等长杆原点只返回代表解，不枚举无穷解。"""
    return [(solve_q1(x, y, a1, a2, q2), q2)
            for q2 in solve_q2(x, y, a1, a2)]


def main():
    for target in [forward_kinematics(*np.deg2rad([30, 45])), [.55, 0], [.05, 0]]:
        for solution in inverse_kinematics(*target):
            error = np.linalg.norm(forward_kinematics(*solution)-target)
            assert error < 1e-9
            print('target:', target, 'q(deg):', np.rad2deg(solution), 'error:', error)
    for target in [[.60, 0], [0, 0]]:
        try:
            inverse_kinematics(*target)
        except ValueError:
            print('Correctly rejected:', target)
        else:
            raise AssertionError(f'Unreachable point accepted: {target}')
    print('PASS: IK branches, boundaries and unreachable points')


if __name__ == '__main__':
    main()
