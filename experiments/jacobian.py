import numpy as np
def jacobian(q1,q2,a1,a2):
    J=np.array([[-a1*np.sin(q1) - a2*np.sin(q1+q2),  -a2*np.sin(q1+q2)],
                [ a1*np.cos(q1) + a2*np.cos(q1+q2),   a2*np.cos(q1+q2)]])
    return J
q=np.deg2rad(np.array([30.0,45.0]))
q_dot=np.deg2rad([10.0,-5.0])
J=jacobian(
    q1=q[0],
    q2=q[1],
    a1=0.30,
    a2=0.25,
)

end_effector_velocity=J@q_dot

print("第一关节贡献:", J[:, 0] * q_dot[0])
print("第二关节贡献:", J[:, 1] * q_dot[1])
print("合成末端速度:", end_effector_velocity)

def forward_kinematics(q1, q2, a1, a2):
    x = a1 * np.cos(q1) + a2 * np.cos(q1 + q2)
    y = a1 * np.sin(q1) + a2 * np.sin(q1 + q2)
    return np.array([x, y])

a1 = 0.30
a2 = 0.25
dt = 1e-6  # 秒

position_before = forward_kinematics(
    q[0], q[1], a1, a2
)
q_next = q + q_dot * dt
position_after = forward_kinematics(
    q_next[0], q_next[1], a1, a2
)
# 用位移除以时间，估计末端速度
velocity_numerical = (
    position_after - position_before
) / dt

error = np.linalg.norm(
    velocity_numerical - end_effector_velocity
)

print("有限差分速度:", velocity_numerical)
print("误差:", error)

assert error < 1e-6