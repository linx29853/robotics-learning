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

print("Jacobian:")
print(J)
print("末端速度:", end_effector_velocity)

print("第一关节贡献:", J[:, 0] * q_dot[0])
print("第二关节贡献:", J[:, 1] * q_dot[1])
print("合成末端速度:", J @ q_dot)