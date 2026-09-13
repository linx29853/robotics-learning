import numpy as np
#DH参数法
##矩阵旋转
def dh_transform(theta,d,a,alpha):
    ct=np.cos(theta)
    st=np.sin(theta)
    ca=np.cos(alpha)
    sa=np.sin(alpha)
    return np.array([
        [ct,-st*ca,st*sa,a*ct],
        [st,ct*ca,-ct*sa,a*st],
        [0.0,sa,ca,d],
        [0.0,0.0,0.0,1.0],
    ],dtype=float)
##DH参数法正运动学
def forward_kinematics_dh(q1,q2,a1,a2):
    A1=dh_transform(
        theta=q1,
        d=0.0,
        a=a1,
        alpha=0.0,
    )
    A2=dh_transform(
        theta=q2,
        d=0.0,
        a=a2,
        alpha=0.0,
    )
    T02=A1@A2  
    return T02

#正运动学比较
def forward_kinematics_closed_form(q1, q2, a1, a2):
    x = a1 * np.cos(q1) + a2 * np.cos(q1 + q2)
    y = a1 * np.sin(q1) + a2 * np.sin(q1 + q2)

    return np.array([x, y])

#参数写入
if __name__ == "__main__":
    A = dh_transform(
        theta=np.pi/2,
        d=0.0,
        a=0.30,
        alpha=0.0,
    )

    print(A)

if __name__ == "__main__":
    q1 = np.deg2rad(30.0)
    q2 = np.deg2rad(45.0)

    T02 = forward_kinematics_dh(
        q1=q1,
        q2=q2,
        a1=0.30,
        a2=0.25,
    )

    position = T02[:2, 3]

position_closed_form = forward_kinematics_closed_form(
    q1=q1,
    q2=q2,
    a1=0.30,
    a2=0.25,
)

error = np.linalg.norm(position - position_closed_form)

#100组检验
def verify_random_configurations():
    rng = np.random.default_rng(seed=42)

    a1 = 0.30
    a2 = 0.25
    max_error = 0.0

    for _ in range(100):
        q1, q2 = rng.uniform(
            low=-np.pi,
            high=np.pi,
            size=2,
        )

        T02 = forward_kinematics_dh(q1, q2, a1, a2)
        position_dh = T02[:2, 3]

        position_closed_form = forward_kinematics_closed_form(
            q1, q2, a1, a2
        )

        error = np.linalg.norm(
            position_dh - position_closed_form
        )

        max_error = max(max_error, error)

        assert np.allclose(
            position_dh,
            position_closed_form,
            atol=1e-12,
        )

    print("PASS: 100组随机姿态全部通过")
    print("最大误差:", max_error)

verify_random_configurations()

#——————————————————————————————————————————————————————————#
print("DH矩阵计算结果:")
print(position)
print("解析公式计算结果:")
print(position_closed_form)
print("两种方法的误差:")
print(error)
print("T02:")
print(T02)
print("末端位置:", position)