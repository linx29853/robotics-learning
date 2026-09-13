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

def forward_kinematics(q1, q2, a1, a2):
    x = a1 * np.cos(q1) + a2 * np.cos(q1 + q2)
    y = a1 * np.sin(q1) + a2 * np.sin(q1 + q2)
    return np.array([x, y])


def inverse_kinematics_numerical(
        target, q_init, a1, a2,learning_rate=0.5, tolerance=1e-6,max_iterations=200,
):
    target=np.asarray(target,dtype=np.float64)
    q = np.asarray(q_init, dtype=np.float64,copy=True)
    max_step=0.1
    for iteration in range(max_iterations):
        position=forward_kinematics(q[0],q[1],a1,a2)
        error=target-position
        error_norm = np.linalg.norm(error)
        if iteration % 10 == 0:
            print(
                f"迭代 {iteration}: "
                f"位置误差 = {error_norm:.8f} m"
            )
        if error_norm < tolerance:
            return q,True,iteration
        if iteration==max_iterations-1:
            break
        J=jacobian(q[0],q[1],a1,a2)
        delta_q = learning_rate * (np.linalg.pinv(J) @ error)
        step_norm=np.linalg.norm(delta_q)
        if step_norm>max_step:
            delta_q=delta_q/step_norm*max_step
        q=q+delta_q
    return q,False,iteration

if __name__ == "__main__":
    a1 = 0.30
    a2 = 0.25

    target = np.array([0.20, 0.40])
    # 先从非奇异姿态开始
    q_initial = np.deg2rad([20.0, 60.0])

    q_solution, success, iterations = (
        inverse_kinematics_numerical(
            target=target,
            q_init=q_initial,
            a1=a1,
            a2=a2,
        )
    )

    # 用正运动学回代最终结果
    actual_position = forward_kinematics(
        q_solution[0],
        q_solution[1],
        a1,
        a2,
    )

    final_error = np.linalg.norm(
        target - actual_position
    )

    print("是否收敛:", success)
    print("更新次数:", iterations)
    print("关节角（度）:", np.rad2deg(q_solution))
    print("目标位置:", target)
    print("实际位置:", actual_position)
    print("最终误差:", final_error)

    assert success, "算法未收敛"
    assert final_error < 1e-6

