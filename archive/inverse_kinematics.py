import numpy as np
def solve_q2(x,y,a1,a2):
    r_squared=x**2+y**2
    c2=(r_squared-a1**2-a2**2)/(2.0*a1*a2)
    #计算c2
    tolerance=1e-12
    if c2<-1.0-tolerance or c2>1.0+tolerance:
        raise ValueError("目标点不可达")  
    c2=np.clip(c2,-1.0,1.0)
    s2_posi=np.sqrt(1.0-c2**2)
    s2_nega=-s2_posi
    #计算正负解
    q2_posi=np.arctan2(s2_posi,c2)
    q2_nega=np.arctan2(s2_nega,c2)
    #计算q2
    #判断可达性
    return q2_posi, q2_nega

def solve_q1(x,y,a1,a2,q2):
    target_angle=np.arctan2(y,x)
    correction_angle=np.arctan2(
        a2*np.sin(q2),
        a1+a2*np.cos(q2),
    )
    return target_angle-correction_angle

def inverse_kinematics(x,y,a1,a2):
    q2_posi, q2_nega = solve_q2(x,y,a1,a2)   
    #print("q2 positive:", np.rad2deg(q2_posi))
    #print("q2 negative:", np.rad2deg(q2_nega))
    q1_posi=solve_q1(x,y,a1,a2,q2_posi)
    q1_nega=solve_q1(x,y,a1,a2,q2_nega)
    #计算q1
    #print("q1 positive:", np.rad2deg(q1_posi))
    #print("q1 negative:", np.rad2deg(q1_nega))
    return [(q1_posi, q2_posi), (q1_nega, q2_nega)] 

def forward_kinematics(q1,q2,a1,a2):
    x1=a1*np.cos(q1)
    y1=a1*np.sin(q1)
    x2=a2*np.cos(q1+q2)
    y2=a2*np.sin(q1+q2)
    return np.array([x1+x2,y1+y2])



if __name__ == "__main__":
    target = np.array([0.32451238, 0.39148146])

    solutions = inverse_kinematics(
        x=target[0],
        y=target[1],
        a1=0.30,
        a2=0.25,
    )

    for index, solution in enumerate(solutions, start=1):
        q1, q2 = solution

        reconstructed = forward_kinematics(
            q1=q1,
            q2=q2,
            a1=0.30,
            a2=0.25,
        )

        error = np.linalg.norm(reconstructed - target)

        print(f"解 {index}:")
        print("  关节角（度）:", np.rad2deg(solution))
        print("  回代位置:", reconstructed)
        print("  回代误差:", error)

        assert np.allclose(
            reconstructed,
            target,
            atol=1e-8,
        )

def test(test_targets,a1,a2):
    for target in test_targets:
        print("\n目标点:", target)
        try:
            solutions = inverse_kinematics(
                x=target[0],
                y=target[1],
                a1=a1,
                a2=a2,
            )
        except ValueError as e:
            print("错误:", e)
            continue
        for index, solution in enumerate(solutions, start=1):
                q1, q2 = solution
                reconstructed = forward_kinematics(
                    q1=q1,
                    q2=q2,
                    a1=a1,
                    a2=a2,
                )
                error = np.linalg.norm(reconstructed - target)
                print(f"解 {index}:")
                print("  关节角（度）:", np.rad2deg(solution))
                print("  回代位置:", reconstructed)
                print("  回代误差:", error)
                assert error < 1e-9  
    
test_targets = [
    np.array([0.55, 0.0]),  # 外边界：手臂完全伸直
    np.array([0.05, 0.0]),  # 内边界：两杆反向折叠
    np.array([0.60, 0.0]),  # 不可达：太远
    np.array([0.00, 0.0]),  # 不可达：太近
]

if __name__ == "__main__":
    test(
            test_targets=test_targets,
            a1=0.30,
            a2=0.25,
        )
