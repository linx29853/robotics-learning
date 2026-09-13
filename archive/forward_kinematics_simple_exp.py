import numpy as np 
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

def forward_kinematics(q1,q2,a1,a2):
    x1=a1*np.cos(q1)
    y1=a1*np.sin(q1)
    x2=a2*np.cos(q1+q2)
    y2=a2*np.sin(q1+q2)
    return np.array([x1+x2,y1+y2])

p=forward_kinematics(
    q1=np.deg2rad(90),
    q2=np.deg2rad(90),
    a1=0.30,
    a2=0.25,
)
print (p)

A1 = dh_transform(np.pi / 2, 0.0, 0.30, 0.0)
A2 = dh_transform(np.pi / 2, 0.0, 0.25, 0.0)

p_elbow = A1[:3, 3]       # 肘部在世界坐标系中的位置
R01 = A1[:3, :3]          # 肘部坐标系相对于世界的方向
p_local = A2[:3, 3]       # 末端在肘部坐标系中的位置

displacement_world = R01 @ p_local

print("肘部位置:", p_elbow)
print("第二段局部位移:", p_local)
print("第二段世界位移:", displacement_world)
print("末端位置:", p_elbow + displacement_world)