"""Planar 2R kinematics: arrows and velocity ellipse in MuJoCo."""
import argparse
import queue
import time

import mujoco
import mujoco.viewer
import numpy as np

XML = '''<mujoco model="Jacobian and singularity">
  <compiler angle="radian"/>
  <option gravity="0 0 0"/>
  <visual><global offwidth="1000" offheight="800"/></visual>
  <worldbody>
    <light pos="0 0 2"/>
    <geom type="plane" size="1 1 .01" rgba=".12 .15 .20 1"/>
    <body pos="0 0 .08">
      <joint name="q1" type="hinge" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0 0 .30 0 0" size=".018" rgba=".95 .55 .16 1"/>
      <geom type="sphere" size=".025" rgba=".95 .25 .25 1"/>
      <body pos=".30 0 0">
        <joint name="q2" type="hinge" axis="0 0 1"/>
        <geom type="capsule" fromto="0 0 0 .25 0 0" size=".014" rgba=".30 .65 .95 1"/>
        <geom type="sphere" size=".021" rgba=".3 .5 1 1"/>
        <site name="tip" pos=".25 0 0" size=".023" rgba="1 1 1 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>'''


def jacobian(q):
    q1, q2 = q
    s, c = np.sin(q1 + q2), np.cos(q1 + q2)
    return np.array([[-.30*np.sin(q1)-.25*s, -.25*s],
                     [.30*np.cos(q1)+.25*c, .25*c]])


def draw(scene, kind, start, end, width, color):
    if np.linalg.norm(end-start) < 1e-10:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, kind, np.zeros(3), np.zeros(3),
                      np.eye(3).ravel(), np.array(color, dtype=np.float32))
    mujoco.mjv_connector(geom, kind, width, start, end)
    scene.ngeom += 1


def decorations(scene, tip, J):
    scene.ngeom = 0
    origin = tip.copy()
    origin[2] += .035
    # All velocity vectors use the same 0.4 s display scale.
    for column, color in zip(J.T, [(1, .2, .2, 1), (.2, .5, 1, 1)]):
        draw(scene, mujoco.mjtGeom.mjGEOM_ARROW, origin,
             origin + np.r_[.4*column, 0], .009, color)
    angles = np.linspace(0, 2*np.pi, 97)
    ellipse = .4 * J @ np.array([np.cos(angles), np.sin(angles)])
    for i in range(96):
        draw(scene, mujoco.mjtGeom.mjGEOM_LINE,
             origin + np.r_[ellipse[:, i], .003],
             origin + np.r_[ellipse[:, i+1], .003], 2, (.2, 1, .45, 1))
    for end, color in [(np.array([.15, 0, .025]), (1,.4,.4,1)),
                       (np.array([0, .15, .025]), (.4,1,.4,1))]:
        draw(scene, mujoco.mjtGeom.mjGEOM_ARROW, np.array([0.,0.,.025]),
             end, .005, color)


def verify(model, data):
    rng = np.random.default_rng(42)
    worst = 0.
    for q in [np.array([0., 0.]), np.array([0., np.pi]),
              *rng.uniform(-np.pi, np.pi, (100, 2))]:
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        jp, jr = np.zeros((3, 2)), np.zeros((3, 2))
        mujoco.mj_jacSite(model, data, jp, jr, 0)
        J = jacobian(q)
        worst = max(worst, float(np.max(np.abs(jp[:2]-J))))
        np.testing.assert_allclose(jp[:2], J, atol=1e-12, rtol=0)
        expected = [.30*np.cos(q[0])+.25*np.cos(sum(q)),
                    .30*np.sin(q[0])+.25*np.sin(sum(q))]
        np.testing.assert_allclose(data.site_xpos[0, :2], expected, atol=1e-12, rtol=0)
    scene = mujoco.MjvScene(model, maxgeom=300)
    decorations(scene, data.site_xpos[0], J)
    assert scene.ngeom > 90
    print(f'PASS: 102 poses, analytic vs MuJoCo Jacobian; max error={worst:.3e}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    if args.check:
        verify(model, data)
        return
    events = queue.SimpleQueue()
    q = np.deg2rad([0., 90.])
    auto = False
    print('1: bent | 2: straight | 3: folded | 4: near singular')
    print('A/D: q1 -/+ 3 deg | S/W: q2 -/+ 3 deg | T: sweep q2')
    print('RED: J column 1 | BLUE: J column 2 | GREEN: velocity ellipse')
    print('Kinematic demonstration: prescribed poses, no torque controller.')
    with mujoco.viewer.launch_passive(model, data, key_callback=events.put,
                                     show_left_ui=False, show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [.15, .08, .08]
            viewer.cam.distance = 1.25
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -90
        last_report = 0.
        while viewer.is_running():
            while not events.empty():
                key = events.get()
                if key in (49, 50, 51, 52):
                    q[:] = np.deg2rad({49:[0,90], 50:[0,0], 51:[0,180], 52:[0,3]}[key])
                    auto = False
                elif key in (65, 68, 83, 87):
                    index, sign = {65:(0,-1), 68:(0,1), 83:(1,-1), 87:(1,1)}[key]
                    q[index] += sign*np.deg2rad(3)
                    auto = False
                elif key == 84:
                    auto = not auto
            if auto:
                q[1] += np.deg2rad(20)/60
            J = jacobian(q)
            with viewer.lock():
                data.qpos[:] = q
                data.qvel[:] = 0
                mujoco.mj_forward(model, data)
                decorations(viewer.user_scn, data.site_xpos[0], J)
            viewer.sync()
            if time.monotonic()-last_report > .5:
                s = np.linalg.svd(J, compute_uv=False)
                label = 'SINGULAR' if s[-1] < 1e-10 else 'regular'
                print(f'q(deg)={np.rad2deg(q).round(1)}  det={np.linalg.det(J):+.5f}'
                      f'  sigma_min={s[-1]:.5f}  {label}', flush=True)
                last_report = time.monotonic()
            time.sleep(1/60)


if __name__ == '__main__':
    main()
