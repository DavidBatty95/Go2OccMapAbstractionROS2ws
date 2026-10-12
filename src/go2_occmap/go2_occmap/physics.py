"""Torque-driven Go2; policy contract adapted from rl_sar (Apache-2.0).

Named-joint adapter follows David Batty's Dougie-Sim baseline. See docs/ATTRIBUTION.md.
The free base is initialized once, then moved only by MuJoCo dynamics.
"""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch

from .assets import ASSETS

JOINTS = tuple(f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
               for joint in ('hip', 'thigh', 'calf'))
DEFAULT = np.tile([0., .8, -1.5], 4)
SCALE = np.tile([.125, .25, .25], 4)
RESOLUTION, ORIGIN, SHAPE = .1, np.array([-5., -4.]), (80, 100)


def rotation(q):
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, np.asarray(q, dtype=float))
    return result.reshape(3, 3)


def obstacles(world):
    boxes = [(5., 0., .1, 4.), (-5., 0., .1, 4.), (0., 4., 5., .1), (0., -4., 5., .1)]
    if world == 'partitioned':
        boxes += [(0., 2.8, .12, 1.2), (0., -2.8, .12, 1.2), (2.8, 1.8, .55, .45),
                  (-3., 2.2, .5, .45), (2.8, -2., .45, .5)]
    elif world != 'room':
        raise ValueError(f'Unknown world: {world}')
    return boxes


def scene_xml(world):
    root = ET.parse(ASSETS/'go2/go2.xml').getroot()
    root.find('compiler').set('meshdir', str(ASSETS/'go2/assets'))
    root.find('option').set('timestep', '.005')
    root.remove(root.find('keyframe'))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.7 .7 .7')
    asset = root.find('asset')
    ET.SubElement(asset, 'texture', name='grid', type='2d', builtin='checker',
                  rgb1='.13 .21 .27', rgb2='.2 .3 .36', width='256', height='256')
    ET.SubElement(asset, 'material', name='ground', texture='grid', texrepeat='10 8', texuniform='true')
    body = root.find('worldbody')
    ET.SubElement(body, 'light', pos='0 0 8', dir='0 0 -1', directional='true')
    ET.SubElement(body, 'geom', name='floor', type='plane', size='0 0 .1',
                  material='ground', friction='.8 .02 .01')
    for i, (x, y, sx, sy) in enumerate(obstacles(world)):
        ET.SubElement(body, 'geom', name=f'obstacle_{i}', type='box', pos=f'{x} {y} .65',
                      size=f'{sx} {sy} .65', rgba='.65 .72 .77 1', group='3')
    return ET.tostring(root, encoding='unicode')


class Simulation:
    def __init__(self, world='partitioned'):
        self.model = mujoco.MjModel.from_xml_string(scene_xml(world))
        self.data = mujoco.MjData(self.model)
        ids = np.array([self.model.joint(name).id for name in JOINTS])
        self.qadr, self.vadr = self.model.jnt_qposadr[ids], self.model.jnt_dofadr[ids]
        self.actuators = np.array([self.model.actuator(name.removesuffix('_joint')).id for name in JOINTS])
        self.base = self.model.body('base_link').id
        self.robot_geoms = set()
        for geom in range(self.model.ngeom):
            parent = int(self.model.geom_bodyid[geom])
            while parent and parent != self.base:
                parent = int(self.model.body_parentid[parent])
            if parent == self.base:
                self.robot_geoms.add(geom)
                if self.model.geom_group[geom] != 2:
                    self.model.geom_group[geom] = 4
        self.obstacle_geoms = {self.model.geom(f'obstacle_{i}').id for i in range(len(obstacles(world)))}
        self.data.qpos[:7] = [-2., 0., .32, 1., 0., 0., 0.]
        self.data.qpos[self.qadr] = DEFAULT
        self.action, self.target = np.zeros(12), DEFAULT.copy()
        self.counter = 0
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(ASSETS/'policy.pt'), map_location='cpu').eval()
        mujoco.mj_forward(self.model, self.data)

    def step(self, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError('Three finite command components required')
        command = np.clip(command, [-.25, -.2, -.65], [.35, .2, .65])
        if self.data.time < 1.:
            command[:] = 0.
        if self.counter % 4 == 0:
            if self.data.time >= .5:
                R = rotation(self.data.qpos[3:7])
                observation = np.concatenate((self.data.qvel[3:6]*.25, R.T@np.array([0., 0., -1.]),
                    command, self.data.qpos[self.qadr]-DEFAULT, self.data.qvel[self.vadr]*.05, self.action))
                with torch.inference_mode():
                    self.action = self.policy(torch.from_numpy(np.clip(observation, -100, 100).astype(np.float32))[None]).numpy().reshape(-1)
                if self.action.shape != (12,) or not np.isfinite(self.action).all():
                    raise RuntimeError('Invalid policy action')
                self.target = DEFAULT+SCALE*np.clip(self.action, -100, 100)
        torques = 20*(self.target-self.data.qpos[self.qadr])-.5*self.data.qvel[self.vadr]
        self.data.ctrl[self.actuators] = np.clip(torques, -23.5, 23.5)
        mujoco.mj_step(self.model, self.data)
        self.counter += 1
        if not np.isfinite(self.data.qpos).all() or self.data.qpos[2] < .13 or self.upright < .45:
            raise RuntimeError('Go2 fell or physics state became invalid')

    @property
    def upright(self):
        return float(rotation(self.data.qpos[3:7])[2, 2])

    @property
    def pose(self):
        R = rotation(self.data.qpos[3:7])
        return np.r_[self.data.qpos[:2], math.atan2(R[1, 0], R[0, 0])]

    def scan(self, count=180, max_range=3.5):
        angles = self.pose[2]+np.linspace(-math.pi, math.pi, count, endpoint=False)
        directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(count)))
        origin = self.data.qpos[:3].copy()
        origin[2] += .12
        distances, ids = np.empty(count), np.empty(count, dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, origin, directions.reshape(-1),
                          np.array([1, 1, 0, 1, 0, 1], dtype=np.uint8), 1, self.base, ids, distances, count, max_range)
        return np.where((distances > .05) & (distances <= max_range), distances, np.inf)

    def obstacle_contact(self):
        return any((int(c.geom1) in self.obstacle_geoms and int(c.geom2) in self.robot_geoms)
                   or (int(c.geom2) in self.obstacle_geoms and int(c.geom1) in self.robot_geoms)
                   for c in self.data.contact)


def true_field(xy):
    """Synthetic proxy in microSv/hour. Simulator/evaluator only."""
    xy = np.asarray(xy)
    result = np.full(xy.shape[:-1], .5)
    for location, strength in [([3., 2.6], 100.), ([-3., -2.5], 35.)]:
        result += strength/(np.sum((xy-location)**2, axis=-1)+.5)
    return result


def truth_grid(world):
    y, x = np.indices(SHAPE)
    xx, yy = ORIGIN[0]+(x+.5)*RESOLUTION, ORIGIN[1]+(y+.5)*RESOLUTION
    occupied = np.zeros(SHAPE, dtype=bool)
    for cx, cy, sx, sy in obstacles(world):
        occupied |= (abs(xx-cx) <= sx) & (abs(yy-cy) <= sy)
    return occupied
