"""ROS-independent observed-grid mapping, sparse graph and sampled-field GP.

This is a new simulation baseline, not a recovered historical ROS 2 algorithm.
"""
import heapq
import math

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.ndimage import distance_transform_edt, uniform_filter

RESOLUTION = .1
ORIGIN = np.array([-5., -4.])
SHAPE = (80, 100)


def cell(xy):
    return np.floor((np.asarray(xy)-ORIGIN)/RESOLUTION).astype(int)[..., ::-1]


def position(rc):
    return ORIGIN+(np.asarray(rc)[..., ::-1]+.5)*RESOLUTION


def line_cells(a, b):
    a, b = np.asarray(a), np.asarray(b)
    count = int(np.max(np.abs(b-a)))+1
    return np.rint(np.linspace(a, b, count)).astype(int)


class ObservedGrid:
    def __init__(self):
        self.log_odds = np.zeros(SHAPE, dtype=np.float32)
        self.seen = np.zeros(SHAPE, dtype=bool)

    def integrate(self, pose, ranges, angle_min=-math.pi, angle_increment=2*math.pi/180, max_range=3.5):
        start = cell(pose[:2])
        for i, distance in enumerate(ranges):
            if np.isnan(distance) or distance < .05:
                continue
            hit = np.isfinite(distance) and distance <= max_range
            distance = min(distance, max_range)
            angle = pose[2]+angle_min+i*angle_increment
            end = cell(np.asarray(pose[:2])+distance*np.array([math.cos(angle), math.sin(angle)]))
            cells = line_cells(start, end)
            valid = (cells[:, 0] >= 0) & (cells[:, 0] < SHAPE[0]) & (cells[:, 1] >= 0) & (cells[:, 1] < SHAPE[1])
            free = cells[:-1] if hit else cells
            free_valid = valid[:-1] if hit else valid
            free = free[free_valid]
            self.seen[free[:, 0], free[:, 1]] = True
            self.log_odds[free[:, 0], free[:, 1]] -= .35
            if hit and valid[-1]:
                self.seen[end[0], end[1]] = True
                self.log_odds[end[0], end[1]] += 1.6
        np.clip(self.log_odds, -4., 4., out=self.log_odds)

    def occupancy(self):
        return np.where(self.seen, np.where(self.log_odds > 0, 100, 0), -1).astype(np.int8)


class SampledField:
    """Bounded recent-sample log-Gaussian regression, not a calibrated dose guarantee."""
    def __init__(self, capacity=64):
        self.capacity, self.xy, self.y = capacity, [], []

    def add(self, xy, rate):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all() or not np.isfinite(rate) or rate < 0:
            return
        # Spatial aggregation avoids filling the whole dictionary while standing.
        if self.xy and np.linalg.norm(xy-self.xy[-1]) < .2:
            self.y[-1] = .8*self.y[-1]+.2*math.log(max(rate, .05))
            return
        self.xy.append(xy.copy())
        self.y.append(math.log(max(rate, .05)))
        self.xy, self.y = self.xy[-self.capacity:], self.y[-self.capacity:]

    @staticmethod
    def kernel(a, b):
        d = np.sqrt(np.sum(((a[:, None]-b[None, :])/1.2)**2, axis=-1))*math.sqrt(3)
        return (1+d)*np.exp(-d)

    def predict(self, query):
        query = np.asarray(query).reshape(-1, 2)
        prior = math.log(8.)
        if not self.xy:
            return np.full(len(query), 8.), np.full(len(query), 8.*math.exp(2))
        x = np.asarray(self.xy)
        factor = cho_factor(self.kernel(x, x)+np.eye(len(x))*.15, lower=True)
        k = self.kernel(query, x)
        mean = prior+k@cho_solve(factor, np.asarray(self.y)-prior)
        variance = np.maximum(0., 1.-np.sum(k*cho_solve(factor, k.T).T, axis=1))
        return np.exp(np.clip(mean, -5, 10)), np.exp(np.clip(mean+2*np.sqrt(variance), -5, 10))


class SparseGraph:
    """Regular spatial sampling of inflated observed free space with checked edges."""
    def __init__(self, occupancy, footprint=.60, stride=4):
        self.grid = np.asarray(occupancy)
        self.clearance = distance_transform_edt(self.grid == 0)*RESOLUTION
        self.safe = self.clearance >= footprint
        mask = np.zeros_like(self.safe)
        mask[::stride, ::stride] = True
        self.cells = np.argwhere(self.safe & mask)
        self.xy = position(self.cells)
        self.edges = [[] for _ in self.cells]
        lookup = {tuple(rc): i for i, rc in enumerate(self.cells)}
        for i, rc in enumerate(self.cells):
            for dr, dc in ((stride, 0), (0, stride), (stride, stride), (stride, -stride)):
                j = lookup.get(tuple(rc+[dr, dc]))
                if j is not None and self.clear_segment(rc, self.cells[j]):
                    distance = float(np.linalg.norm(self.xy[i]-self.xy[j]))
                    self.edges[i].append((j, distance))
                    self.edges[j].append((i, distance))
        self.information = uniform_filter((self.grid < 0).astype(float), size=23, mode='constant')

    def clear_segment(self, a, b):
        points = line_cells(a, b)
        valid = (points >= 0).all() and (points < self.safe.shape).all()
        return bool(valid and self.safe[points[:, 0], points[:, 1]].all())

    def recovery_segment(self, a, b):
        """Local reconnection margin for policy drift; never crosses unknown cells.

        Global routes retain the nominal 0.60 m clearance. Reconnection requires
        >=0.50 m and cannot reduce the current clearance by more than 0.02 m.
        """
        points = line_cells(a, b)
        if not (points >= 0).all() or not (points < self.safe.shape).all():
            return False
        clearance = self.clearance[points[:, 0], points[:, 1]]
        return bool((self.grid[points[:, 0], points[:, 1]] == 0).all()
                    and clearance.min() >= max(.50, clearance[0]-.02))

    def anchor(self, xy, recovery=False):
        for i in np.argsort(np.linalg.norm(self.xy-np.asarray(xy), axis=1))[:12]:
            if np.linalg.norm(self.xy[i]-xy) < .9 and (
                    self.clear_segment(cell(xy), self.cells[i]) or
                    (recovery and self.recovery_segment(cell(xy), self.cells[i]))):
                return int(i)
        return None

    def search(self, start, rates=None, radiation_weight=0.):
        costs = np.full(len(self.xy), np.inf)
        length = np.full(len(self.xy), np.inf)
        parent = np.full(len(self.xy), -1, dtype=int)
        costs[start], length[start] = 0., 0.
        queue = [(0., start)]
        while queue:
            cost, i = heapq.heappop(queue)
            if cost > costs[i]:
                continue
            for j, distance in self.edges[i]:
                risk = 0. if rates is None else .5*(rates[i]+rates[j])/20.
                candidate = cost+distance*(1+radiation_weight*risk)
                if candidate < costs[j]:
                    costs[j], length[j], parent[j] = candidate, length[i]+distance, i
                    heapq.heappush(queue, (candidate, j))
        return costs, length, parent

    def path(self, parent, start, goal):
        route, current = [], goal
        while current != start:
            if current < 0 or len(route) > len(parent):
                raise ValueError('Disconnected graph path')
            route.append(current)
            current = int(parent[current])
        return self.xy[[start, *reversed(route)]]

    def exposure(self, parent, lengths, start, upper_rates, speed=.12):
        """Trapezoidal upper-rate integration along each route in a search tree."""
        dose = np.full(len(self.xy), np.inf)
        dose[start] = 0.
        for i in np.argsort(lengths):
            previous = int(parent[i])
            if previous >= 0:
                distance = np.linalg.norm(self.xy[i]-self.xy[previous])
                dose[i] = dose[previous]+distance*(upper_rates[i]+upper_rates[previous])/(2*speed*3600)
        return dose


def follow_path(pose, path, speed=.22):
    if len(path) == 0:
        return np.zeros(3)
    target = path[0]
    for candidate in path:
        target = candidate
        if np.linalg.norm(target-pose[:2]) > .3:
            break
    delta = target-pose[:2]
    error = math.atan2(math.sin(math.atan2(delta[1], delta[0])-pose[2]),
                       math.cos(math.atan2(delta[1], delta[0])-pose[2]))
    forward = min(speed, .6*np.linalg.norm(delta))*max(0., math.cos(error)) if abs(error) < .65 else 0.
    return np.array([forward, 0., np.clip(1.1*error, -.55, .55)])
