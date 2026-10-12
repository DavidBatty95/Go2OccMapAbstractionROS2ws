import numpy as np
import pytest

from go2_occmap.physics import JOINTS, Simulation, true_field


def test_policy_named_joints_and_untethered_walk():
    sim = Simulation('room')
    assert len(sim.actuators) == len(set(sim.actuators)) == len(JOINTS) == 12
    assert sim.model.neq == 0
    start = sim.data.qpos[:2].copy()
    for _ in range(1400):
        sim.step([.25, 0, 0])
    assert np.linalg.norm(sim.data.qpos[:2]-start) > .5
    assert sim.upright > .9
    assert .2 < sim.data.qpos[2] < .5
    assert not sim.obstacle_contact()


def test_room_scan_and_invalid_control():
    sim = Simulation('room')
    ranges = sim.scan()
    assert len(ranges) == 180
    assert 2.7 < ranges[0] < 3.1  # west wall from x=-2
    assert np.isinf(ranges[90])  # east wall beyond 3.5 m
    with pytest.raises(ValueError):
        sim.step([np.nan, 0, 0])


def test_field_is_finite_and_falls_with_distance():
    assert true_field([3., 2.6]) > true_field([0., 0.])
    assert np.isfinite(true_field([[3., 2.6], [-3., -2.5]])).all()
