import math

import numpy as np

from go2_occmap.control import Watchdog, scan_gate
from go2_occmap.mapping import ObservedGrid, SampledField, SparseGraph, cell, follow_path, position


def test_watchdog_rejects_nonfinite_and_expired_commands():
    watchdog = Watchdog()
    assert np.all(watchdog.value(1) == 0)
    watchdog.update([.2, 0, .3], 2)
    assert watchdog.value(2.1)[0] == .2
    assert np.all(watchdog.value(2.6) == 0)
    watchdog.update([math.nan, 0, 0], 3)
    assert np.all(watchdog.value(3.1) == 0)
    assert np.all(watchdog.value(2) == 0)


def test_forward_stop_and_turning():
    ranges = np.full(180, np.inf)
    ranges[90] = .5
    assert scan_gate([.2, 0, .3], ranges).tolist() == [0., 0., .3]
    assert np.all(scan_gate([.2, 0, 0], []) == 0)


def test_scan_updates_observed_free_and_hit_only():
    grid = ObservedGrid()
    grid.integrate([0, 0, 0], [1.], angle_min=0, angle_increment=1)
    occupancy = grid.occupancy()
    assert occupancy[tuple(cell([.5, 0]))] == 0
    assert occupancy[tuple(cell([1., 0]))] == 100
    assert occupancy[tuple(cell([2., 0]))] == -1


def test_graph_does_not_connect_through_unknown_or_wall():
    occupancy = np.zeros((80, 100), dtype=int)
    occupancy[:, 48:53] = -1
    graph = SparseGraph(occupancy)
    left, right = graph.anchor([-2., 0.]), graph.anchor([2., 0.])
    costs, _, _ = graph.search(left)
    assert np.isinf(costs[right])
    occupancy[32:48, 48:53] = 0
    graph = SparseGraph(occupancy)
    left, right = graph.anchor([-2., 0.]), graph.anchor([2., 0.])
    costs, _, parent = graph.search(left)
    assert np.isfinite(costs[right])
    route = graph.path(parent, left, right)
    assert len(route) > 2
    assert all(graph.clear_segment(cell(a), cell(b)) for a, b in zip(route, route[1:]))


def test_risk_changes_path_away_from_high_rate_band():
    occupancy = np.zeros((80, 100), dtype=int)
    graph = SparseGraph(occupancy)
    start, goal = graph.anchor([-2, 0]), graph.anchor([2, 0])
    rates = np.where((abs(graph.xy[:, 0]) < .8) & (abs(graph.xy[:, 1]) < .8), 1000., 1.)
    _, _, geometric = graph.search(start)
    _, _, weighted = graph.search(start, rates, radiation_weight=2.)
    a, b = graph.path(geometric, start, goal), graph.path(weighted, start, goal)
    assert np.max(abs(b[:, 1])) > np.max(abs(a[:, 1]))+.4


def test_gp_is_positive_bounded_and_uncertain_far_from_samples():
    field = SampledField(capacity=8)
    for i in range(30):
        field.add([i*.3, 0.], 5.)
    assert len(field.xy) == 8
    mean, upper = field.predict([[8.7, 0.], [8.7, 8.]])
    assert (mean > 0).all() and (upper >= mean).all()
    assert upper[1]/mean[1] > upper[0]/mean[0]


def test_route_follower_turns_before_forward_motion():
    assert follow_path([0, 0, 0], np.array([[0, 1]]))[0] == 0
    command = follow_path([0, 0, 0], np.array([[1, 0]]))
    assert 0 < command[0] <= .22 and abs(command[2]) < 1e-9
    assert np.all(follow_path([0, 0, 0], []) == 0)


def test_cell_centers_round_trip():
    cells = np.array([[0, 0], [30, 70], [79, 99]])
    np.testing.assert_array_equal(cell(position(cells)), cells)


def test_route_exposure_integrates_rate_times_travel_time():
    graph = SparseGraph(np.zeros((80, 100), dtype=int))
    start, goal = graph.anchor([-2., 0.]), graph.anchor([2., 0.])
    _, lengths, parent = graph.search(start)
    dose = graph.exposure(parent, lengths, start, np.full(len(graph.xy), 36.), speed=.1)
    assert np.isclose(dose[goal], lengths[goal]/.1*36./3600)


def test_local_reconnection_preserves_minimum_clearance_and_rejects_unknown():
    occupancy = np.zeros((80, 100), dtype=int)
    occupancy[:, 50] = 100
    graph = SparseGraph(occupancy)
    xy = position([40, 45])  # 0.5 m from wall: outside nominal 0.60 m margin.
    assert graph.anchor(xy) is None
    anchor = graph.anchor(xy, recovery=True)
    assert anchor is not None
    assert graph.recovery_segment(cell(xy), graph.cells[anchor])
    assert not graph.recovery_segment([40, 49], graph.cells[anchor])
    occupancy[40, 45] = -1
    assert SparseGraph(occupancy).anchor(xy, recovery=True) is None
