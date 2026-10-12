#!/usr/bin/env python3
"""Score retained ROS/physics artifacts and render actual-state replays headlessly."""
import argparse
import hashlib
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from go2_occmap.mapping import SampledField, SparseGraph
from go2_occmap.physics import scene_xml

BACKGROUND = '#0b1724'
PALETTE = ListedColormap(['#152838', '#edf2f5', '#748c9b'])


def draw_map(axis, grid, trajectory, graph=None, route=None):
    indexed = np.where(grid < 0, 0, np.where(grid == 0, 1, 2))
    axis.imshow(indexed, origin='lower', extent=(-5, 5, -4, 4), cmap=PALETTE, vmin=0, vmax=2)
    if graph is not None:
        edges = [[graph.xy[i], graph.xy[j]] for i, neighbours in enumerate(graph.edges) for j, _ in neighbours if j > i]
        if edges:
            axis.add_collection(LineCollection(edges, colors='#2babb9', linewidths=.35, alpha=.7))
        if len(graph.xy):
            axis.scatter(*graph.xy.T, c='#2b9d98', s=3)
    if len(trajectory):
        axis.plot(trajectory[:, 0], trajectory[:, 1], color='#2872db', lw=2)
        axis.scatter(*trajectory[-1], s=35, color='#ffaf49', edgecolor='#16334c', zorder=5)
    if route is not None and len(route):
        route = np.asarray(route)
        axis.plot(route[:, 0], route[:, 1], color='#e07324', lw=1.5, ls='--')
    axis.scatter(-2, 0, marker='s', s=25, color='#783fc4')
    axis.set(xlim=(-5, 5), ylim=(-4, 4), xlabel='x / m', ylabel='y / m', aspect='equal')


def gif(frames, target, duration=100):
    palette = frames[len(frames)//2].quantize(colors=96)
    frames = [f.quantize(palette=palette, dither=0) for f in frames]
    frames[0].save(target, save_all=True, append_images=frames[1:], duration=duration,
                   loop=0, optimize=True, disposal=1)


def render(output, config, physics, mapping, trace, summary):
    trajectory = physics['qpos'][:, :2]
    graph = SparseGraph(mapping['grid'])
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), layout='constrained')
    draw_map(axes[0], mapping['grid'], trajectory, graph)
    axes[0].set_title('Observed occupancy + abstraction graph')
    samples = np.load(output/'field_samples.npz')
    field = SampledField()
    field.xy, field.y = list(samples['xy']), list(samples['log_rate'])
    if len(graph.xy):
        mean, upper = field.predict(graph.xy)
        axes[1].set_facecolor(BACKGROUND)
        color = axes[1].scatter(*graph.xy.T, c=mean, s=18, cmap='inferno', vmin=0,
                               vmax=max(20, float(np.percentile(mean, 95))))
        fig.colorbar(color, ax=axes[1], label='Predicted synthetic rate / µSv/h')
        axes[1].plot(trajectory[:, 0], trajectory[:, 1], color='#55c9df', lw=1)
        axes[1].set(xlim=(-5, 5), ylim=(-4, 4), aspect='equal', title='Final sampled-field GP mean', xlabel='x / m', ylabel='y / m')
    axes[2].plot(physics['time'], physics['dose'], label='Actual evaluator exposure')
    measurements = np.load(output/'measurements.npz')['values']
    if len(measurements):
        axes[2].plot(measurements[:, 0], measurements[:, 4], label='Count-derived estimate', ls='--')
    axes[2].axhline(config['dose_budget'], c='#ad3f5a', ls=':', label='Budget')
    axes[2].set(xlabel='Simulation time / s', ylabel='Synthetic dose / µSv', title='Exposure and return outcome')
    axes[2].legend(fontsize=8)
    fig.suptitle(f"{config['world']} / seed {config['seed']} / {summary['outcome']} / ideal pose and scans")
    fig.savefig(output/'report.png', dpi=120)
    plt.close(fig)
    frames = []
    end = float(physics['time'][-1])
    for stamp in np.linspace(float(mapping['time'][0]), end, 60):
        index = max(0, np.searchsorted(mapping['time'], stamp, side='right')-1)
        grid = mapping['snapshots'][index]
        travelled = trajectory[physics['time'] <= stamp]
        decision = next((t for t in reversed(trace) if t['time_s'] <= stamp), {})
        fig, ax = plt.subplots(figsize=(7.2, 4.8), dpi=100)
        fig.subplots_adjust(left=.10, right=.98, bottom=.16, top=.91)
        draw_map(ax, grid, travelled, SparseGraph(grid), decision.get('path'))
        ax.set_title(f"Recorded observations / {stamp:.1f} s / {decision.get('state', 'initializing')}", fontsize=11)
        fig.text(.5, .01, 'Dark = unknown / teal = graph / blue = actual trajectory / accelerated replay', ha='center', fontsize=8)
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
        plt.close(fig)
    gif(frames, output/'mapping.gif')
    model = mujoco.MjModel.from_xml_string(scene_xml(config['world']))
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.distance, camera.elevation, camera.azimuth = 2.4, -25., 135.
    moving = np.flatnonzero(np.linalg.norm(trajectory-trajectory[0], axis=1) > .15)
    start = int(moving[0]) if len(moving) else 0
    indices = np.linspace(start, min(start+119, len(trajectory)-1), 60, dtype=int)
    frames = []
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 15)
    with mujoco.Renderer(model, height=340, width=640) as renderer:
        for index in indices:
            data.qpos[:] = physics['qpos'][index]
            mujoco.mj_forward(model, data)
            camera.lookat[:] = data.qpos[:3]
            renderer.update_scene(data, camera=camera)
            image = Image.new('RGB', (640, 400), BACKGROUND)
            image.paste(Image.fromarray(renderer.render()), (0, 30))
            draw = ImageDraw.Draw(image)
            draw.text((15, 6), f"GO2 / {config['world'].upper()} / RECORDED PHYSICS", font=font, fill='#68ddd0')
            draw.text((15, 378), f"t = {physics['time'][index]:.1f} s / torque-driven Go2 / replay camera", font=font, fill='#e8f3fc')
            frames.append(image)
    gif(frames, output/'walking.gif')
    frames[len(frames)//2].save(output/'go2.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--require-return', action='store_true')
    args = parser.parse_args()
    output = args.output
    required = ['manifest.json', 'simulation.json', 'mapping.json', 'exploration.json', 'physics.npz',
                'mapping.npz', 'trace.json', 'field_samples.npz', 'measurements.npz', 'launch_exit.json', 'ros_interfaces.json']
    missing = [name for name in required if not (output/name).is_file()]
    if missing:
        summary = {'outcome': 'FAIL', 'missing_artifacts': missing, 'note': 'Inspect ros.log for node failures'}
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary, indent=2))
        return 1
    manifest = json.loads((output/'manifest.json').read_text())
    sim = json.loads((output/'simulation.json').read_text())
    mapper = json.loads((output/'mapping.json').read_text())
    explorer = json.loads((output/'exploration.json').read_text())
    interfaces = json.loads((output/'ros_interfaces.json').read_text())
    physics, mapping = np.load(output/'physics.npz'), np.load(output/'mapping.npz')
    trace = json.loads((output/'trace.json').read_text())
    checks = dict(launch_exited_cleanly=json.loads((output/'launch_exit.json').read_text())['exit_code'] == 0,
        physics_healthy=sim['error'] is None and sim['minimum_upright'] > .8 and sim['reason'] in ('duration', 'returned'),
        live_ros_interfaces=all(n > 0 for n in interfaces['messages'].values()) and not any(interfaces['invalid'].values()),
        physical_movement=sim['path_length_m'] >= .75,
        real_command_flow=sim['command_messages'] >= 10,
        observed_map=mapper['scan_updates'] >= 20 and mapper['observed_free_cells'] >= 100,
        graph_built=explorer['max_graph_nodes'] >= 3 and explorer['max_graph_edges'] >= 2,
        radiation_received=explorer['radiation_samples'] >= 5,
        no_obstacle_contacts=sim['obstacle_contact_steps'] == 0,
        within_actual_budget=sim['actual_dose_usv'] <= explorer['dose_budget_usv'])
    returned = explorer['state'] == 'returned' and explorer['final_home_distance_m'] < .4
    if args.require_return:
        checks['returned_home'] = returned
    truth_free = ~physics['true_occupied']
    final_observed = int(np.sum((mapping['grid'] == 0) & truth_free))
    coverage = final_observed/int(np.sum(truth_free))
    initial_observed = int(np.sum((mapping['snapshots'][0] == 0) & truth_free)) if len(mapping['snapshots']) else 0
    summary = dict(outcome='PASS' if all(checks.values()) else 'FAIL', checks=checks, returned_home=returned,
        observed_free_fraction=coverage, initial_observed_free_fraction=initial_observed/int(np.sum(truth_free)),
        observed_free_area_m2=final_observed*.01, simulation=sim, mapping=mapper, exploration=explorer, interfaces=interfaces,
        runtime_source_sha256=manifest['runtime_source_sha256'], image_id=manifest['image_id'])
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    if len(physics['time']) and len(mapping['time']):
        render(output, manifest['config'], physics, mapping, trace, summary)
    rows = ''.join(f'<tr><td>{html.escape(k)}</td><td>{v}</td></tr>' for k, v in checks.items())
    report = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Go2 occupancy exploration report</title><style>body{{font:16px system-ui;background:#0b1724;color:#e8f3fc;max-width:1200px;margin:32px auto;padding:20px}}a{{color:#68ddd0}}img{{max-width:100%}}table{{border-collapse:collapse}}td{{padding:8px 20px;border:1px solid #456}}.gallery{{display:flex;gap:15px;flex-wrap:wrap}}.gallery img{{width:48%;min-width:300px}}</style>
<h1>Go2 occupancy exploration: {summary['outcome']}</h1><p>{html.escape(manifest['config']['world'])} / seed {manifest['config']['seed']} / observed-free fraction {coverage:.1%} / returned: {returned}</p>
<p>Real MuJoCo joint physics and ROS 2 messages. Ideal pose and scans; synthetic radiation proxy.</p>
<img src="report.png" alt="Observed map, inferred field and exposure"><div class="gallery"><img src="walking.gif" alt="Recorded Go2 physics replay"><img src="mapping.gif" alt="Recorded observed-map replay"></div>
<h2>Run checks</h2><table>{rows}</table><p><a href="summary.json">Summary JSON</a> · <a href="manifest.json">Source manifest</a> · <a href="ros.log">ROS log</a> · <a href="trace.json">Planner trace</a></p>
<p>Coverage is observed known-map-domain free area, not full-facility autonomy. A passed run is one trial, not a dose-safety guarantee.</p></html>'''
    (output/'index.html').write_text(report)
    artifacts = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir()
                 if p.is_file() and p.name != 'artifact_hashes.json'}
    (output/'artifact_hashes.json').write_text(json.dumps(artifacts, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
