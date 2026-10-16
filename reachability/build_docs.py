#!/usr/bin/env python3
"""Build the GitHub Pages report (docs/index.html) from reachability/results/*.json.

    python3 reachability/build_docs.py          # then commit docs/

Plain Python (no cuRobo, no ROS). One section per robot found in results/; figures
from reachability/figures/ and the input sketches are copied to docs/assets/.
"""
import datetime
import html
import json
import math
import shutil
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DOCS = ROOT / 'docs'

ROBOT_NAMES = {'rby1m': 'Rainbow RB-Y1 (Model M, mecanum)', 'vb2': 'Lanxin VB2', 'g1': 'Galbot G1',
               'moz1': 'SpiritAI Moz1'}
COMBOS = [(1, 'fixed'), (1, 'mobile'), (2, 'fixed'), (2, 'mobile')]
TASK = {1: 'Task 1: pull + suction pick', 2: 'Task 2: full tray, two hands'}
BASE = {'fixed': 'fixed base', 'mobile': 'moving base'}


def esc(s):
    return html.escape(str(s))


def cell(level):
    """One ladder cell: what worked on this level, or how far it missed."""
    probs = level['problems']
    good = [p for p in probs if p['success']]
    if good:
        b = good[0]['base']
        rows = ', '.join(p['label'].replace('row ', '') for p in good) if 'row' in good[0] else ''
        detail = f"rows {rows}" if rows else ''
        if abs(b[0]) + abs(b[1]) + abs(b[2]) > 1e-6:
            detail += f"{' · ' if detail else ''}base {b[0]:+.2f}, {b[1]:+.2f} m, {math.degrees(b[2]):+.0f}°"
        return f'<td class="ok" title="{esc(detail)}"><span class="mark">✓</span><span class="d">{esc(detail)}</span></td>'
    best = min(probs, key=lambda p: p['position_error_mm'])
    if best.get('ik_success'):
        why = 'body hits tray'
    else:
        why = f"misses by {best['position_error_mm']:.0f} mm"
    return f'<td class="no" title="{esc(why)}"><span class="mark">✗</span><span class="d">{esc(why)}</span></td>'


def robot_section(robot, results, figures):
    have = {(r['task'], r['base']): r for r in results}
    levels = max(len(r['levels']) for r in results)
    any_r = results[0]
    head = ''.join(f'<th>{esc(TASK[t].split(":")[0])}<br><small>{esc(BASE[b])}</small></th>' for t, b in COMBOS)
    body = []
    for lvl in reversed(range(levels)):
        top = any_r['levels'][lvl]['board_top']
        cells = ''.join(cell(have[(t, b)]['levels'][lvl]) if (t, b) in have else '<td class="na">–</td>'
                        for t, b in COMBOS)
        body.append(f'<tr><th class="lvl">{lvl}<small>{top:.2f} m</small></th>{cells}</tr>')
    summary = []
    for t, b in COMBOS:
        r = have.get((t, b))
        if r:
            hi = r['highest_ok']
            top = r['levels'][hi]['board_top'] if hi is not None else None
            summary.append(f'<div class="stat"><div class="k">{esc(TASK[t].split(":")[0])}, {esc(BASE[b])}</div>'
                           f'<div class="v">{"–" if hi is None else f"level {hi}"}</div>'
                           f'<div class="s">{"none" if top is None else f"board top {top:.2f} m"}</div></div>')
    figs = ''.join(f'<figure><img src="assets/{esc(f.name)}" alt="{esc(f.stem)}" loading="lazy">'
                   f'<figcaption>{esc(caption(f.stem))}</figcaption></figure>' for f in figures)
    return f'''
<section id="{esc(robot)}">
  <h2>{esc(ROBOT_NAMES.get(robot, robot))}</h2>
  <div class="stats">{''.join(summary)}</div>
  <div class="scroll"><table class="ladder">
    <thead><tr><th>Level<br><small>board top</small></th>{head}</tr></thead>
    <tbody>{''.join(body)}</tbody>
  </table></div>
  <p class="note">✓ = a collision-free whole-body pose exists (hover for the vial rows / base pose);
  ✗ = closest attempt and why it failed.</p>
  {f'<div class="figs">{figs}</div>' if figs else ''}
</section>'''


def caption(stem):
    parts = stem.split('_')            # rby1m_task2_mobile_L05
    try:
        return f"{TASK[int(parts[1][4:])]}, {BASE[parts[2]]}, level {int(parts[3][1:])}: " \
               "robot collision spheres (blue; hands green), held tray (orange), shelves (grey)."
    except (IndexError, KeyError, ValueError):
        return stem


def main():
    results = [json.load(open(f)) for f in sorted((HERE / 'results').glob('*.json'))]
    results = [r for r in results if r.get('scene', 'full') == 'full']
    by_robot = {}
    for r in results:
        by_robot.setdefault(r['robot'], []).append(r)

    assets = DOCS / 'assets'
    assets.mkdir(parents=True, exist_ok=True)
    figures = {}
    for f in sorted((HERE / 'figures').glob('*.png')):
        shutil.copy2(f, assets / f.name)
        figures.setdefault(f.name.split('_')[0], []).append(f)
    for f in ('shelves.png', 'tray.png'):
        if (ROOT / f).exists():
            shutil.copy2(ROOT / f, assets / f)
    (DOCS / '.nojekyll').write_text('')

    try:
        commit = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', '--short', 'HEAD'], text=True).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        commit = 'unknown'
    today = datetime.date.today().isoformat()
    sections = ''.join(robot_section(rb, rs, figures.get(rb, [])) for rb, rs in by_robot.items())
    nav = ' · '.join(f'<a href="#{esc(rb)}">{esc(ROBOT_NAMES.get(rb, rb))}</a>' for rb in by_robot)
    pending = [n for k, n in ROBOT_NAMES.items() if k not in by_robot]

    page = f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Shelf Reachability</title>
<meta name="description" content="Highest shelf level each wheeled humanoid can work, per task, from collision-aware whole-body IK.">
<style>
:root {{
  --bg: #f7f7f5; --fg: #1d2127; --muted: #5d6573; --card: #ffffff; --line: #dfe2e7;
  --ok: #1f8a4c; --ok-bg: #e3f4ea; --no: #a33a2c; --no-bg: #f8e6e3; --accent: #2f5fa7; --code: #eef0f3;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    --bg: #15181d; --fg: #e6e8eb; --muted: #9aa3b1; --card: #1d2128; --line: #2e343d;
    --ok: #5cc98a; --ok-bg: #173325; --no: #ef8d7f; --no-bg: #3a1f1b; --accent: #8db4f0; --code: #252a33;
  }}
}}
:root[data-theme="dark"] {{
  --bg: #15181d; --fg: #e6e8eb; --muted: #9aa3b1; --card: #1d2128; --line: #2e343d;
  --ok: #5cc98a; --ok-bg: #173325; --no: #ef8d7f; --no-bg: #3a1f1b; --accent: #8db4f0; --code: #252a33;
}}
* {{ box-sizing: border-box; }}
body {{ margin: 0; background: var(--bg); color: var(--fg);
  font: 16px/1.55 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }}
main {{ max-width: 980px; margin: 0 auto; padding: 32px 16px 64px; }}
h1 {{ font-size: 1.9rem; margin: 0 0 4px; letter-spacing: -0.01em; }}
h2 {{ font-size: 1.35rem; margin: 40px 0 12px; }}
h3 {{ font-size: 1.05rem; margin: 24px 0 8px; }}
.sub {{ color: var(--muted); margin: 0 0 20px; }}
a {{ color: var(--accent); }}
.lead {{ font-size: 1.08rem; background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 14px 16px; }}
.stats {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin: 12px 0 16px; }}
.stat {{ background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 10px 12px; }}
.stat .k {{ color: var(--muted); font-size: .85rem; }}
.stat .v {{ font-size: 1.35rem; font-weight: 650; }}
.stat .s {{ color: var(--muted); font-size: .85rem; }}
.scroll {{ overflow-x: auto; }}
table.ladder {{ border-collapse: separate; border-spacing: 3px; width: 100%; min-width: 560px; }}
.ladder th, .ladder td {{ padding: 6px 8px; border-radius: 6px; text-align: left; vertical-align: middle; font-size: .9rem; }}
.ladder thead th {{ color: var(--muted); font-weight: 600; }}
.ladder th.lvl {{ white-space: nowrap; font-weight: 650; }}
.ladder th small, .ladder thead small {{ display: block; color: var(--muted); font-weight: 400; }}
.ladder td.ok {{ background: var(--ok-bg); color: var(--ok); }}
.ladder td.no {{ background: var(--no-bg); color: var(--no); }}
.ladder td.na {{ color: var(--muted); }}
.mark {{ font-weight: 700; margin-right: 6px; }}
.d {{ font-size: .8rem; }}
.note {{ color: var(--muted); font-size: .88rem; }}
.figs {{ display: grid; grid-template-columns: 1fr; gap: 16px; margin-top: 16px; }}
figure {{ margin: 0; background: #fff; border: 1px solid var(--line); border-radius: 10px; overflow: hidden; }}
figure img {{ width: 100%; height: auto; display: block; }}
figcaption {{ background: var(--card); color: var(--muted); font-size: .85rem; padding: 8px 12px; }}
.sketches {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 16px; }}
ul {{ padding-left: 20px; }}
li {{ margin: 4px 0; }}
code, pre {{ background: var(--code); border-radius: 6px; font-size: .88rem; }}
code {{ padding: 1px 5px; }}
pre {{ padding: 12px; overflow-x: auto; }}
footer {{ color: var(--muted); font-size: .85rem; margin-top: 48px; border-top: 1px solid var(--line); padding-top: 12px; }}
</style>
</head>
<body>
<main>
  <h1>Shelf reachability</h1>
  <p class="sub">Which tray level can each wheeled humanoid work in a 1.0 m aisle between two 13-level shelves?</p>
  <p class="lead">Computed with whole-body, two-handed, collision-aware inverse kinematics
  (<a href="https://github.com/NVlabs/curobo">cuRobo</a>) against the same shelf geometry as the Gazebo world.
  A level counts as workable when a pose exists that puts both hands on target with no part of the robot
  touching the shelves, the other trays or the tray it holds.</p>
  <p class="note">Robots: {nav}{f" · pending: {esc(', '.join(pending))}" if pending else ''}</p>

  <h2>Setup</h2>
  <ul>
    <li><b>Shelves</b>: two, 1.22 m long, 0.62 m deep, facing each other across a 1.0 m aisle.
      Bottom board top at 0.36 m, then 12 levels every 0.14 m (top board 2.04 m). 19 trays per level,
      34 vials (2 × 17) per tray.</li>
    <li><b>Task 1, pull + suction pick</b>: the left hand holds the tray handle with the tray pulled 25 cm out
      (its back end stays on the board as the datum); the right hand, an emulated suction tool pointing down
      (13 cm long), touches a vial in a row clear of the board above (rows 1–4; the 10 cm gap
      between boards leaves ~17 mm above the vials inside the shelf).</li>
    <li><b>Task 2, full tray with two hands</b>: the tray is fully out (back end 10 cm in front of the shelf);
      the left hand holds the handle (from the front or from above), the right hand holds the back end
      from above, fingers either side of the tray.</li>
    <li><b>Fixed base</b>: robot at the aisle centre facing the shelf (as spawned in Gazebo).
      <b>Moving base</b>: the solver may also place and turn the base anywhere in the aisle.</li>
  </ul>
  <div class="sketches">
    <figure><img src="assets/shelves.png" alt="Shelf plan view sketch" loading="lazy"><figcaption>Input sketch: shelves and aisle.</figcaption></figure>
    <figure><img src="assets/tray.png" alt="Tray sketch" loading="lazy"><figcaption>Input sketch: tray and vials.</figcaption></figure>
  </div>

  {sections}

  <h2>Method</h2>
  <ul>
    <li>Robot geometry from the robots' URDFs (<a href="https://github.com/artc-asr/rochestra">rochestra</a>),
      approximated by collision spheres fitted automatically to the meshes; self-collision checked.</li>
    <li>Collision scene: every collision box of the Gazebo world, minus the target tray, plus a keep-out box
      under each shelf (the robot may not drive under the bottom board).</li>
    <li>Each solution is re-checked against the whole tray (only the hands may touch it), and its hand poses
      re-computed independently from the original URDF (worst error 2.7 mm, tolerance 5 mm).</li>
    <li>Task 1 tries 256 random start poses per problem, task 2 1024 (its solutions are narrow).</li>
  </ul>

  <h2>Not covered yet</h2>
  <ul>
    <li>Joint torques and payload (tray assumed 2.44 kg), balance, and the motion between poses: only the key
      holding pose is checked.</li>
    <li>Real shelf details: whether the space under the bottom board is open (assumed closed), and the tray
      length (sketch says 620 mm; the model uses 580 mm = 58 handle + 17 × 30 + 12).</li>
  </ul>

  <h2>Reproduce</h2>
<pre>git clone https://github.com/artc-asr/givaudan_automation_sim.git && cd givaudan_automation_sim
./scripts/setup_workspace.sh                       # ROS 2 Humble workspace (robot descriptions)
# cuRobo: see reachability/README.md
env -u PYTHONPATH deps/curobo-venv/bin/python reachability/sweep.py --robot rby1m --task 2 --base mobile
python3 reachability/build_docs.py                 # rebuilds this page</pre>

  <footer>Generated {today} from <code>reachability/results/</code> at commit <code>{esc(commit)}</code> ·
  <a href="https://github.com/artc-asr/givaudan_automation_sim">source</a></footer>
</main>
</body>
</html>
'''
    (DOCS / 'index.html').write_text(page)
    print(f'wrote {DOCS / "index.html"}: {len(by_robot)} robot(s), {len(results)} result files, '
          f'{sum(len(v) for v in figures.values())} figures')


if __name__ == '__main__':
    main()
