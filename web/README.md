# Web demo

An interactive, in-browser version of the planner. Draw obstacles, move the
start/goal, and hit **Plan & Drive** to watch Hybrid A* plan a path and the
pure-pursuit controller track it — with live metrics.

This is a faithful **JavaScript port** of the C++ implementation in the parent
directory (same bicycle model, dual heuristic, Dubins math, and controller).
The C++ version remains the reference; the JS reproduces the same results
(identical path lengths and ~9 cm tracking RMS on both built-in scenarios).

## Files

- `index.html` — the demo UI (loads `planner.js`).
- `planner.js` — the algorithm, browser + Node compatible.
- `demo.html` — a single-file build (planner inlined); open this if you just
  want one file with no dependencies.

## Run locally

Open `demo.html` directly in a browser, or serve the folder:

```bash
python3 -m http.server 8000   # then visit http://localhost:8000/web/
```

## Host on GitHub Pages

Push the repo, enable Pages (Settings → Pages → deploy from branch), and the
demo will be live at `https://<user>.github.io/<repo>/web/`. Link that URL when
you share the project — it lets anyone try it with zero setup.

## Headless check

`planner.js` also runs under Node, so the algorithm can be tested without a
browser:

```js
const AP = require('./planner.js');
const sc = AP.makeScenario('obstacles');
const map = new AP.GridMap(60, 40, 0.5);
sc.obstacles.forEach(o => map.addObstacle(o));
map.inflate(0.5 * sc.veh.width + 0.2);
console.log(new AP.HybridAStar(map, sc.veh).plan(sc.start, sc.goal).success);
```
