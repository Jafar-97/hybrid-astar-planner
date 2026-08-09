/* Hybrid A* motion planner + pure-pursuit tracking — JavaScript port.
 *
 * This is a faithful port of the C++ implementation in this repo (same
 * bicycle model, same dual heuristic, same Dubins analytic expansion, same
 * pure-pursuit controller). It runs entirely in the browser so the demo needs
 * no build step. The C++ version remains the reference implementation.
 *
 * Works both in the browser (attaches to window.AP) and under Node (exports)
 * so the algorithm can be unit-tested headlessly.
 */
(function (root) {
  "use strict";
  const PI = Math.PI;

  function normalizeAngle(a) {
    while (a > PI) a -= 2 * PI;
    while (a <= -PI) a += 2 * PI;
    return a;
  }
  function mod2pi(a) {
    let r = a % (2 * PI);
    if (r < 0) r += 2 * PI;
    return r;
  }
  const hypot2 = (dx, dy) => Math.sqrt(dx * dx + dy * dy);
  const dist = (a, b) => hypot2(a.x - b.x, a.y - b.y);

  // ---- bicycle model ----
  function bicycleStep(p, steer, ds, L) {
    return {
      x: p.x + ds * Math.cos(p.theta),
      y: p.y + ds * Math.sin(p.theta),
      theta: normalizeAngle(p.theta + (ds * Math.tan(steer)) / L),
    };
  }

  // ---- vehicle ----
  function defaultVehicle() {
    const v = {
      wheelbase: 2.7, max_steer: 0.55, width: 1.9, length: 4.5,
      max_speed: 8.0, max_accel: 2.5,
    };
    v.minTurnRadius = () => v.wheelbase / Math.tan(v.max_steer);
    return v;
  }

  // ---- grid map / costmap ----
  class GridMap {
    constructor(widthM, heightM, res) {
      this.width = widthM; this.height = heightM; this.res = res;
      this.cols = Math.ceil(widthM / res);
      this.rows = Math.ceil(heightM / res);
      this.occ = new Uint8Array(this.cols * this.rows);
      this.obstacles = [];
    }
    idx(cx, cy) { return cy * this.cols + cx; }
    worldToCell(x, y) {
      const cx = Math.floor(x / this.res), cy = Math.floor(y / this.res);
      return { cx, cy, ok: cx >= 0 && cx < this.cols && cy >= 0 && cy < this.rows };
    }
    inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }
    occupied(cx, cy) {
      if (cx < 0 || cx >= this.cols || cy < 0 || cy >= this.rows) return true;
      return this.occ[this.idx(cx, cy)] !== 0;
    }
    isFree(x, y) {
      const c = this.worldToCell(x, y);
      if (!c.ok) return false;
      return this.occ[this.idx(c.cx, c.cy)] === 0;
    }
    addObstacle(o) {
      this.obstacles.push(o);
      const a = this.worldToCell(Math.min(o.x0, o.x1), Math.min(o.y0, o.y1));
      const b = this.worldToCell(Math.max(o.x0, o.x1), Math.max(o.y0, o.y1));
      const cx0 = Math.max(0, a.cx), cy0 = Math.max(0, a.cy);
      const cx1 = Math.min(this.cols - 1, b.cx), cy1 = Math.min(this.rows - 1, b.cy);
      for (let cy = cy0; cy <= cy1; cy++)
        for (let cx = cx0; cx <= cx1; cx++) this.occ[this.idx(cx, cy)] = 1;
    }
    inflate(radius) {
      const r = Math.ceil(radius / this.res);
      if (r <= 0) return;
      const src = this.occ.slice();
      for (let cy = 0; cy < this.rows; cy++)
        for (let cx = 0; cx < this.cols; cx++) {
          if (src[this.idx(cx, cy)] === 0) continue;
          for (let dy = -r; dy <= r; dy++)
            for (let dx = -r; dx <= r; dx++) {
              if (dx * dx + dy * dy > r * r) continue;
              const nx = cx + dx, ny = cy + dy;
              if (nx < 0 || nx >= this.cols || ny < 0 || ny >= this.rows) continue;
              this.occ[this.idx(nx, ny)] = 1;
            }
        }
    }
  }

  // ---- Dubins curves ----
  function dubinsWords(a, b, d) {
    const sa = Math.sin(a), sb = Math.sin(b), ca = Math.cos(a), cb = Math.cos(b);
    const cab = Math.cos(a - b);
    const out = [];
    // LSL
    {
      const p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
      if (p2 >= 0) {
        const tmp = Math.atan2(cb - ca, d + sa - sb);
        out.push({ t: mod2pi(-a + tmp), p: Math.sqrt(p2), q: mod2pi(b - tmp), types: ["L", "S", "L"] });
      }
    }
    // RSR
    {
      const p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
      if (p2 >= 0) {
        const tmp = Math.atan2(ca - cb, d - sa + sb);
        out.push({ t: mod2pi(a - tmp), p: Math.sqrt(p2), q: mod2pi(-b + tmp), types: ["R", "S", "R"] });
      }
    }
    // LSR
    {
      const p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
      if (p2 >= 0) {
        const p = Math.sqrt(p2);
        const tmp = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2.0, p);
        out.push({ t: mod2pi(-a + tmp), p, q: mod2pi(-b + tmp), types: ["L", "S", "R"] });
      }
    }
    // RSL
    {
      const p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb);
      if (p2 >= 0) {
        const p = Math.sqrt(p2);
        const tmp = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2.0, p);
        out.push({ t: mod2pi(a - tmp), p, q: mod2pi(b - tmp), types: ["R", "S", "L"] });
      }
    }
    // RLR
    {
      const tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
      if (Math.abs(tmp) <= 1) {
        const p = mod2pi(2 * PI - Math.acos(tmp));
        const t = mod2pi(a - Math.atan2(ca - cb, d - sa + sb) + p / 2);
        out.push({ t, p, q: mod2pi(a - b - t + p), types: ["R", "L", "R"] });
      }
    }
    // LRL
    {
      const tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8;
      if (Math.abs(tmp) <= 1) {
        const p = mod2pi(2 * PI - Math.acos(tmp));
        const t = mod2pi(-a - Math.atan2(ca - cb, d + sa - sb) + p / 2);
        out.push({ t, p, q: mod2pi(mod2pi(b) - a - t + p), types: ["L", "R", "L"] });
      }
    }
    return out;
  }

  function dubinsShortest(start, goal, r) {
    if (r <= 0) return { valid: false };
    const dx = goal.x - start.x, dy = goal.y - start.y;
    const D = hypot2(dx, dy);
    const d = D / r;
    const theta = Math.atan2(dy, dx);
    const a = mod2pi(start.theta - theta);
    const b = mod2pi(goal.theta - theta);
    const words = dubinsWords(a, b, d);
    let best = null, bestCost = Infinity;
    for (const w of words) {
      const c = w.t + w.p + w.q;
      if (c < bestCost) { bestCost = c; best = w; }
    }
    if (!best) return { valid: false };
    return {
      valid: true, types: best.types, radius: r, start,
      len: [best.t * r, best.p * r, best.q * r],
      length: () => (best.t + best.p + best.q) * r,
    };
  }
  function dubinsLength(start, goal, r) {
    const p = dubinsShortest(start, goal, r);
    return p.valid ? p.length() : Infinity;
  }
  function sampleDubins(path, step) {
    const out = [];
    if (!path.valid) return out;
    let p = { ...path.start };
    out.push({ ...p });
    for (let seg = 0; seg < 3; seg++) {
      const segLen = path.len[seg];
      const type = path.types[seg];
      const kappa = type === "L" ? 1 / path.radius : type === "R" ? -1 / path.radius : 0;
      let traveled = 0;
      while (traveled < segLen - 1e-9) {
        const ds = Math.min(step, segLen - traveled);
        if (Math.abs(kappa) < 1e-9) {
          p.x += ds * Math.cos(p.theta);
          p.y += ds * Math.sin(p.theta);
        } else {
          const nt = p.theta + kappa * ds;
          p.x += (Math.sin(nt) - Math.sin(p.theta)) / kappa;
          p.y += (-Math.cos(nt) + Math.cos(p.theta)) / kappa;
          p.theta = normalizeAngle(nt);
        }
        traveled += ds;
        out.push({ ...p });
      }
    }
    return out;
  }

  // ---- binary min-heap keyed on numeric priority ----
  class MinHeap {
    constructor() { this.a = []; }
    get size() { return this.a.length; }
    push(prio, val) {
      const a = this.a; a.push({ prio, val });
      let i = a.length - 1;
      while (i > 0) {
        const par = (i - 1) >> 1;
        if (a[par].prio <= a[i].prio) break;
        [a[par], a[i]] = [a[i], a[par]]; i = par;
      }
    }
    pop() {
      const a = this.a; const top = a[0]; const last = a.pop();
      if (a.length) {
        a[0] = last; let i = 0;
        for (;;) {
          const l = 2 * i + 1, r = 2 * i + 2; let s = i;
          if (l < a.length && a[l].prio < a[s].prio) s = l;
          if (r < a.length && a[r].prio < a[s].prio) s = r;
          if (s === i) break;
          [a[s], a[i]] = [a[i], a[s]]; i = s;
        }
      }
      return top;
    }
  }

  // ---- Hybrid A* ----
  const defaultPlannerConfig = () => ({
    xy_resolution: 0.5, theta_bins: 72, step_size: 1.4, n_steer: 5,
    steer_penalty: 1.5, steer_change_penalty: 1.5, analytic_period: 5,
    max_iterations: 300000, goal_xy_tol: 0.6, goal_yaw_tol: 0.2,
  });

  class HybridAStar {
    constructor(map, veh, cfg) {
      this.map = map; this.veh = veh;
      this.cfg = Object.assign(defaultPlannerConfig(), cfg || {});
    }
    poseCollisionFree(p) {
      if (!this.map.inBounds(p.x, p.y)) return false;
      const halfL = 0.5 * this.veh.length;
      for (let s = -halfL; s <= halfL + 1e-6; s += 0.7) {
        if (!this.map.isFree(p.x + s * Math.cos(p.theta), p.y + s * Math.sin(p.theta)))
          return false;
      }
      return true;
    }
    pathCollisionFree(poses) {
      for (const p of poses) if (!this.poseCollisionFree(p)) return false;
      return true;
    }
    buildHolonomic(goal) {
      const m = this.map, N = m.cols * m.rows;
      const field = new Float64Array(N).fill(Infinity);
      const g = m.worldToCell(goal.x, goal.y);
      if (!g.ok) return field;
      const heap = new MinHeap();
      const gi = m.idx(g.cx, g.cy);
      field[gi] = 0; heap.push(0, gi);
      const dx = [1, -1, 0, 0, 1, 1, -1, -1], dy = [0, 0, 1, -1, 1, -1, 1, -1];
      while (heap.size) {
        const { prio: c, val: i } = heap.pop();
        if (c > field[i]) continue;
        const cx = i % m.cols, cy = (i / m.cols) | 0;
        for (let k = 0; k < 8; k++) {
          const nx = cx + dx[k], ny = cy + dy[k];
          if (nx < 0 || nx >= m.cols || ny < 0 || ny >= m.rows) continue;
          if (m.occupied(nx, ny)) continue;
          const step = (k < 4 ? 1 : 1.41421356) * m.res;
          const ni = m.idx(nx, ny), nc = c + step;
          if (nc < field[ni]) { field[ni] = nc; heap.push(nc, ni); }
        }
      }
      return field;
    }
    holonomicCost(field, x, y) {
      const c = this.map.worldToCell(x, y);
      if (!c.ok) return Infinity;
      return field[this.map.idx(c.cx, c.cy)];
    }
    plan(start, goal) {
      const t0 = (typeof performance !== "undefined" ? performance.now() : Date.now());
      const cfg = this.cfg, veh = this.veh, m = this.map;
      const R = veh.minTurnRadius();
      const res = cfg.xy_resolution, dbin = (2 * PI) / cfg.theta_bins;
      const keyOf = (p) => {
        const xi = Math.floor(p.x / res), yi = Math.floor(p.y / res);
        const ti = Math.floor(mod2pi(p.theta) / dbin) % cfg.theta_bins;
        return xi + "," + yi + "," + ti;
      };
      const hfield = this.buildHolonomic(goal);
      const heuristic = (p) => {
        const hnh = dubinsLength(p, goal, R);
        const hh = this.holonomicCost(hfield, p.x, p.y);
        const h = Math.max(hnh, hh);
        return isFinite(h) ? h : hnh;
      };
      const steers = [];
      if (cfg.n_steer <= 1) steers.push(0);
      else for (let i = 0; i < cfg.n_steer; i++)
        steers.push(-veh.max_steer + (2 * veh.max_steer * i) / (cfg.n_steer - 1));

      const nodes = [];
      const bestG = new Map();
      const s0 = { pose: start, g: 0, f: heuristic(start), steer: 0, parent: -1 };
      nodes.push(s0); bestG.set(keyOf(start), 0);
      const open = new MinHeap(); open.push(s0.f, 0);

      const substeps = 10, ds = cfg.step_size / substeps;
      let goalNode = -1, analyticTail = [], iter = 0, expanded = 0;

      while (open.size && iter < cfg.max_iterations) {
        const { prio: fcur, val: idx } = open.pop();
        iter++;
        const cur = nodes[idx];
        if (fcur > cur.f + 1e-9) continue;

        if (dist(cur.pose, goal) < cfg.goal_xy_tol &&
            Math.abs(normalizeAngle(cur.pose.theta - goal.theta)) < cfg.goal_yaw_tol) {
          goalNode = idx; break;
        }
        const near = dist(cur.pose, goal) < 15.0;
        if (near && iter % cfg.analytic_period === 0) {
          const dp = dubinsShortest(cur.pose, goal, R);
          if (dp.valid) {
            const samp = sampleDubins(dp, 0.3);
            if (this.pathCollisionFree(samp)) { analyticTail = samp; goalNode = idx; break; }
          }
        }
        expanded++;
        for (const steer of steers) {
          let p = cur.pose, ok = true;
          for (let k = 0; k < substeps; k++) {
            p = bicycleStep(p, steer, ds, veh.wheelbase);
            if (!this.poseCollisionFree(p)) { ok = false; break; }
          }
          if (!ok) continue;
          const moveCost = cfg.step_size * (1 + cfg.steer_penalty * Math.abs(steer) +
            cfg.steer_change_penalty * Math.abs(steer - cur.steer));
          const gNew = cur.g + moveCost;
          const key = keyOf(p);
          const prev = bestG.get(key);
          if (prev !== undefined && prev <= gNew + 1e-6) continue;
          bestG.set(key, gNew);
          const nn = { pose: p, g: gNew, f: gNew + heuristic(p), steer, parent: idx };
          const nidx = nodes.length; nodes.push(nn); open.push(nn.f, nidx);
        }
      }

      const t1 = (typeof performance !== "undefined" ? performance.now() : Date.now());
      const result = { success: false, path: [], pathLength: 0, nodesExpanded: expanded,
                       iterations: iter, planTimeMs: t1 - t0 };
      if (goalNode < 0) return result;

      const chain = [];
      for (let i = goalNode; i >= 0; i = nodes[i].parent) chain.push(i);
      chain.reverse();
      const dense = [{ ...nodes[chain[0]].pose }];
      for (let c = 1; c < chain.length; c++) {
        let p = nodes[chain[c - 1]].pose;
        const steer = nodes[chain[c]].steer;
        for (let k = 0; k < substeps; k++) { p = bicycleStep(p, steer, ds, veh.wheelbase); dense.push({ ...p }); }
      }
      for (let i = 1; i < analyticTail.length; i++) dense.push(analyticTail[i]);
      let len = 0;
      for (let i = 1; i < dense.length; i++) len += dist(dense[i - 1], dense[i]);
      result.success = true; result.path = dense; result.pathLength = len;
      result.planTimeMs = t1 - t0;
      return result;
    }
  }

  // ---- pure-pursuit + PID controller ----
  const defaultControllerConfig = () => ({
    lookahead_min: 2.0, lookahead_k: 0.3, target_speed: 4.0, curve_slowdown: 3.5,
    kp: 1.2, ki: 0.05, kd: 0.05, goal_tol: 1.0,
  });

  class PurePursuit {
    constructor(path, veh, cfg) {
      this.path = path; this.veh = veh;
      this.cfg = Object.assign(defaultControllerConfig(), cfg || {});
      this.lastIdx = 0; this.integral = 0; this.prevErr = 0;
    }
    nearestIndex(pose) {
      let best = this.lastIdx, bestD = 1e18;
      for (let i = this.lastIdx; i < this.path.length; i++) {
        const d = dist(pose, this.path[i]);
        if (d < bestD) { bestD = d; best = i; }
        if (d > bestD + 5.0) break;
      }
      return best;
    }
    control(pose, speed, dt) {
      const cmd = { steer: 0, accel: 0, target_x: 0, target_y: 0, cross_track: 0, finished: false };
      if (!this.path.length) { cmd.finished = true; return cmd; }
      const ni = this.nearestIndex(pose); this.lastIdx = ni;
      const r = this.path[ni];
      cmd.cross_track = -Math.sin(r.theta) * (pose.x - r.x) + Math.cos(r.theta) * (pose.y - r.y);
      if (dist(pose, this.path[this.path.length - 1]) < this.cfg.goal_tol) {
        cmd.finished = true; cmd.accel = -this.veh.max_accel; return cmd;
      }
      const Ld = this.cfg.lookahead_min + this.cfg.lookahead_k * Math.max(0, speed);
      let ti = ni;
      for (let i = ni; i < this.path.length; i++) { ti = i; if (dist(pose, this.path[i]) >= Ld) break; }
      const target = this.path[ti];
      cmd.target_x = target.x; cmd.target_y = target.y;
      const alpha = normalizeAngle(Math.atan2(target.y - pose.y, target.x - pose.x) - pose.theta);
      const ld = Math.max(dist(pose, target), 1e-3);
      let steer = Math.atan2(2 * this.veh.wheelbase * Math.sin(alpha), ld);
      cmd.steer = Math.max(-this.veh.max_steer, Math.min(this.veh.max_steer, steer));
      const curveFactor = 1 / (1 + this.cfg.curve_slowdown * Math.abs(cmd.steer));
      const vRef = this.cfg.target_speed * curveFactor;
      const err = vRef - speed;
      this.integral += err * dt;
      const deriv = (err - this.prevErr) / Math.max(dt, 1e-6);
      this.prevErr = err;
      let accel = this.cfg.kp * err + this.cfg.ki * this.integral + this.cfg.kd * deriv;
      cmd.accel = Math.max(-this.veh.max_accel, Math.min(this.veh.max_accel, accel));
      return cmd;
    }
  }

  function simulate(path, start, veh, ccfg, dt, maxTime) {
    dt = dt || 0.05; maxTime = maxTime || 90;
    const res = { log: [], metrics: { completed: false, sim_time: 0, cte_rms: 0, cte_max: 0, avg_speed: 0, distance: 0 } };
    if (!path.length) return res;
    const ctrl = new PurePursuit(path, veh, ccfg);
    let pose = { ...start }, v = 0, t = 0;
    let sumSq = 0, maxCte = 0, sumSpeed = 0, distTraveled = 0, n = 0, prev = { ...pose };
    while (t < maxTime) {
      const cmd = ctrl.control(pose, v, dt);
      const cte = Math.abs(cmd.cross_track);
      sumSq += cte * cte; maxCte = Math.max(maxCte, cte); sumSpeed += v; n++;
      res.log.push({ t, x: pose.x, y: pose.y, theta: pose.theta, v, steer: cmd.steer,
        target_x: cmd.target_x, target_y: cmd.target_y, cte: cmd.cross_track });
      if (cmd.finished && v < 0.3) { res.metrics.completed = true; break; }
      if (cte > 6.0) break;
      v += cmd.accel * dt; v = Math.max(0, Math.min(veh.max_speed, v));
      pose = bicycleStep(pose, cmd.steer, v * dt, veh.wheelbase);
      distTraveled += dist(prev, pose); prev = { ...pose }; t += dt;
    }
    res.metrics.sim_time = t;
    res.metrics.cte_rms = n ? Math.sqrt(sumSq / n) : 0;
    res.metrics.cte_max = maxCte;
    res.metrics.avg_speed = n ? sumSpeed / n : 0;
    res.metrics.distance = distTraveled;
    return res;
  }

  // ---- built-in scenarios ----
  function makeScenario(name) {
    const veh = defaultVehicle();
    if (name === "scurve") {
      const obs = [
        { x0: 0, y0: 0, x1: 60, y1: 1 }, { x0: 0, y0: 39, x1: 60, y1: 40 },
        { x0: 22, y0: 0, x1: 26, y1: 26 }, { x0: 36, y0: 14, x1: 40, y1: 40 },
      ];
      return { name, obstacles: obs, start: { x: 5, y: 20, theta: 0 }, goal: { x: 55, y: 20, theta: 0 }, veh };
    }
    const obs = [
      { x0: 0, y0: 0, x1: 60, y1: 1 }, { x0: 0, y0: 39, x1: 60, y1: 40 },
      { x0: 15, y0: 8, x1: 20, y1: 24 }, { x0: 28, y0: 18, x1: 33, y1: 40 },
      { x0: 30, y0: 0, x1: 34, y1: 10 }, { x0: 42, y0: 10, x1: 47, y1: 30 },
    ];
    return { name: "obstacles", obstacles: obs, start: { x: 5, y: 20, theta: 0 }, goal: { x: 56, y: 12, theta: 0 }, veh };
  }

  const AP = {
    normalizeAngle, mod2pi, dist, bicycleStep, defaultVehicle, GridMap,
    dubinsShortest, dubinsLength, sampleDubins, HybridAStar, PurePursuit,
    simulate, makeScenario, defaultPlannerConfig, defaultControllerConfig,
  };
  root.AP = AP;
  if (typeof module !== "undefined" && module.exports) module.exports = AP;
})(typeof window !== "undefined" ? window : globalThis);
