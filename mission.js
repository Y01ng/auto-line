/* mission.js - 2026 AIxHI track map, route planner, mission compiler, crossing detector.
 * Pure functions (no browser APIs) so Node can test them (test_mission.js).
 * Units: map in cm, steps in mm. Headings: 0 = east (right on the map), 1 = south, 2 = west, 3 = north.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.Mission = api;
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  const E = 0, S = 1, W = 2, N = 3;
  const TURN_DEG = [0, 90, 180, -90];   // by turnDelta: straight, right, U-turn, left
  const TURN_COST = [0, 12, 22, 12];    // cm-equivalent penalty for turning

  // 2026 AIxHI map (안내자료 16쪽 그림). Origin = top-left grid crossing, x → right, y → down (cm).
  const TRACK = {
    rows: [[0, [-23, 0, 18, 36, 54, 69]], [18, [0, 18, 36]], [37, [-17, 0]], [37, [18, 36, 54, 69]],
           [56, [-17, 0, 18]], [56, [36, 54]], [74, [-17, 0, 18, 36, 54, 69]]],
    cols: [[0, [0, 18, 37, 56, 74]], [18, [0, 18, 37, 56, 74]], [36, [0, 18, 37, 56, 74]], [54, [0, 37, 56, 74]]],
    bars: [[69, 0], [69, 37], [69, 74], [-17, 37], [-17, 56], [-17, 74]],   // T-bars at line ends
    box: [-23, 0],                                     // 밀집지역 (figures) centre
    start: { node: '69,0', heading: W },               // AI robot start: right end of the top line, facing west
    stubs: ['-17,37', '-17,56', '-17,74'],             // 전달존: left T-ends
    objects: [
      { id: 1, name: '빨간 공', node: '18,0' },
      { id: 2, name: '블록 더미', node: '36,37' },
      { id: 3, name: '주황 블록', node: '54,74' },
      { id: 4, name: '피규어 A', node: '-23,0' },
      { id: 5, name: '피규어 B', node: '-23,0' },
    ],
  };

  /** Practice track shaped like a "2": top line → right side down → middle line → left side down → bottom line.
   *  w = width, h1 = upper height, h2 = lower height (cm). Corners are the nodes. */
  function practiceTrack(w, h1, h2) {
    w = Math.round(w); h1 = Math.round(h1); h2 = Math.round(h2);
    return {
      rows: [[0, [0, w]], [h1, [0, w]], [h1 + h2, [0, w]]],
      cols: [[w, [0, h1]], [0, [h1, h1 + h2]]],
      bars: [], box: null,
      start: { node: '0,0', heading: E },
      stubs: [`${w},${h1 + h2}`],
      objects: [],
    };
  }

  const turnDelta = (h1, h2) => (((h2 - h1) % 4) + 4) % 4;

  function buildGraph(T = TRACK) {
    const nodes = new Map();
    const add = (x, y) => {
      const k = `${x},${y}`;
      if (!nodes.has(k)) nodes.set(k, { id: k, x, y, bar: false, adj: [] });
      return nodes.get(k);
    };
    const link = (a, b) => {
      const dx = b.x - a.x, dy = b.y - a.y;
      const dir = dx > 0 ? E : dx < 0 ? W : dy > 0 ? S : N;
      const len = Math.abs(dx) + Math.abs(dy);
      a.adj.push({ to: b.id, dir, len });
      b.adj.push({ to: a.id, dir: (dir + 2) % 4, len });
    };
    for (const [y, xs] of T.rows) for (let i = 0; i + 1 < xs.length; i++) link(add(xs[i], y), add(xs[i + 1], y));
    for (const [x, ys] of T.cols) for (let i = 0; i + 1 < ys.length; i++) link(add(x, ys[i]), add(x, ys[i + 1]));
    for (const [x, y] of T.bars) add(x, y).bar = true;
    return nodes;
  }

  /** Does the camera see a line across the path at this node when travelling in `dir`? */
  function hasCross(node, dir) {
    return node.bar || node.adj.some((a) => a.dir % 2 !== dir % 2);
  }

  function dijkstra(G, start, startHeading, blocked) {
    const key = (n, h) => n + '|' + h;
    const dist = new Map([[key(start, startHeading), 0]]);
    const prev = new Map();
    const pq = [[0, start, startHeading]];
    while (pq.length) {
      pq.sort((a, b) => a[0] - b[0]);
      const [d, n, h] = pq.shift();
      if (d > dist.get(key(n, h))) continue;
      for (const a of G.get(n).adj) {
        if (blocked.has(a.to)) continue;
        const nd = d + a.len + TURN_COST[turnDelta(h, a.dir)];
        const k = key(a.to, a.dir);
        if (nd < (dist.has(k) ? dist.get(k) : Infinity)) { dist.set(k, nd); prev.set(k, key(n, h)); pq.push([nd, a.to, a.dir]); }
      }
    }
    return { dist, prev, key };
  }

  /** Cheapest way to stand on a neighbour of `target`, facing it, at least `minApproachCm` away. */
  function planTo(G, pose, target, blockedIn, minApproachCm) {
    const blocked = new Set(blockedIn);
    blocked.add(target);
    blocked.delete(pose.node);
    const { dist, prev, key } = dijkstra(G, pose.node, pose.heading, blocked);
    let best = null;
    for (const a of G.get(target).adj) {
      const from = a.to, dirToT = (a.dir + 2) % 4;
      if (blocked.has(from) || a.len < minApproachCm) continue;
      for (let h = 0; h < 4; h++) {
        const k = key(from, h);
        if (!dist.has(k)) continue;
        const c = dist.get(k) + TURN_COST[turnDelta(h, dirToT)] + a.len;
        if (!best || c < best.cost) best = { cost: c, from, heading: h, dirToT, len: a.len };
      }
    }
    if (!best) return null;
    const states = [];
    for (let k = key(best.from, best.heading); k; k = prev.get(k)) {
      const [n, h] = k.split('|');
      states.unshift({ node: n, heading: +h });
    }
    return { cost: best.cost, states, from: best.from, approachDir: best.dirToT, approachLen: best.len };
  }

  /** states -> [turn | leg] steps. A leg is one straight run; marks = nodes passed (distance in mm). */
  function statesToSteps(G, states, heading) {
    const steps = [];
    let h = heading, leg = null;
    const flush = () => { if (leg) { steps.push(leg); leg = null; } };
    for (let i = 1; i < states.length; i++) {
      const from = states[i - 1].node, to = states[i].node, dir = states[i].heading;
      const e = G.get(from).adj.find((a) => a.to === to);
      if (dir !== h) {
        flush();
        steps.push({ op: 'turn', deg: TURN_DEG[turnDelta(h, dir)], at: from });
        h = dir;
      }
      if (!leg) leg = { op: 'leg', dir, from, to, len: 0, marks: [] };
      leg.len += e.len * 10;
      leg.to = to;
      leg.marks.push({ node: to, s: leg.len, cross: hasCross(G.get(to), dir) });
    }
    flush();
    return { steps, heading: h };
  }

  /**
   * targets: object ids in the announced order. o: {grab (mm, wheel axle → grip point),
   * dropOffset (mm before the T-bar), clear (mm extra back-off after release), stubs: [node ids]}.
   * Returns {steps, error}. Each pick: drive to the crossing before the object, face it, drive
   * (edge - grab) blind, grip, back to the crossing. Each drop: same toward a 전달존 T-end, release, back off.
   */
  function compileMission(T, targets, o) {
    const G = buildGraph(T);
    const objs = T.objects.map((x) => Object.assign({}, x, { done: false }));
    const stubs = o.stubs && o.stubs.length ? o.stubs : T.stubs;
    let pose = Object.assign({}, T.start);
    const steps = [];
    try {
      targets.forEach((tid, i) => {
        const obj = objs.find((x) => x.id === tid);
        if (!obj) throw new Error('없는 물체 번호: ' + tid);
        const blocked = new Set(objs.filter((x) => !x.done && x.node !== obj.node).map((x) => x.node));
        const pick = planTo(G, pose, obj.node, blocked, (o.grab + 20) / 10);
        if (!pick) throw new Error(`${obj.name}까지 갈 길이 없어요`);
        const a = statesToSteps(G, pick.states, pose.heading);
        steps.push(...a.steps);
        if (a.heading !== pick.approachDir) steps.push({ op: 'turn', deg: TURN_DEG[turnDelta(a.heading, pick.approachDir)], at: pick.from });
        const appr = Math.round(pick.approachLen * 10 - o.grab);
        steps.push({ op: 'move', mm: appr, why: `${obj.name} 앞까지` });
        steps.push({ op: 'grip', why: `${obj.name} 집기` });
        steps.push({ op: 'move', mm: -appr, why: '교차로로 후진' });
        obj.done = true;
        pose = { node: pick.from, heading: pick.approachDir };

        const stub = stubs[i % stubs.length];
        const blocked2 = new Set(objs.filter((x) => !x.done).map((x) => x.node));
        const drop = planTo(G, pose, stub, blocked2, 0);
        if (!drop) throw new Error(`전달존(${stub})까지 갈 길이 없어요`);
        const b = statesToSteps(G, drop.states, pose.heading);
        steps.push(...b.steps);
        if (b.heading !== drop.approachDir) steps.push({ op: 'turn', deg: TURN_DEG[turnDelta(b.heading, drop.approachDir)], at: drop.from });
        const dropAppr = Math.max(0, Math.round(drop.approachLen * 10 - o.dropOffset - o.grab));
        steps.push({ op: 'move', mm: dropAppr, why: '전달존으로' });
        steps.push({ op: 'release', why: `${obj.name} 내려놓기` });
        steps.push({ op: 'move', mm: -dropAppr, why: '교차로로 후진' });
        pose = { node: drop.from, heading: drop.approachDir };
      });
      if (targets.length && o.clear > 0) steps.push({ op: 'move', mm: -o.clear, why: '마지막 물러나기' });
    } catch (e) {
      return { steps, error: e.message };
    }
    return { steps, error: null };
  }

  function describe(step) {
    const dirs = ['→', '↓', '←', '↑'];
    switch (step.op) {
      case 'leg': return `선 따라 ${dirs[step.dir]} ${(step.len / 10).toFixed(0)}cm (${step.from} → ${step.to}, 교차로 ${step.marks.filter((m) => m.cross).length}개)`;
      case 'turn': return `${step.deg > 0 ? '오른쪽' : '왼쪽'} ${Math.abs(step.deg)}° 회전 (${step.at})`;
      case 'move': return `${step.mm >= 0 ? '직진' : '후진'} ${Math.abs(step.mm)}mm${step.why ? ' · ' + step.why : ''}`;
      case 'grip': return '집게 닫기' + (step.why ? ' · ' + step.why : '');
      case 'release': return '집게 열기' + (step.why ? ' · ' + step.why : '');
      default: return JSON.stringify(step);
    }
  }

  // ---- camera: a tape line crossing the path (perpendicular bar / branch) ----
  /** gray = tape-ready grayscale (colourful pixels already pushed to white). thr = dark threshold.
   *  Returns {present, row (0..1 of image height), rows}. */
  function detectCrossing(gray, W, H, o = {}) {
    const roiTop = o.roiTop ?? 0.4, frac = o.crossFrac ?? 0.35, thr = o.thr;
    const y0 = Math.floor(H * roiTop), minRun = W * frac, maxRows = (H - y0) * 0.4;
    const raw = [];
    let cur = null;
    for (let y = y0; y < H; y++) {
      const row = y * W;
      let best = 0, run = 0;
      for (let x = 0; x < W; x++) {
        if (gray[row + x] < thr) { run++; if (run > best) best = run; } else run = 0;
      }
      if (best < minRun) continue;
      if (cur && y - cur.y1 <= 2) cur.y1 = y; else { cur = { y0: y, y1: y }; raw.push(cur); }
    }
    const bands = raw.filter((b) => b.y1 - b.y0 >= 1 && b.y1 - b.y0 + 1 <= maxRows)
      .map((b) => ({ row: ((b.y0 + b.y1) / 2 + 0.5) / H, rows: b.y1 - b.y0 + 1 }));
    if (!bands.length) return { present: false, row: null, rows: 0, bands };
    const nearest = bands[bands.length - 1];        // lowest in the image = closest to the robot
    return { present: true, row: nearest.row, rows: nearest.rows, bands };
  }

  /** RGBA -> grayscale where colourful pixels (objects, coloured tape) become white, so only
   *  black-ish tape stays dark. satMax = max(R,G,B) - min(R,G,B) allowed for "tape". */
  function tapeGray(px, n, satMax = 70) {
    const g = new Uint8Array(n);
    for (let i = 0, j = 0; i < n; i++, j += 4) {
      const r = px[j], gg = px[j + 1], b = px[j + 2];
      const mx = r > gg ? (r > b ? r : b) : (gg > b ? gg : b);
      const mn = r < gg ? (r < b ? r : b) : (gg < b ? gg : b);
      g[i] = mx - mn > satMax ? 255 : (r * 77 + gg * 150 + b * 29) >> 8;
    }
    return g;
  }

  return { TRACK, practiceTrack, E, S, W, N, TURN_DEG, turnDelta, buildGraph, hasCross, planTo, statesToSteps, compileMission, describe, detectCrossing, tapeGray };
});
