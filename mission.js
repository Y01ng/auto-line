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

  const HOT_PEN = 90;                   // cm-equivalent: turning while the gripper points at / sweeps an object
  /** Is there an object within gripper reach straight ahead of node n in direction dir? */
  function hotAt(G, n, dir, objNodes, reachCm) {
    return reachCm > 0 && G.get(n).adj.some((a) => a.dir === dir && objNodes.has(a.to) && a.len <= reachCm);
  }
  function turnPen(G, n, h, dir, objNodes, reachCm) {
    const td = turnDelta(h, dir);
    if (!td || !(reachCm > 0)) return 0;
    if (hotAt(G, n, h, objNodes, reachCm)) return HOT_PEN;                       // gripper is at an object: don't swing it
    if (td === 2 && hotAt(G, n, (h + 1) % 4, objNodes, reachCm)) return HOT_PEN;  // U-turn passes the right-hand side
    return 0;
  }

  function dijkstra(G, start, startHeading, blocked, objNodes, reachCm) {
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
        const nd = d + a.len + TURN_COST[turnDelta(h, a.dir)] + (objNodes ? turnPen(G, n, h, a.dir, objNodes, reachCm) : 0);
        const k = key(a.to, a.dir);
        if (nd < (dist.has(k) ? dist.get(k) : Infinity)) { dist.set(k, nd); prev.set(k, key(n, h)); pq.push([nd, a.to, a.dir]); }
      }
    }
    return { dist, prev, key };
  }

  /** Cheapest way to stand on a neighbour of `target`, facing it, at least `minApproachCm` away. */
  function planTo(G, pose, target, blockedIn, minApproachCm, o = {}) {
    const blocked = new Set(blockedIn);
    blocked.add(target);
    blocked.delete(pose.node);
    // o.reachCm: how far ahead the gripper reaches. o.pick: target is an object (don't swing into it).
    const reach = o.reachCm || 0;
    const objNodes = new Set(blockedIn);
    if (o.pick) objNodes.add(target);
    objNodes.delete(pose.node);
    const { dist, prev, key } = dijkstra(G, pose.node, pose.heading, blocked, objNodes, reach);
    let best = null;
    for (const a of G.get(target).adj) {
      const from = a.to, dirToT = (a.dir + 2) % 4;
      if (blocked.has(from) || a.len < minApproachCm) continue;
      for (let h = 0; h < 4; h++) {
        const k = key(from, h);
        if (!dist.has(k)) continue;
        // turning toward an object that is already within reach would hit it sideways: come in straight
        const near = !!o.pick && h !== dirToT && a.len <= reach;
        const c = dist.get(k) + TURN_COST[turnDelta(h, dirToT)] + a.len + (near ? HOT_PEN : 0) + turnPen(G, from, h, dirToT, objNodes, reach);
        if (!best || c < best.cost) best = { cost: c, from, heading: h, dirToT, len: a.len, near };
      }
    }
    if (!best) return null;
    const states = [];
    for (let k = key(best.from, best.heading); k; k = prev.get(k)) {
      const [n, h] = k.split('|');
      states.unshift({ node: n, heading: +h });
    }
    return { cost: best.cost, states, from: best.from, approachDir: best.dirToT, approachLen: best.len, near: best.near };
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
    const grab = o.grab, jaw = o.jaw ?? 100;             // jaw: back-off needed before the open gripper may swing
    const reachCm = (grab + 70) / 10;
    let freeStubs = stubs.slice();
    let pose = Object.assign({}, T.start);
    let folded = false;                                  // gripper closed empty after a drop -> open before the next pick
    const steps = [];
    try {
      targets.forEach((tid, i) => {
        const obj = objs.find((x) => x.id === tid);
        if (!obj) throw new Error('없는 물체 번호: ' + tid);
        const blocked = new Set(objs.filter((x) => !x.done && x.node !== obj.node).map((x) => x.node));
        const pick = planTo(G, pose, obj.node, blocked, Math.max(0, grab - 40) / 10, { reachCm, pick: true });
        if (!pick) throw new Error(`${obj.name}까지 갈 길이 없어요`);
        const a = statesToSteps(G, pick.states, pose.heading);
        const seq = a.steps.slice();
        if (a.heading !== pick.approachDir) {
          const t = { op: 'turn', deg: TURN_DEG[turnDelta(a.heading, pick.approachDir)], at: pick.from };
          if (pick.near) t.warn = '집게가 물체를 옆에서 칠 수 있어요';
          seq.push(t);
        }
        if (folded) {                                    // open once the gripper points away from the delivered object
          seq.splice(seq.length && seq[0].op === 'turn' ? 1 : 0, 0, { op: 'release', why: '집게 다시 열기' });
          folded = false;
        }
        steps.push(...seq);
        const appr = Math.max(0, Math.round(pick.approachLen * 10 - grab));
        if (appr) steps.push({ op: 'move', mm: appr, why: `${obj.name} 앞까지` });
        steps.push({ op: 'grip', why: `${obj.name} 집기`, pick: true });
        if (appr) steps.push({ op: 'move', mm: -appr, why: '교차로로 후진' });
        obj.done = true;
        pose = { node: pick.from, heading: pick.approachDir };

        // nearest 전달존 end that is still empty
        const blocked2 = new Set(objs.filter((x) => !x.done).map((x) => x.node));
        if (!freeStubs.length) freeStubs = stubs.slice();
        let drop = null, stub = null;
        for (const s of freeStubs) {
          const d = planTo(G, pose, s, blocked2, 0, { reachCm });
          if (d && (!drop || d.cost < drop.cost)) { drop = d; stub = s; }
        }
        if (!drop) throw new Error('전달존까지 갈 길이 없어요');
        freeStubs.splice(freeStubs.indexOf(stub), 1);
        const b = statesToSteps(G, drop.states, pose.heading);
        steps.push(...b.steps);
        if (b.heading !== drop.approachDir) steps.push({ op: 'turn', deg: TURN_DEG[turnDelta(b.heading, drop.approachDir)], at: drop.from });
        const dropAppr = Math.max(0, Math.round(drop.approachLen * 10 - o.dropOffset - grab));
        if (dropAppr) steps.push({ op: 'move', mm: dropAppr, why: '전달존으로' });
        steps.push({ op: 'release', why: `${obj.name} 내려놓기 (${stub})` });
        const last = i === targets.length - 1;
        if (last || dropAppr >= jaw) {
          if (dropAppr) steps.push({ op: 'move', mm: -dropAppr, why: '교차로로 후진' });
        } else {
          // the object sits between the open jaws: turning now would drag it away.
          // Back off, close the empty gripper, return to the crossing, then turn.
          steps.push({ op: 'move', mm: -jaw, why: '물체에서 물러나기' });
          steps.push({ op: 'grip', why: '빈 집게 접기' });
          if (jaw - dropAppr) steps.push({ op: 'move', mm: jaw - dropAppr, why: '교차로로' });
          folded = true;
        }
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
      case 'turn': return `${step.deg > 0 ? '오른쪽' : '왼쪽'} ${Math.abs(step.deg)}° 회전 (${step.at})` + (step.warn ? ' ⚠ ' + step.warn : '');
      case 'move': return `${step.mm >= 0 ? '직진' : '후진'} ${Math.abs(step.mm)}mm${step.why ? ' · ' + step.why : ''}`;
      case 'grip': return '집게 닫기' + (step.why ? ' · ' + step.why : '');
      case 'release': return '집게 열기' + (step.why ? ' · ' + step.why : '');
      default: return JSON.stringify(step);
    }
  }

  // ---- camera: follow ONE line, ignore other dark things ----
  /** Forget where the line was (new leg / after a turn): look around the image centre again. */
  function resetTrack(st) { st.exp = null; return st; }

  /**
   * Gated line tracker. gray = tape-ready grayscale, thr = dark threshold, st = state object kept
   * between frames. A dark run only counts as "the line" when it is (1) about as wide as the tape
   * and (2) close to where the line was in the previous frame (or to the image centre after
   * resetTrack). Of several candidates the one nearest to the expected position wins - not the
   * longest. Same result shape as SpikeCore.detectLine (+ gate / exp for drawing).
   */
  function trackLine(gray, W, H, thr, st, o = {}) {
    const roiTop = o.roiTop ?? 0.4, nb = 3, gate0 = o.gate ?? 0.16, gateMax = o.gateMax ?? 0.34;
    const y0 = Math.min(H - nb, Math.max(0, Math.floor(H * roiTop)));
    const center = (W - 1) / 2;
    if (st.exp == null) { st.exp = center; st.age = 99; st.cx = [null, null, null]; if (st.w === undefined) st.w = null; }
    if (st.age > 5) st.exp += (center - st.exp) * 0.15;          // lost: drift back to the middle
    if (st.age > 30) st.w = null;                                // lost for long: relearn the tape width
    const gate = W * (st.age <= 5 ? gate0 : Math.min(gateMax, gate0 + 0.03 * (st.age - 5)));
    const wMin = st.w ? Math.max(2, st.w * 0.45) : Math.max(3, W * 0.03);
    const wMax = st.w ? Math.min(W * 0.35, st.w * 2.2) : W * 0.3;
    const bandH = (H - y0) / nb;
    const res = { found: false, error: 0, thr, y0, bands: [], reason: '', gate, exp: st.exp };
    const widths = [];
    for (let b = 0; b < nb; b++) {
      const ya = Math.round(y0 + b * bandH), yb = Math.round(y0 + (b + 1) * bandH);
      const expB = st.age <= 5 && st.cx[b] != null ? st.cx[b] : st.exp;
      const centers = [];
      for (let y = ya; y < yb; y++) {
        const row = y * W;
        let run = 0, bestC = null, bestD = 1e9, bestW = 0;
        for (let x = 0; x <= W; x++) {
          if (x < W && gray[row + x] < thr) { run++; continue; }
          if (run >= wMin && run <= wMax) {
            const c = x - (run + 1) / 2, d = Math.abs(c - expB);
            if (d < bestD) { bestD = d; bestC = c; bestW = run; }
          }
          run = 0;
        }
        if (bestC != null && bestD <= gate) { centers.push(bestC); widths.push(bestW); }
      }
      let cx = null;
      if (centers.length >= Math.max(2, (yb - ya) * 0.4)) { centers.sort((p, q) => p - q); cx = centers[centers.length >> 1]; }
      res.bands.push({ ya, yb, cx });
    }
    const weights = [0.2, 0.3, 0.5];                             // the band nearest to the robot counts most
    let ws = 0, es = 0, nearest = null;
    res.bands.forEach((bd, i) => {
      if (bd.cx == null) return;
      ws += weights[i]; es += weights[i] * ((bd.cx - center) / (W / 2)); nearest = bd.cx;
    });
    if (ws > 0) {
      res.found = true;
      res.error = Math.max(-1, Math.min(1, es / ws));
      res.bands.forEach((bd, i) => { st.cx[i] = bd.cx; });
      st.exp = nearest; st.age = 0;
      if (widths.length >= 4) {
        widths.sort((p, q) => p - q);
        const m = widths[widths.length >> 1];
        st.w = st.w ? st.w * 0.8 + m * 0.2 : m;
      }
    } else { st.age++; res.reason = 'no-line'; }
    return res;
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
    const minRows = o.minRows ?? 2;                 // thinner than the tape (tile gaps, cables) = not a cross line
    const bands = raw.filter((b) => b.y1 - b.y0 + 1 >= minRows && b.y1 - b.y0 + 1 <= maxRows)
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

  return { TRACK, practiceTrack, E, S, W, N, TURN_DEG, turnDelta, buildGraph, hasCross, planTo, statesToSteps, compileMission, describe, resetTrack, trackLine, detectCrossing, tapeGray };
});
