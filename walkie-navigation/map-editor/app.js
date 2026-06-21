// Walkie map editor - static SPA. vanilla canvas 2D.
// PGM coord convention: image row 0 = top; world Y axis = up.
// worldToPx(wx, wy): px = (wx - ox)/res, py = H - (wy - oy)/res
'use strict';

const DEFAULT_LABELS = ['table', 'shelf', 'chair', 'sofa', 'tv', 'food', 'drink'];
const FREE = 254, OCC = 0;

const state = {
  meta: null,
  w: 0, h: 0,
  pixels: null,
  original: null,
  prefix: 'map',
  elements: [],
  labels: DEFAULT_LABELS.slice(),
  selected: null,
  tool: 'pen',
  brush: 3,
  view: { x: 0, y: 0, s: 1 },
  undo: [],
  redo: [],
  drawing: null,
  currentStroke: null,
  dirty: false,
};

function markDirty() { state.dirty = true; }

const $ = (s) => document.querySelector(s);
const canvas = $('#cv');
const ctx = canvas.getContext('2d');
const off = document.createElement('canvas');
const offCtx = off.getContext('2d');

// ───── File I/O ─────────────────────────────────────────────────────

function parsePGM(buf) {
  const u8 = new Uint8Array(buf);
  let i = 0;
  const tok = () => {
    while (i < u8.length) {
      const c = u8[i];
      if (c === 0x23) { while (i < u8.length && u8[i] !== 0x0a) i++; }
      else if (c <= 0x20) i++;
      else break;
    }
    const s = i;
    while (i < u8.length && u8[i] > 0x20) i++;
    return new TextDecoder().decode(u8.slice(s, i));
  };
  const magic = tok();
  if (magic !== 'P5') throw new Error('not P5 PGM: ' + magic);
  const w = parseInt(tok()), h = parseInt(tok()), max = parseInt(tok());
  i++; // single whitespace after maxval per spec
  const pixels = new Uint8Array(u8.buffer, u8.byteOffset + i, w * h).slice();
  return { w, h, max, pixels };
}

function writePGM(w, h, pixels) {
  const header = new TextEncoder().encode(`P5\n${w} ${h}\n255\n`);
  const out = new Uint8Array(header.length + pixels.length);
  out.set(header); out.set(pixels, header.length);
  return out;
}

function parseYAML(text) {
  const r = {};
  for (const line of text.split('\n')) {
    const m = line.match(/^\s*([a-z_]+)\s*:\s*(.+?)\s*$/i);
    if (!m) continue;
    let v = m[2].trim();
    if (v.startsWith('[')) {
      v = v.slice(1, v.lastIndexOf(']')).split(',').map(s => parseFloat(s));
    } else if (!isNaN(parseFloat(v)) && v.match(/^-?\d/)) v = parseFloat(v);
    r[m[1]] = v;
  }
  return r;
}

function writeYAML(m) {
  return `image: ${m.image}\nmode: ${m.mode}\nresolution: ${m.resolution}\norigin: [${m.origin.join(', ')}]\nnegate: ${m.negate}\noccupied_thresh: ${m.occupied_thresh}\nfree_thresh: ${m.free_thresh}\n`;
}

async function loadFolder(files) {
  let pgm = null, og = null, yaml = null, elemJson = null;
  for (const f of files) {
    const n = f.name.toLowerCase();
    if (n.endsWith('_og.pgm')) og = f;
    else if (n.endsWith('.pgm') && !n.endsWith('_keepout.pgm')) pgm = pgm || f;
    else if (n.endsWith('.yaml') || n.endsWith('.yml')) yaml = f;
    else if (n.endsWith('_element.json')) elemJson = f;
  }
  if (!pgm) { status('error: no .pgm in folder'); return; }
  if (!yaml) { status('error: no .yaml in folder'); return; }

  const pgmBuf = await pgm.arrayBuffer();
  const yamlText = await yaml.text();
  const parsed = parsePGM(pgmBuf);
  const meta = parseYAML(yamlText);

  state.meta = meta;
  state.w = parsed.w; state.h = parsed.h;
  state.pixels = parsed.pixels;
  let ogNote = '';
  if (og) {
    const ogBuf = await og.arrayBuffer();
    const ogP = parsePGM(ogBuf);
    if (ogP.w === parsed.w && ogP.h === parsed.h) state.original = ogP.pixels;
    else { state.original = parsed.pixels.slice(); ogNote = ' (og dims mismatch, using map.pgm as baseline)'; }
  } else {
    state.original = parsed.pixels.slice();
    ogNote = ' (no _og.pgm; Restore will revert to as-loaded only)';
  }
  state.prefix = (pgm.name.replace(/_og\.pgm$|\.pgm$/i, '')) || 'map';
  $('#prefix-input').value = state.prefix;

  state.elements = [];
  if (elemJson) {
    try {
      const j = JSON.parse(await elemJson.text());
      state.elements = j.elements || [];
      if (j.labels) {
        for (const l of j.labels) if (!state.labels.includes(l)) state.labels.push(l);
      }
    } catch (e) { console.warn('bad element json', e); }
  }
  rebuildLabelSelect();
  rebuildElemList();
  renderPixels();
  fitView();
  updateInfo();
  $('#export-btn').disabled = false;
  state.dirty = false;
  status(`loaded ${pgm.name} ${parsed.w}×${parsed.h}${ogNote}`);
}

function exportAll() {
  if (!state.meta) return;
  const prefix = ($('#prefix-input').value || 'map').replace(/[^\w\-]/g, '_');
  const meta = { ...state.meta, image: `${prefix}.pgm` };
  download(`${prefix}_og.pgm`, writePGM(state.w, state.h, state.original), 'application/octet-stream');
  download(`${prefix}.pgm`, writePGM(state.w, state.h, state.pixels), 'application/octet-stream');
  download(`${prefix}.yaml`, new TextEncoder().encode(writeYAML(meta)), 'text/yaml');
  download(`${prefix}_element.json`, new TextEncoder().encode(
    JSON.stringify({ labels: state.labels, elements: state.elements }, null, 2)
  ), 'application/json');
  download(`${prefix}_keepout.pgm`, writePGM(state.w, state.h, buildKeepout()), 'application/octet-stream');
  state.dirty = false;
  status(`exported 5 files with prefix "${prefix}"`);
}

window.addEventListener('beforeunload', (ev) => {
  if (state.dirty) { ev.preventDefault(); ev.returnValue = ''; }
});

function buildKeepout() {
  const px = new Uint8Array(state.w * state.h).fill(FREE);
  for (const e of state.elements) {
    if (e.type !== 'nogo') continue;
    const poly = e.coords.map(([wx, wy]) => {
      const p = worldToPx(wx, wy);
      return [p.px, p.py];
    });
    if (poly.length >= 3) rasterPoly(px, state.w, state.h, poly, OCC);
  }
  return px;
}

function download(name, bytes, type) {
  const blob = new Blob([bytes], { type });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ───── Coord helpers ────────────────────────────────────────────────

function worldToPx(wx, wy) {
  const [ox, oy] = state.meta.origin;
  const r = state.meta.resolution;
  return { px: (wx - ox) / r, py: state.h - (wy - oy) / r };
}
function pxToWorld(px, py) {
  const [ox, oy] = state.meta.origin;
  const r = state.meta.resolution;
  return { wx: ox + px * r, wy: oy + (state.h - py) * r };
}
function screenToPx(sx, sy) {
  const rect = canvas.getBoundingClientRect();
  return { px: (sx - rect.left - state.view.x) / state.view.s, py: (sy - rect.top - state.view.y) / state.view.s };
}
function screenToWorld(sx, sy) {
  const p = screenToPx(sx, sy);
  return pxToWorld(p.px, p.py);
}

// ───── Render ───────────────────────────────────────────────────────

function renderPixels() {
  off.width = state.w; off.height = state.h;
  const id = offCtx.createImageData(state.w, state.h);
  for (let p = 0, j = 0; p < state.pixels.length; p++, j += 4) {
    const g = state.pixels[p];
    id.data[j] = g; id.data[j+1] = g; id.data[j+2] = g; id.data[j+3] = 255;
  }
  offCtx.putImageData(id, 0, 0);
}

function draw() {
  const W = canvas.width = canvas.clientWidth;
  const H = canvas.height = canvas.clientHeight;
  ctx.fillStyle = '#555'; ctx.fillRect(0, 0, W, H);
  if (!state.meta) return;

  ctx.save();
  ctx.translate(state.view.x, state.view.y);
  ctx.scale(state.view.s, state.view.s);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, 0, 0);
  drawGrid();
  drawOrigin();
  drawElements();
  drawCursor();
  ctx.restore();

  drawScaleBar();
  $('#zoom-info').textContent = `${(state.view.s * 100).toFixed(0)}%`;
}

function drawGrid() {
  const r = state.meta.resolution;
  const pxPerMeter = 1 / r;
  const target = 80 / state.view.s;
  const targetM = target * r;
  const nice = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50];
  let stepM = nice.find(n => n >= targetM) || 100;
  const stepPx = stepM * pxPerMeter;

  ctx.lineWidth = 1 / state.view.s;
  ctx.strokeStyle = 'rgba(0, 150, 255, 0.25)';
  ctx.beginPath();
  const tl = screenToPx(0, 0);
  const br = screenToPx(canvas.width, canvas.height);
  const o0 = worldToPx(0, 0);
  const startX = Math.floor((tl.px - o0.px) / stepPx) * stepPx + o0.px;
  const startY = Math.floor((tl.py - o0.py) / stepPx) * stepPx + o0.py;
  for (let x = startX; x <= br.px; x += stepPx) {
    ctx.moveTo(x, tl.py); ctx.lineTo(x, br.py);
  }
  for (let y = startY; y <= br.py; y += stepPx) {
    ctx.moveTo(tl.px, y); ctx.lineTo(br.px, y);
  }
  ctx.stroke();
  state._gridStepM = stepM;
}

function drawOrigin() {
  const o = worldToPx(0, 0);
  const r = 8 / state.view.s;
  ctx.lineWidth = 2 / state.view.s;
  ctx.strokeStyle = '#ff3b3b';
  ctx.beginPath(); ctx.moveTo(o.px - r, o.py); ctx.lineTo(o.px + r, o.py); ctx.stroke();
  ctx.strokeStyle = '#3bff3b';
  ctx.beginPath(); ctx.moveTo(o.px, o.py - r); ctx.lineTo(o.px, o.py + r); ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = `${10 / state.view.s}px sans-serif`;
  ctx.fillText('(0,0)', o.px + r, o.py - r);
}

function drawElements() {
  for (const e of state.elements) drawElement(e, state.selected === e.id);
  if (state.drawing) {
    drawElement(state.drawing, true, true);
    // rubber-band line from last vertex to cursor (polygon/line/nogo only)
    if (cursorPx && state.drawing.type !== 'rect' && state.drawing.coords.length) {
      const last = state.drawing.coords[state.drawing.coords.length - 1];
      const lp = worldToPx(last[0], last[1]);
      ctx.save();
      ctx.setLineDash([4 / state.view.s, 4 / state.view.s]);
      ctx.strokeStyle = state.drawing.type === 'nogo' ? '#ff4444' : '#ffeb3b';
      ctx.lineWidth = 1 / state.view.s;
      ctx.beginPath();
      ctx.moveTo(lp.px, lp.py);
      ctx.lineTo(cursorPx.px, cursorPx.py);
      ctx.stroke();
      ctx.restore();
    }
  }
}

function drawElement(e, selected, preview = false) {
  const col = e.type === 'nogo' ? '#ff4444' : selected ? '#ffeb3b' : '#22d3ee';
  ctx.lineWidth = (selected ? 2 : 1.5) / state.view.s;
  ctx.strokeStyle = col;
  ctx.fillStyle = e.type === 'nogo' ? 'rgba(255,68,68,0.25)' : 'rgba(34,211,238,0.15)';
  const pts = e.coords.map(([wx, wy]) => worldToPx(wx, wy));
  if (e.type === 'point') {
    const p = pts[0];
    const r = 5 / state.view.s;
    ctx.beginPath(); ctx.arc(p.px, p.py, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  } else if (pts.length >= 2) {
    ctx.beginPath();
    ctx.moveTo(pts[0].px, pts[0].py);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].px, pts[i].py);
    if (e.closed) { ctx.closePath(); ctx.fill(); }
    ctx.stroke();
    if (preview) {
      for (const p of pts) {
        ctx.beginPath(); ctx.arc(p.px, p.py, 3 / state.view.s, 0, Math.PI * 2); ctx.fill();
      }
    }
  }
  if (!preview && pts.length) {
    ctx.fillStyle = col;
    ctx.font = `${11 / state.view.s}px sans-serif`;
    ctx.fillText(e.label, pts[0].px + 5 / state.view.s, pts[0].py - 5 / state.view.s);
  }
}

function drawCursor() {
  if (!cursorPx || !['pen','eraser','restore'].includes(state.tool)) return;
  ctx.strokeStyle = state.tool === 'pen' ? '#000' : state.tool === 'eraser' ? '#fff' : '#0f0';
  ctx.lineWidth = 1 / state.view.s;
  ctx.beginPath();
  ctx.arc(cursorPx.px, cursorPx.py, state.brush, 0, Math.PI * 2);
  ctx.stroke();
}

function drawScaleBar() {
  if (!state.meta) return;
  const stepM = state._gridStepM || 1;
  const pxLen = stepM / state.meta.resolution * state.view.s;
  const x = 12, y = canvas.height - 18;
  ctx.fillStyle = 'rgba(0,0,0,0.6)'; ctx.fillRect(x - 4, y - 12, pxLen + 8, 22);
  ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x, y); ctx.lineTo(x + pxLen, y);
  ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4);
  ctx.moveTo(x + pxLen, y - 4); ctx.lineTo(x + pxLen, y + 4);
  ctx.stroke();
  ctx.fillStyle = '#fff'; ctx.font = '11px sans-serif';
  const lbl = stepM >= 1 ? `${stepM} m` : `${(stepM * 100).toFixed(0)} cm`;
  ctx.fillText(lbl, x + pxLen / 2 - 12, y - 6);
}

// ───── Pixel tools ──────────────────────────────────────────────────

function paintBrush(px, py) {
  const r = state.brush;
  const r2 = r * r;
  const x0 = Math.max(0, Math.floor(px - r));
  const x1 = Math.min(state.w - 1, Math.ceil(px + r));
  const y0 = Math.max(0, Math.floor(py - r));
  const y1 = Math.min(state.h - 1, Math.ceil(py + r));
  if (x1 < x0 || y1 < y0) return;
  for (let y = y0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++) {
      const dx = x - px, dy = y - py;
      if (dx*dx + dy*dy > r2) continue;
      const i = y * state.w + x;
      let v;
      if (state.tool === 'pen') v = OCC;
      else if (state.tool === 'eraser') v = FREE;
      else v = state.original[i];
      if (state.pixels[i] === v) continue;
      if (!state.currentStroke.has(i)) state.currentStroke.set(i, state.pixels[i]);
      state.pixels[i] = v;
    }
  }
  const id = offCtx.getImageData(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
  for (let y = y0, k = 0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++, k += 4) {
      const g = state.pixels[y * state.w + x];
      id.data[k] = g; id.data[k+1] = g; id.data[k+2] = g; id.data[k+3] = 255;
    }
  }
  offCtx.putImageData(id, x0, y0);
}

// ───── Polygon raster (even-odd scanline) ───────────────────────────

function rasterPoly(out, w, h, poly, val) {
  let minY = h, maxY = 0;
  for (const [, y] of poly) {
    if (y < minY) minY = Math.floor(y);
    if (y > maxY) maxY = Math.ceil(y);
  }
  minY = Math.max(0, minY); maxY = Math.min(h - 1, maxY);
  for (let y = minY; y <= maxY; y++) {
    const xs = [];
    const yy = y + 0.5;
    for (let i = 0; i < poly.length; i++) {
      const [x1, y1] = poly[i];
      const [x2, y2] = poly[(i + 1) % poly.length];
      if ((y1 <= yy && y2 > yy) || (y2 <= yy && y1 > yy)) {
        xs.push(x1 + (yy - y1) / (y2 - y1) * (x2 - x1));
      }
    }
    xs.sort((a, b) => a - b);
    for (let i = 0; i + 1 < xs.length; i += 2) {
      const a = Math.max(0, Math.ceil(xs[i]));
      const b = Math.min(w - 1, Math.floor(xs[i + 1]));
      for (let x = a; x <= b; x++) out[y * w + x] = val;
    }
  }
}

// ───── Undo / redo ──────────────────────────────────────────────────

function pushUndo(act) { state.undo.push(act); state.redo.length = 0; if (state.undo.length > 100) state.undo.shift(); }
function undo() { const a = state.undo.pop(); if (a) { applyInverse(a); state.redo.push(a); markDirty(); rebuildElemList(); draw(); } }
function redoFn() { const a = state.redo.pop(); if (a) { applyForward(a); state.undo.push(a); markDirty(); rebuildElemList(); draw(); } }

function applyInverse(a) {
  if (a.kind === 'pixel') {
    for (const [i, oldV] of a.diffs) state.pixels[i] = oldV;
    renderPixels();
  } else if (a.kind === 'elem-add') {
    state.elements = state.elements.filter(e => e.id !== a.el.id);
  } else if (a.kind === 'elem-del') {
    state.elements.push(a.el);
  }
}
function applyForward(a) {
  if (a.kind === 'pixel') {
    for (const [i] of a.diffs) state.pixels[i] = a.newV.get(i);
    renderPixels();
  } else if (a.kind === 'elem-add') {
    state.elements.push(a.el);
  } else if (a.kind === 'elem-del') {
    state.elements = state.elements.filter(e => e.id !== a.el.id);
  }
}

// ───── Input handling ───────────────────────────────────────────────

let cursorPx = null;
let panning = null;
let painting = false;

canvas.addEventListener('mousedown', (ev) => {
  if (!state.meta) return;
  if (ev.button === 1 || (ev.button === 0 && ev.altKey)) {
    panning = { sx: ev.clientX, sy: ev.clientY, vx: state.view.x, vy: state.view.y };
    return;
  }
  if (ev.button === 2) {
    if (state.drawing && (state.tool === 'polygon' || state.tool === 'line' || state.tool === 'nogo')) {
      finishPoly(false);
    }
    return;
  }
  if (ev.button !== 0) return;

  const w = screenToWorld(ev.clientX, ev.clientY);
  const p = screenToPx(ev.clientX, ev.clientY);

  if (state.tool === 'select') {
    const id = hitTest(w.wx, w.wy);
    state.selected = id;
    rebuildElemList();
    draw();
    return;
  }
  if (['pen', 'eraser', 'restore'].includes(state.tool)) {
    painting = true;
    state.currentStroke = new Map();
    paintBrush(p.px, p.py);
    draw();
  } else if (state.tool === 'point') {
    addElement({ type: 'point', label: currentLabel(), coords: [[w.wx, w.wy]], closed: false });
  } else if (state.tool === 'rect') {
    state.drawing = { id: 'tmp', label: currentLabel(), type: 'rect', coords: [[w.wx, w.wy], [w.wx, w.wy]], closed: true };
  } else if (state.tool === 'polygon' || state.tool === 'line' || state.tool === 'nogo') {
    if (!state.drawing) {
      const t = state.tool === 'nogo' ? 'nogo' : 'polygon';
      state.drawing = { id: 'tmp', label: state.tool === 'nogo' ? 'no-go' : currentLabel(), type: t, coords: [[w.wx, w.wy]], closed: false };
    } else {
      const start = state.drawing.coords[0];
      const startPx = worldToPx(start[0], start[1]);
      const dist = Math.hypot(startPx.px - p.px, startPx.py - p.py);
      if (state.tool !== 'line' && state.drawing.coords.length >= 3 && dist * state.view.s < 8) {
        finishPoly(true);
      } else {
        state.drawing.coords.push([w.wx, w.wy]);
      }
    }
    draw();
  }
});

canvas.addEventListener('mousemove', (ev) => {
  if (!state.meta) return;
  if (panning) {
    state.view.x = panning.vx + (ev.clientX - panning.sx);
    state.view.y = panning.vy + (ev.clientY - panning.sy);
    draw();
    return;
  }
  const p = screenToPx(ev.clientX, ev.clientY);
  const w = screenToWorld(ev.clientX, ev.clientY);
  cursorPx = p;
  status(`world (${w.wx.toFixed(3)}, ${w.wy.toFixed(3)}) m   px (${Math.floor(p.px)}, ${Math.floor(p.py)})`);

  if (painting) {
    paintBrush(p.px, p.py);
  } else if (state.drawing) {
    if (state.drawing.type === 'rect') {
      state.drawing.coords[1] = [w.wx, w.wy];
    }
  }
  draw();
});

canvas.addEventListener('mouseup', (ev) => {
  if (panning) { panning = null; return; }
  if (painting) {
    painting = false;
    if (state.currentStroke && state.currentStroke.size) {
      const newV = new Map();
      for (const [i] of state.currentStroke) newV.set(i, state.pixels[i]);
      pushUndo({ kind: 'pixel', diffs: state.currentStroke, newV });
      markDirty();
    }
    state.currentStroke = null;
  }
  if (state.tool === 'rect' && state.drawing) {
    const c = state.drawing.coords;
    const el = { type: 'rect', label: currentLabel(),
      coords: [[c[0][0], c[0][1]], [c[1][0], c[0][1]], [c[1][0], c[1][1]], [c[0][0], c[1][1]]], closed: true };
    state.drawing = null;
    addElement(el);
  }
});

canvas.addEventListener('contextmenu', (ev) => ev.preventDefault());

canvas.addEventListener('wheel', (ev) => {
  if (!state.meta) return;
  ev.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const mx = ev.clientX - rect.left, my = ev.clientY - rect.top;
  const factor = ev.deltaY < 0 ? 1.15 : 1 / 1.15;
  const ns = Math.max(0.05, Math.min(80, state.view.s * factor));
  state.view.x = mx - (mx - state.view.x) * (ns / state.view.s);
  state.view.y = my - (my - state.view.y) * (ns / state.view.s);
  state.view.s = ns;
  draw();
}, { passive: false });

window.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT') return;
  if (ev.key === 'Escape') {
    if (state.drawing) {
      if (state.drawing.type !== 'rect') finishPoly(false);
      else { state.drawing = null; draw(); }
    }
  } else if (ev.key === 'Delete' || ev.key === 'Backspace') {
    if (state.selected) deleteElement(state.selected);
  } else if ((ev.ctrlKey || ev.metaKey) && ev.key === 'z') { ev.preventDefault(); undo(); }
  else if ((ev.ctrlKey || ev.metaKey) && (ev.key === 'y' || (ev.shiftKey && ev.key === 'Z'))) { ev.preventDefault(); redoFn(); }
});

function finishPoly(closed) {
  if (!state.drawing) return;
  const minPts = closed ? 3 : 2;
  if (state.drawing.coords.length < minPts) { state.drawing = null; draw(); return; }
  const el = { type: state.drawing.type, label: state.drawing.label,
    coords: state.drawing.coords, closed };
  state.drawing = null;
  addElement(el);
}

// ───── Element list ─────────────────────────────────────────────────

let nextId = 1;
function addElement(el) {
  el.id = `e${nextId++}`;
  state.elements.push(el);
  pushUndo({ kind: 'elem-add', el });
  markDirty();
  rebuildElemList();
  draw();
}
function deleteElement(id) {
  const idx = state.elements.findIndex(e => e.id === id);
  if (idx < 0) return;
  const el = state.elements.splice(idx, 1)[0];
  if (state.selected === id) state.selected = null;
  pushUndo({ kind: 'elem-del', el });
  markDirty();
  rebuildElemList();
  draw();
}
function renameElement(id, label) {
  const el = state.elements.find(e => e.id === id);
  if (!el || !label || el.label === label) return;
  el.label = label;
  if (!state.labels.includes(label)) { state.labels.push(label); rebuildLabelSelect(); }
  markDirty();
  rebuildElemList();
  draw();
}

// hit-test in world coords; returns element id or null
function hitTest(wx, wy) {
  const tol = 6 / state.view.s * state.meta.resolution; // 6 screen px in meters
  // iterate top-down (later-drawn first)
  for (let i = state.elements.length - 1; i >= 0; i--) {
    const e = state.elements[i];
    const pts = e.coords;
    if (e.type === 'point') {
      const [x, y] = pts[0];
      if (Math.hypot(x - wx, y - wy) < tol) return e.id;
    } else if (e.closed && pts.length >= 3) {
      if (pointInPoly(wx, wy, pts)) return e.id;
    } else if (pts.length >= 2) {
      for (let k = 0; k < pts.length - 1; k++) {
        if (distToSeg(wx, wy, pts[k], pts[k + 1]) < tol) return e.id;
      }
    }
  }
  return null;
}
function pointInPoly(x, y, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}
function distToSeg(x, y, a, b) {
  const [ax, ay] = a, [bx, by] = b;
  const dx = bx - ax, dy = by - ay;
  const len2 = dx*dx + dy*dy;
  if (len2 === 0) return Math.hypot(x - ax, y - ay);
  const t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / len2));
  return Math.hypot(x - (ax + t * dx), y - (ay + t * dy));
}

function rebuildElemList() {
  const ul = $('#elem-list');
  while (ul.firstChild) ul.removeChild(ul.firstChild);
  for (const e of state.elements) {
    const li = document.createElement('li');
    if (state.selected === e.id) li.className = 'sel';
    const span = document.createElement('span');
    span.textContent = e.label + ' ';
    const em = document.createElement('em');
    em.style.color = '#888';
    em.textContent = e.type;
    span.appendChild(em);
    const x = document.createElement('span');
    x.className = 'x'; x.textContent = '×';
    li.appendChild(span); li.appendChild(x);
    li.onclick = (ev) => { if (ev.target !== x) { state.selected = e.id; rebuildElemList(); draw(); } };
    li.ondblclick = (ev) => {
      if (ev.target === x) return;
      const input = document.createElement('input');
      input.value = e.label; input.size = 12;
      input.onclick = (e2) => e2.stopPropagation();
      input.onkeydown = (e2) => { if (e2.key === 'Enter') input.blur(); if (e2.key === 'Escape') { input.value = e.label; input.blur(); } };
      input.onblur = () => renameElement(e.id, input.value.trim());
      li.replaceChild(input, span);
      input.focus(); input.select();
    };
    x.onclick = (ev) => { ev.stopPropagation(); deleteElement(e.id); };
    ul.appendChild(li);
  }
}

// ───── Labels ───────────────────────────────────────────────────────

function rebuildLabelSelect() {
  const sel = $('#label-select');
  while (sel.firstChild) sel.removeChild(sel.firstChild);
  for (const l of state.labels) {
    const o = document.createElement('option'); o.value = l; o.textContent = l;
    sel.appendChild(o);
  }
  saveLabels();
}
function currentLabel() { return $('#label-select').value || state.labels[0] || 'unknown'; }
function saveLabels() { try { localStorage.setItem('walkie-labels', JSON.stringify(state.labels)); } catch {} }
function loadLabels() {
  try {
    const s = JSON.parse(localStorage.getItem('walkie-labels') || '[]');
    for (const l of s) if (!state.labels.includes(l)) state.labels.push(l);
  } catch {}
}

// ───── UI wiring ────────────────────────────────────────────────────

$('#folder-input').addEventListener('change', (ev) => loadFolder(ev.target.files));
$('#export-btn').addEventListener('click', exportAll);
$('#brush').addEventListener('input', (ev) => { state.brush = +ev.target.value; $('#brush-val').textContent = ev.target.value; draw(); });
$('#add-label').addEventListener('click', () => {
  const v = prompt('new label:');
  if (!v) return;
  const t = v.trim();
  if (!t || state.labels.includes(t)) return;
  state.labels.push(t);
  rebuildLabelSelect();
  $('#label-select').value = t;
});
$('#undo').addEventListener('click', undo);
$('#redo').addEventListener('click', redoFn);
$('#fit').addEventListener('click', fitView);
$('#clear-elems').addEventListener('click', () => {
  if (!state.elements.length) return;
  if (!confirm(`delete all ${state.elements.length} elements?`)) return;
  for (const e of [...state.elements]) deleteElement(e.id);
});

document.querySelectorAll('button[data-tool]').forEach(b => {
  b.addEventListener('click', () => setTool(b.dataset.tool));
});
function setTool(t) {
  state.tool = t;
  state.drawing = null;
  document.querySelectorAll('button[data-tool]').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
  draw();
}

function fitView() {
  if (!state.meta) return;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  const s = Math.min(W / state.w, H / state.h) * 0.95;
  state.view.s = s;
  state.view.x = (W - state.w * s) / 2;
  state.view.y = (H - state.h * s) / 2;
  draw();
}

function updateInfo() {
  const m = state.meta;
  const info = $('#map-info');
  while (info.firstChild) info.removeChild(info.firstChild);
  const rows = [
    ['dims', `${state.w} × ${state.h} px`],
    ['resolution', `${m.resolution} m/px`],
    ['size', `${(state.w * m.resolution).toFixed(2)} × ${(state.h * m.resolution).toFixed(2)} m`],
    ['origin', `[${m.origin.join(', ')}]`],
    ['occ/free', `${m.occupied_thresh} / ${m.free_thresh}`],
  ];
  for (const [k, v] of rows) {
    const d = document.createElement('div');
    d.textContent = `${k}: `;
    const b = document.createElement('b');
    b.textContent = v;
    d.appendChild(b);
    info.appendChild(d);
  }
}

function status(t) { $('#status').textContent = t; }

window.addEventListener('resize', () => draw());

loadLabels();
rebuildLabelSelect();
setTool('pen');
draw();
status('drag a map folder to begin');

// ───── Self-check (run in console: window._test()) ──────────────────
window._test = function () {
  // ponytail: minimal round-trip, asserts in console
  const w = 4, h = 3;
  const px = new Uint8Array([0,128,255,0, 50,100,150,200, 10,20,30,40]);
  const buf = writePGM(w, h, px);
  const back = parsePGM(buf.buffer);
  console.assert(back.w === w && back.h === h, 'pgm dims');
  for (let i = 0; i < px.length; i++) console.assert(back.pixels[i] === px[i], 'pgm bytes ' + i);

  const yaml = 'image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.5, -2.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n';
  const m = parseYAML(yaml);
  console.assert(m.resolution === 0.05 && m.origin[0] === -1.5, 'yaml parse');

  const out = new Uint8Array(16).fill(254);
  rasterPoly(out, 4, 4, [[1,1],[3,1],[3,3],[1,3]], 0);
  let blackCount = 0;
  for (const v of out) if (v === 0) blackCount++;
  console.assert(blackCount >= 4, 'raster filled, got ' + blackCount);

  console.log('self-check ok');
};
