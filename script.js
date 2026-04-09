/* ================================================================
   EK301 Truss Analyzer  —  script.js
   Boston University  ·  Engineering Mechanics  ·  Spring 2026
   ================================================================

   Algorithm follows EK301 Project 2 document (Section 2.4):
     Eq.(1)  M = 2J − 3
     Eq.(2)  C = C_L · L_total + C_J · J
     Eq.(3-8) Sx, Sy support matrices
     Eq.(9)  Load vector L
     Eq.(10) [A][T] + [L] = [0]
     Eq.(11) T = −A⁻¹L
     Eq.(12) F_crit = C_fit(L_0/L)^α ± U_fit
     Eq.(13) T_m = R_m × W_ℓ
     Eq.(16) W_fail = −P_crit,c / R_c
   ================================================================ */

'use strict';

/* ──────────────────────────────────────────────────────────────
   1.  PRESET TRUSS DATA (UPDATED FOR EK301 LENGTH CONSTRAINTS)
   ────────────────────────────────────────────────────────────── */

const PRESETS = [
  {
    name: 'Truss 1: Small Pratt',
    subtitle: '5 Joints · 7 Members  (Compliant Span: 28in)',
    joints:  [[0,0], [14,0], [28,0], [7,10], [21,10]],          
    members: [[0,1], [1,2], [0,3], [1,3], [1,4], [2,4], [3,4]],
    pinJoint:    0,   // J1
    rollerJoint: 2,   // J3
    loadJoint:   1,   // J2
    W: 32             // oz
  },
  {
    name: 'Truss 2: Warren Full',
    subtitle: '7 Joints · 11 Members  (Compliant Span: 30in)',
    joints:  [[0,0], [10,0], [20,0], [30,0], [5,8.66], [15,8.66], [25,8.66]],
    members: [[0,1], [1,2], [2,3], [4,5], [5,6], [0,4], [1,4], [1,5], [2,5], [2,6], [3,6]],
    pinJoint:    0,   // J1
    rollerJoint: 3,   // J4
    loadJoint:   2,   // J3
    W: 32
  },
  {
    name: 'Truss 3: Warren Standard',
    subtitle: '5 Joints · 7 Members  (Compliant Span: 26in)',
    joints:  [[0,0], [13,0], [26,0], [6.5,11], [19.5,11]],
    members: [[0,1], [1,2], [0,3], [1,3], [1,4], [2,4], [3,4]],
    pinJoint:    0,   // J1
    rollerJoint: 2,   // J3
    loadJoint:   1,   // J2
    W: 32
  }
];

/* ──────────────────────────────────────────────────────────────
   2.  APPLICATION STATE
   ────────────────────────────────────────────────────────────── */

let currentTrussIdx = -1;
let currentResults  = null;
let viewMode        = 'visual';   // 'visual' | 'matrix'
let hoveredMember   = -1;
let canvasTransform = null;       // { tx, ty } closures for hit-testing

/* ──────────────────────────────────────────────────────────────
   3.  MATH UTILITIES
   ────────────────────────────────────────────────────────────── */

/**
 * Solve Ax = b via Gauss–Jordan with partial pivoting.
 * Returns the solution vector x.
 */
function linsolve(A, b) {
  const n = A.length;
  // Build augmented matrix [A | b]
  const aug = A.map((row, i) => [...row, b[i]]);

  for (let col = 0; col < n; col++) {
    // Partial pivot
    let maxRow = col;
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(aug[row][col]) > Math.abs(aug[maxRow][col])) maxRow = row;
    }
    if (maxRow !== col) [aug[col], aug[maxRow]] = [aug[maxRow], aug[col]];

    const pivot = aug[col][col];
    if (Math.abs(pivot) < 1e-12) continue;

    // Eliminate all other rows
    for (let row = 0; row < n; row++) {
      if (row === col) continue;
      const f = aug[row][col] / pivot;
      for (let k = col; k <= n; k++) aug[row][k] -= f * aug[col][k];
    }
    // Normalize pivot row
    for (let k = col; k <= n; k++) aug[col][k] /= pivot;
  }
  return aug.map(r => r[n]);
}

/** Format number for display (suppress near-zero values) */
function fmt(x, d = 4) {
  if (Math.abs(x) < 1e-9) return '0';
  const r = parseFloat(x.toFixed(d));
  return r.toString();
}

function fmtF(x) { return fmt(x, 3); }   // forces  (3 dp)
function fmtL(x) { return fmt(x, 4); }   // lengths (4 dp)
function fmtC(x) { return fmt(x, 2); }   // costs   (2 dp)

/** Euclidean distance from point (px,py) to segment (x1,y1)-(x2,y2) */
function distToSeg(px, py, x1, y1, x2, y2) {
  const dx = x2 - x1, dy = y2 - y1;
  const len2 = dx*dx + dy*dy;
  if (len2 < 1e-12) return Math.hypot(px - x1, py - y1);
  let t = ((px-x1)*dx + (py-y1)*dy) / len2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (x1+t*dx), py - (y1+t*dy));
}

/* ──────────────────────────────────────────────────────────────
   4.  TRUSS SOLVER
   ────────────────────────────────────────────────────────────── */

function solveTruss(truss) {
  const { joints, members, pinJoint, rollerJoint, loadJoint, W } = truss;
  const J = joints.length;
  const M = members.length;

  /* ── Member lengths ── */
  const lengths = members.map(([j1,j2]) => {
    const dx = joints[j2][0] - joints[j1][0];
    const dy = joints[j2][1] - joints[j1][1];
    return Math.hypot(dx, dy);
  });

  /* ── Connection matrix C  (J × M) ── */
  const C = Array.from({length: J}, () => new Array(M).fill(0));
  members.forEach(([j1,j2], m) => { C[j1][m] = 1; C[j2][m] = 1; });

  /* ── Support matrices Sx, Sy  (J × 3)
       Col 0 = R_Px (x at pin), Col 1 = R_Py (y at pin), Col 2 = R_Ry (y at roller)
       Follows Eqs.(3-8) of the assignment ── */
  const Sx = Array.from({length: J}, () => [0,0,0]);
  const Sy = Array.from({length: J}, () => [0,0,0]);
  Sx[pinJoint][0]    = 1;   // R_Px enters x-eq at pin
  Sy[pinJoint][1]    = 1;   // R_Py enters y-eq at pin
  Sy[rollerJoint][2] = 1;   // R_Ry enters y-eq at roller

  /* ── Load vector L  (2J × 1) — Eq.(9) ── */
  const Lvec = new Array(2*J).fill(0);
  Lvec[J + loadJoint] = -W;   // downward load at load joint

  /* ── Equilibrium matrix A  (2J × M+3) — Section 2.4.3 ──
       Rows 0..J-1   : ΣFx at each joint
       Rows J..2J-1  : ΣFy at each joint
       Cols 0..M-1   : member tensions T_m
       Col  M        : R_Px
       Col  M+1      : R_Py
       Col  M+2      : R_Ry  ── */
  const A = Array.from({length: 2*J}, () => new Array(M+3).fill(0));

  members.forEach(([j1,j2], m) => {
    const dx = joints[j2][0] - joints[j1][0];
    const dy = joints[j2][1] - joints[j1][1];
    const L  = lengths[m];
    A[j1][m]     =  dx/L;   // x-eq at j1
    A[j2][m]     = -dx/L;   // x-eq at j2 (opposite direction)
    A[J+j1][m]   =  dy/L;   // y-eq at j1
    A[J+j2][m]   = -dy/L;   // y-eq at j2
  });
  A[pinJoint][M]        = 1;  // R_Px in x-eq
  A[J+pinJoint][M+1]    = 1;  // R_Py in y-eq
  A[J+rollerJoint][M+2] = 1;  // R_Ry in y-eq

  /* ── Solve  A T = −L  →  T = −A⁻¹ L  (Eq. 11) ── */
  const Tsol = linsolve(A, Lvec.map(x => -x));

  const memberForces = Tsol.slice(0, M);
  const RPx = Tsol[M], RPy = Tsol[M+1], RRy = Tsol[M+2];

  /* ── Buckling analysis — Eq.(12), (13), (16) ──
       F_crit = C_fit(L_0/L)^α ± U_fit
       C_fit = 37.5 oz, L_0 = 10 in, α = 2, U_fit = 10 oz  ── */
  const C_FIT = 37.5, L0 = 10, ALPHA = 2, U_FIT = 10;

  const buckle = memberForces.map((Tm, m) => {
    const Lm   = lengths[m];
    const Pcrit = C_FIT * Math.pow(L0/Lm, ALPHA);
    const Rm    = Tm / W;                       // Eq.(13): T_m = R_m × W
    const isC   = Tm < -1e-9;
    const isZ   = Math.abs(Tm) < 1e-9;
    const Wfail  = isC ? Pcrit / Math.abs(Rm) : Infinity;  // Eq.(16)
    const deltaW = isC ? U_FIT / Math.abs(Rm) : 0;
    return { Pcrit, Rm, Wfail, deltaW, isCompression: isC, isZero: isZ };
  });

  /* ── Critical member (smallest Wfail) ── */
  let critMember = -1, minWfail = Infinity;
  buckle.forEach((d, m) => {
    if (d.isCompression && d.Wfail < minWfail) {
      minWfail = d.Wfail; critMember = m;
    }
  });
  const critDW = critMember >= 0 ? buckle[critMember].deltaW : 0;

  /* ── Cost — Eq.(2)  C = C_L · L_total + C_J · J,  C_L=1, C_J=10 ── */
  const Ltotal = lengths.reduce((s,l) => s+l, 0);
  const cost   = Ltotal + 10*J;
  const ratio  = minWfail < Infinity ? minWfail / cost : 0;

  return {
    truss, J, M,
    lengths, memberForces,
    RPx, RPy, RRy,
    buckle, critMember,
    Wfail: minWfail, deltaW: critDW,
    Ltotal, cost, ratio,
    C, Sx, Sy, A, Lvec, T: Tsol
  };
}

/* ──────────────────────────────────────────────────────────────
   5.  CANVAS DRAWING
   ────────────────────────────────────────────────────────────── */

function getTransform(joints, cw, ch) {
  const xs = joints.map(j => j[0]);
  const ys = joints.map(j => j[1]);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);

  const rangeX = maxX - minX || 1;
  const rangeY = maxY - minY || 1;
  const padX = 85, padY = 75;

  // Ensure truss doesn't look too flat (minimum visual y-range = 25% of x-range)
  const visRangeY = Math.max(rangeY, rangeX * 0.26);

  const sx = (cw - 2*padX) / rangeX;
  const sy = (ch - 2*padY) / visRangeY;
  const scale = Math.min(sx, sy);

  const dw = rangeX * scale;
  const dh = visRangeY * scale;
  const ox = padX + (cw - 2*padX - dw) / 2;
  const oy = ch - padY - (ch - 2*padY - dh) / 2;

  const tx = x => ox + (x - minX) * scale;
  const ty = y => oy - (y - minY) * scale;
  return { tx, ty, scale };
}

function drawTruss(results, hovered = -1) {
  const canvas = document.getElementById('trussCanvas');
  const ctx = canvas.getContext('2d');
  const cw = canvas.width, ch = canvas.height;

  ctx.clearRect(0, 0, cw, ch);
  ctx.fillStyle = '#f8fafc';
  ctx.fillRect(0, 0, cw, ch);

  if (!results) return;

  const { truss, memberForces, buckle, critMember } = results;
  const { joints, members, pinJoint, rollerJoint, loadJoint, W } = truss;

  const { tx, ty } = getTransform(joints, cw, ch);
  canvasTransform = { tx, ty };

  /* ── Draw members ── */
  members.forEach(([j1,j2], m) => {
    const x1 = tx(joints[j1][0]), y1 = ty(joints[j1][1]);
    const x2 = tx(joints[j2][0]), y2 = ty(joints[j2][1]);
    const force = memberForces[m];
    const isCrit   = m === critMember;
    const isHov    = m === hovered;
    const isZero   = buckle[m].isZero;
    const isComp   = buckle[m].isCompression;

    ctx.save();

    // Critical member glow
    if (isCrit) {
      ctx.shadowBlur  = 14;
      ctx.shadowColor = '#f59e0b';
    }

    ctx.lineWidth = isHov ? 7 : (isCrit ? 6 : 3.5);
    ctx.lineCap   = 'round';

    if (isZero)       ctx.strokeStyle = '#94a3b8';
    else if (isComp)  ctx.strokeStyle = isCrit ? '#f59e0b' : '#ef4444';
    else              ctx.strokeStyle = '#10b981';

    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.restore();

    /* Force label */
    if (memberForces.length > 0) {
      const mx = (x1+x2)/2, my = (y1+y2)/2;
      const dx = x2-x1, dy2 = y2-y1;
      const len = Math.hypot(dx, dy2) || 1;
      const nx = -dy2/len * 13, ny = dx/len * 13;
      const label = isZero ? 'ZFM' : (Math.abs(force).toFixed(2) + (isComp ? 'C' : 'T'));
      ctx.save();
      ctx.font = isHov ? 'bold 12px sans-serif' : '11px sans-serif';
      ctx.fillStyle = isZero ? '#94a3b8' : (isComp ? (isCrit ? '#b45309' : '#dc2626') : '#059669');
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, mx + nx, my + ny);
      ctx.restore();
    }

    /* Member number (small, gray) */
    const mx2 = (x1+x2)/2, my2 = (y1+y2)/2;
    ctx.save();
    ctx.font = '9px sans-serif';
    ctx.fillStyle = '#cbd5e1';
    ctx.textAlign = 'center';
    ctx.fillText('m'+(m+1), mx2, my2 + 16);
    ctx.restore();
  });

  /* ── Draw supports ── */
  drawPin   (ctx, tx(joints[pinJoint][0]),    ty(joints[pinJoint][1]));
  drawRoller(ctx, tx(joints[rollerJoint][0]), ty(joints[rollerJoint][1]));

  /* ── Draw load arrow ── */
  drawLoad(ctx, tx(joints[loadJoint][0]), ty(joints[loadJoint][1]), W);

  /* ── Draw joints (on top of everything) ── */
  joints.forEach(([x,y], j) => {
    const cx = tx(x), cy = ty(y);
    let fill = '#334155';
    if (j === pinJoint)    fill = '#3b82f6';
    if (j === rollerJoint) fill = '#8b5cf6';
    if (j === loadJoint)   fill = '#f97316';

    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, 7, 0, 2*Math.PI);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();

    ctx.save();
    ctx.font = 'bold 11px sans-serif';
    ctx.fillStyle = '#1e293b';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText('J'+(j+1), cx, cy - 10);
    ctx.restore();
  });
}

function drawPin(ctx, x, y) {
  const s = 15;
  ctx.save();
  ctx.fillStyle = '#3b82f6';
  ctx.strokeStyle = '#1d4ed8';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x-s, y+s*1.15);
  ctx.lineTo(x+s, y+s*1.15);
  ctx.closePath();
  ctx.fill(); ctx.stroke();
  const gy = y + s*1.15;
  ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(x-s-4, gy); ctx.lineTo(x+s+4, gy); ctx.stroke();
  for (let i = -s; i <= s+2; i += 7) {
    ctx.beginPath(); ctx.moveTo(x+i, gy); ctx.lineTo(x+i-5, gy+8); ctx.stroke();
  }
  ctx.restore();
}

function drawRoller(ctx, x, y) {
  const s = 13;
  ctx.save();
  ctx.fillStyle = '#8b5cf6';
  ctx.strokeStyle = '#6d28d9';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x-s, y+s*1.15);
  ctx.lineTo(x+s, y+s*1.15);
  ctx.closePath();
  ctx.fill(); ctx.stroke();
  const gy = y + s*1.15 + 7;
  [-6,6].forEach(dx => {
    ctx.beginPath();
    ctx.arc(x+dx, gy, 4.5, 0, 2*Math.PI);
    ctx.fillStyle = '#8b5cf6'; ctx.fill();
  });
  const gl = gy + 5;
  ctx.strokeStyle = '#8b5cf6'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(x-s-4, gl); ctx.lineTo(x+s+4, gl); ctx.stroke();
  for (let i = -s; i <= s+2; i += 7) {
    ctx.beginPath(); ctx.moveTo(x+i, gl); ctx.lineTo(x+i-5, gl+8); ctx.stroke();
  }
  ctx.restore();
}

function drawLoad(ctx, x, y, W) {
  const len = 38;
  ctx.save();
  ctx.strokeStyle = '#f97316';
  ctx.fillStyle   = '#f97316';
  ctx.lineWidth   = 2.5;
  ctx.lineCap     = 'round';
  ctx.beginPath(); ctx.moveTo(x, y-len); ctx.lineTo(x, y-14); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x, y-4);
  ctx.lineTo(x-8, y-18);
  ctx.lineTo(x+8, y-18);
  ctx.closePath(); ctx.fill();
  ctx.font = 'bold 13px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('W = '+W+' oz', x, y - len - 8);
  ctx.restore();
}

/* ──────────────────────────────────────────────────────────────
   6.  RESULTS RENDERING
   ────────────────────────────────────────────────────────────── */

function renderSummary(res) {
  document.getElementById('m-wfail').textContent = res.Wfail < Infinity ? fmtF(res.Wfail) : '∞';
  document.getElementById('m-dw')   .textContent = res.Wfail < Infinity ? fmtF(res.deltaW)  : '—';
  document.getElementById('m-cost') .textContent = fmtC(res.cost);
  document.getElementById('m-ratio').textContent = fmtF(res.ratio, 4);
  document.getElementById('rxn-rpx').textContent = fmtF(res.RPx) + ' oz';
  document.getElementById('rxn-rpy').textContent = fmtF(res.RPy) + ' oz';
  document.getElementById('rxn-rry').textContent = fmtF(res.RRy) + ' oz';
}

function renderForcesTable(res) {
  const { truss, lengths, memberForces, buckle, critMember } = res;
  const tbody = document.getElementById('ftbody');
  let html = '';

  memberForces.forEach((Tm, m) => {
    const [j1,j2] = truss.members[m];
    const d  = buckle[m];
    const tc = d.isZero ? '<span class="zfm">ZFM</span>' :
               d.isCompression ? '<span class="compr">C</span>' :
               '<span class="tension">T</span>';
    const pcrit = d.isCompression ? fmtF(d.Pcrit) + ' oz' : '—';
    const wfail = d.isCompression ? fmtF(d.Wfail) + ' ± ' + fmtF(d.deltaW) : '—';
    const isCrit = m === critMember;
    const starLabel = isCrit ? ' ★' : '';

    html += `<tr class="${isCrit ? 'crit-row' : ''}">
      <td>m${m+1}${starLabel}</td>
      <td>J${j1+1}–J${j2+1}</td>
      <td>${fmtL(lengths[m])}</td>
      <td>${fmtF(Math.abs(Tm))}</td>
      <td>${tc}</td>
      <td>${pcrit}</td>
      <td>${wfail}</td>
    </tr>`;
  });

  tbody.innerHTML = html;
  scheduleTypeset(document.querySelector('.rxn-box'));
}

/* ──────────────────────────────────────────────────────────────
   7.  MATRIX DISPLAY
   ────────────────────────────────────────────────────────────── */

function renderMatrices(res) {
  const { truss, J, M, C, Sx, Sy, Lvec, A, T, memberForces, RPx, RPy, RRy } = res;
  const jLabels = Array.from({length:J}, (_,i) => 'J'+(i+1));
  const mLabels = Array.from({length:M}, (_,i) => 'm'+(i+1));
  const rxnLbls = ['R<sub>Px</sub>','R<sub>Py</sub>','R<sub>Ry</sub>'];

  let html = '';

  html += matSection('Connection Matrix C (J × M)',
    'C<sub>j,m</sub> = 1 if member m is connected to joint j, else 0.',
    renderMat(C, jLabels, mLabels));

  html += matSection('Support Matrix S<sub>x</sub> (J × 3)',
    'Column 1 = R<sub>Px</sub>. Entry = 1 at pin joint, 0 elsewhere.',
    renderMat(Sx, jLabels, ['R<sub>Px</sub>','·','·']));

  html += matSection('Support Matrix S<sub>y</sub> (J × 3)',
    'Column 2 = R<sub>Py</sub> (pin), Column 3 = R<sub>Ry</sub> (roller).',
    renderMat(Sy, jLabels, ['·','R<sub>Py</sub>','R<sub>Ry</sub>']));

  const LrowLbls = [
    ...Array.from({length:J}, (_,i) => 'L<sub>x,J'+(i+1)+'</sub>'),
    ...Array.from({length:J}, (_,i) => 'L<sub>y,J'+(i+1)+'</sub>')
  ];
  html += matSection('Load Vector L (2J × 1)',
    'L<sub>J+ℓ</sub> = −W at load joint ℓ, all other entries = 0.',
    renderMat(Lvec.map(v=>[v]), LrowLbls, ['value']));

  const aRowLbls = [
    ...Array.from({length:J}, (_,i) => 'xJ'+(i+1)),
    ...Array.from({length:J}, (_,i) => 'yJ'+(i+1))
  ];
  const aColLbls = [...mLabels, ...rxnLbls];
  html += matSection('Equilibrium Matrix A (2J × M+3)',
    '[A][T]+[L]=[0] → rows 1..J: ΣFx; rows J+1..2J: ΣFy. Last 3 cols = reaction unknowns.',
    renderMat(A, aRowLbls, aColLbls));

  const Tsub = [
    ...Array.from({length:M}, (_,i) => 'T<sub>'+(i+1)+'</sub>'),
    'R<sub>Px</sub>','R<sub>Py</sub>','R<sub>Ry</sub>'
  ];
  html += matSection('Solution Vector T = −A<sup>−1</sup>L (M+3 × 1)',
    'Member tensions (positive = T, negative = C) and reaction forces.',
    renderMat(T.map(v=>[v]), Tsub, ['value']));

  document.getElementById('matrix-display').innerHTML = html;
}

function matSection(title, desc, tableHtml) {
  return `<div class="mat-section">
    <h4>${title}</h4>
    <p>${desc}</p>
    <div class="mat-scroll">${tableHtml}</div>
  </div>`;
}

function renderMat(matrix, rowLbls, colLbls) {
  let rows = matrix;
  // Handle 1-D array (vector) passed as flat array
  if (!Array.isArray(rows[0])) rows = rows.map(v => [v]);

  let html = '<table class="matrix-tbl"><thead><tr>';
  if (rowLbls) html += '<th class="row-h"></th>';
  colLbls.forEach(c => { html += `<th>${c}</th>`; });
  html += '</tr></thead><tbody>';

  rows.forEach((row, i) => {
    html += '<tr>';
    if (rowLbls) html += `<th class="row-h">${rowLbls[i]}</th>`;
    row.forEach(v => {
      const n = Math.abs(v) < 1e-9 ? 0 : parseFloat(v.toFixed(4));
      const cls = n > 0 ? 'pos' : (n < 0 ? 'neg' : 'zero');
      html += `<td class="${cls}">${n}</td>`;
    });
    html += '</tr>';
  });
  html += '</tbody></table>';
  return html;
}

/* ──────────────────────────────────────────────────────────────
   8.  MATH STEPS (Step-by-step derivation, LaTeX via MathJax)
   ────────────────────────────────────────────────────────────── */

let stepCounter = 0;

function renderMathSteps(res) {
  const container = document.getElementById('math-steps');
  container.innerHTML = buildAllSteps(res);
  // Open first step by default
  const first = container.querySelector('.step-body');
  if (first) { first.classList.add('open'); container.querySelector('.step-chev').classList.add('open'); }
  scheduleTypeset(container);
}

function buildAllSteps(res) {
  stepCounter = 0;
  return [
    step1_validity(res),
    step2_joints(res),
    step3_lengths(res),
    step4_C(res),
    step5_Sxy(res),
    step6_L(res),
    step7_A(res),
    step8_solve(res),
    step9_forces(res),
    step10_buckling(res),
    step11_critical(res),
    step12_cost(res)
  ].join('');
}

function makeStep(title, badge, body) {
  stepCounter++;
  const id = 'step-' + stepCounter;
  return `
  <div class="math-step">
    <div class="step-header" onclick="toggleStep('${id}')">
      <span class="step-num">Step ${stepCounter}</span>
      <span class="step-title">${title}</span>
      ${badge ? `<span class="badge" style="background:${badge.bg};color:${badge.col}">${badge.text}</span>` : ''}
      <span class="step-chev" id="chev-${id}">▼</span>
    </div>
    <div class="step-body" id="${id}">${body}</div>
  </div>`;
}

/* ── Step 1: Validity ── */
function step1_validity(res) {
  const { J, M } = res;
  const expected = 2*J - 3;
  const ok = M === expected;
  return makeStep('Truss Validity Check &nbsp; — &nbsp; $M = 2J - 3$',
    ok ? {bg:'#d1fae5',col:'#065f46',text:'✓ Valid'} : {bg:'#fee2e2',col:'#991b1b',text:'✗ Invalid'}, `
  <p>A simple planar truss with $J$ joints and $M$ members must satisfy
     (EK301 Assignment Eq.&nbsp;1):</p>
  <div class="math-block">$$M = 2J - 3 \\tag{1}$$</div>
  <p>Substituting the values for this truss:</p>
  <div class="math-block">$$M = ${M}, \\quad J = ${J}$$</div>
  <div class="math-block">$$2J - 3 = 2(${J}) - 3 = ${expected}$$</div>
  ${ok
    ? `<div class="ok-box">✓ $M = ${M} = 2(${J})-3 = ${expected}$ — this is a valid simple truss.</div>`
    : `<div class="warn-box">✗ $M=${M} \\neq ${expected}$ — check the truss definition.</div>`}
`);
}

/* ── Step 2: Joint Locations ── */
function step2_joints(res) {
  const { truss, J } = res;
  const { joints, pinJoint, rollerJoint, loadJoint } = truss;
  const Xarr = joints.map(j => j[0]).join(', ');
  const Yarr = joints.map(j => j[1]).join(', ');
  const rows = joints.map((j,i) => {
    const notes = [];
    if (i===pinJoint)    notes.push('Pin support');
    if (i===rollerJoint) notes.push('Roller support');
    if (i===loadJoint)   notes.push('Load joint (W applied here)');
    return `<tr><td>J${i+1}</td><td>${j[0]}</td><td>${j[1]}</td><td>${notes.join(', ') || '—'}</td></tr>`;
  }).join('');

  return makeStep('Define Joint Locations — $\\mathbf{X}$, $\\mathbf{Y}$ vectors', null, `
  <p>We define the coordinates of all $J = ${J}$ joints
     using two vectors <strong>X</strong> and <strong>Y</strong>
     (Section 2.4.1 of the assignment):</p>
  <div class="math-block">$$\\mathbf{X} = [${Xarr}] \\text{ in}$$</div>
  <div class="math-block">$$\\mathbf{Y} = [${Yarr}] \\text{ in}$$</div>
  <table class="step-table">
    <thead><tr><th>Joint</th><th>x (in)</th><th>y (in)</th><th>Role</th></tr></thead>
    <tbody>${rows}</tbody>
  </table>
  <div class="info-box">The coordinate system has the pin support at the origin.
  The x-axis runs horizontally along the support line; the y-axis points upward.</div>
`);
}

/* ── Step 3: Member Lengths ── */
function step3_lengths(res) {
  const { truss, lengths, M } = res;
  const { joints, members } = truss;
  let eqs = '';
  members.forEach(([j1,j2], m) => {
    const dx = joints[j2][0]-joints[j1][0];
    const dy = joints[j2][1]-joints[j1][1];
    eqs += `<div class="math-block">
      $$\\ell_{${m+1}} = \\sqrt{(x_{J${j2+1}}-x_{J${j1+1}})^2+(y_{J${j2+1}}-y_{J${j1+1}})^2}
        = \\sqrt{(${dx})^2+(${dy})^2}
        = \\sqrt{${dx*dx+dy*dy}}
        \\approx ${fmtL(lengths[m])} \\text{ in}$$
    </div>`;
  });

  return makeStep('Member Lengths', null, `
  <p>For each member $m$ connecting joints $j_1$ and $j_2$:</p>
  <div class="math-block">
    $$\\ell_m = \\sqrt{(x_{j_2}-x_{j_1})^2 + (y_{j_2}-y_{j_1})^2}$$
  </div>
  ${eqs}
  <div class="info-box">Total member length: $L_{\\text{total}} = ${fmtL(res.Ltotal)}$ in</div>
`);
}

/* ── Step 4: Connection Matrix C ── */
function step4_C(res) {
  const { truss, J, M, C } = res;
  const jLabels = Array.from({length:J}, (_,i) => 'J'+(i+1));
  const mLabels = Array.from({length:M}, (_,i) => 'm'+(i+1));

  return makeStep('Connection Matrix $\\mathbf{C}$ ($J \\times M$)', null, `
  <p>The <em>connection matrix</em> $\\mathbf{C}$ has $J$ rows (joints) and $M$ columns (members).
     Entry $C_{j,m} = 1$ if member $m$ is connected to joint $j$, else $0$:</p>
  <div class="math-block">
    $$\\mathbf{C}_{j,m} = \\begin{cases} 1 & \\text{if member } m
      \\text{ is connected to joint } j \\\\ 0 & \\text{elsewhere}\\end{cases}$$
  </div>
  <p>Each column must sum to 2 (every member connects exactly 2 joints).
     Each row sum equals the number of members meeting at that joint.</p>
  <div class="mat-scroll">${renderMat(C, jLabels, mLabels)}</div>
`);
}

/* ── Step 5: Support Matrices ── */
function step5_Sxy(res) {
  const { truss, J, Sx, Sy } = res;
  const { pinJoint, rollerJoint } = truss;
  const jLabels = Array.from({length:J}, (_,i) => 'J'+(i+1));

  return makeStep('Support Connectivity Matrices $\\mathbf{S}_x$, $\\mathbf{S}_y$ ($J \\times 3$)', null, `
  <p>For a statically determinate truss supported by one pin and one roller we have
  three unknown reactions (Eqs. 3–8 of the assignment):</p>
  <div class="math-block">
    $$R_{Px} = \\text{pin x-reaction}, \\quad
      R_{Py} = \\text{pin y-reaction}, \\quad
      R_{Ry} = \\text{roller y-reaction}$$
  </div>
  <p><strong>$\\mathbf{S}_x$</strong> (size $J\\times3$): column 1 = $R_{Px}$, all others zero:</p>
  <div class="math-block">
    $$(\\mathbf{S}_x)_{j,1} = \\begin{cases}1 & j=P \\\\ 0 & \\text{else}\\end{cases}, \\quad
      (\\mathbf{S}_x)_{j,2} = (\\mathbf{S}_x)_{j,3} = 0$$
  </div>
  <div class="mat-scroll">${renderMat(Sx, jLabels, ['R_{Px}','·','·'])}</div>
  <br>
  <p><strong>$\\mathbf{S}_y$</strong> (size $J\\times3$): column 2 = $R_{Py}$ at pin, column 3 = $R_{Ry}$ at roller:</p>
  <div class="math-block">
    $$(\\mathbf{S}_y)_{j,2} = \\begin{cases}1 & j=P \\\\ 0 & \\text{else}\\end{cases}, \\quad
      (\\mathbf{S}_y)_{j,3} = \\begin{cases}1 & j=R \\\\ 0 & \\text{else}\\end{cases}$$
  </div>
  <div class="mat-scroll">${renderMat(Sy, jLabels, ['·','R_{Py}','R_{Ry}'])}</div>
  <div class="info-box">Pin at J${pinJoint+1}, Roller at J${rollerJoint+1}.</div>
`);
}

/* ── Step 6: Load Vector L ── */
function step6_L(res) {
  const { truss, J, Lvec } = res;
  const { loadJoint, W } = truss;
  const LrowLbls = [
    ...Array.from({length:J}, (_,i) => 'L_{x,J'+(i+1)+'}'),
    ...Array.from({length:J}, (_,i) => 'L_{y,J'+(i+1)+'}')
  ];
  const dispLbls = [
    ...Array.from({length:J}, (_,i) => 'x-force J'+(i+1)),
    ...Array.from({length:J}, (_,i) => 'y-force J'+(i+1))
  ];

  return makeStep('Load Vector $\\mathbf{L}$ ($2J \\times 1$) — Eq.&thinsp;(9)', null, `
  <p>$\\mathbf{L}$ has $2J = ${2*J}$ entries.
     The first $J$ entries are x-direction loads; the next $J$ are y-direction loads
     (Eq.&nbsp;9 of the assignment):</p>
  <div class="math-block">
    $$L_{J+\\ell} = -W \\quad (\\text{downward load at joint }\\ell = J${loadJoint+1})$$
  </div>
  <div class="math-block">
    $$L_{J+${loadJoint+1}} = -W = -${W} \\text{ oz}, \\quad \\text{all other entries} = 0$$
  </div>
  <div class="mat-scroll">${renderMat(Lvec.map(v=>[v]), dispLbls, ['value'])}</div>
  <div class="warn-box">Even though there is no horizontal external load, the $R_{Px}$ column
  must still appear in the matrix — otherwise $\\mathbf{A}$ is singular.</div>
`);
}

/* ── Step 7: Equilibrium Matrix A ── */
function step7_A(res) {
  const { truss, J, M, A, lengths } = res;
  const { joints, members, pinJoint, rollerJoint } = truss;

  // Show the formula for one sample member (member 0)
  const [j1s,j2s] = members[0];
  const dxs = joints[j2s][0]-joints[j1s][0];
  const dys = joints[j2s][1]-joints[j1s][1];
  const ls  = lengths[0];

  const aRowLbls = [
    ...Array.from({length:J}, (_,i) => 'xJ'+(i+1)),
    ...Array.from({length:J}, (_,i) => 'yJ'+(i+1))
  ];
  const mLabels = Array.from({length:M}, (_,i) => 'm'+(i+1));
  const aColLbls = [...mLabels, 'RPx','RPy','RRy'];

  return makeStep('Equilibrium Matrix $\\mathbf{A}$ ($2J \\times (M+3)$) — Eq.&thinsp;(10)', null, `
  <p>We assemble $\\mathbf{A}$ so that $[\\mathbf{A}][\\mathbf{T}]+[\\mathbf{L}]=[\\mathbf{0}]$
     (Eq. 10).  For member $m$ connecting joints $j_1$&ndash;$j_2$ with length $\\ell_m$,
     the <em>unit vector</em> from $j_1$ toward $j_2$ is:</p>
  <div class="math-block">
    $$\\hat{u}_{12} = \\left(\\frac{x_{j_2}-x_{j_1}}{\\ell_m},\\;
                               \\frac{y_{j_2}-y_{j_1}}{\\ell_m}\\right)$$
  </div>
  <p>The non-zero entries in $\\mathbf{A}$ for member $m$ are:</p>
  <div class="math-block">
    $$A_{j_1,\\,m} = \\frac{x_{j_2}-x_{j_1}}{\\ell_m}, \\quad
      A_{j_2,\\,m} = \\frac{x_{j_1}-x_{j_2}}{\\ell_m}$$
  </div>
  <div class="math-block">
    $$A_{J+j_1,\\,m} = \\frac{y_{j_2}-y_{j_1}}{\\ell_m}, \\quad
      A_{J+j_2,\\,m} = \\frac{y_{j_1}-y_{j_2}}{\\ell_m}$$
  </div>
  <p>For example, member $m_1$ connecting J${j1s+1}–J${j2s+1} ($\\ell_1 \\approx ${fmtL(ls)}$ in):</p>
  <div class="math-block">
    $$A_{${j1s+1},1} = \\frac{${dxs}}{${fmtL(ls)}} \\approx ${fmtL(dxs/ls)}, \\quad
      A_{${j2s+1},1} = \\frac{${-dxs}}{${fmtL(ls)}} \\approx ${fmtL(-dxs/ls)}$$
  </div>
  <p>Reaction force columns (last 3 cols): $A_{P,M+1}=1$ ($R_{Px}$ in x-eq at pin),
     $A_{J+P,M+2}=1$ ($R_{Py}$ in y-eq at pin), $A_{J+R,M+3}=1$ ($R_{Ry}$ in y-eq at roller).</p>
  <div class="mat-scroll">${renderMat(A, aRowLbls, aColLbls)}</div>
`);
}

/* ── Step 8: Solve T = −A⁻¹ L ── */
function step8_solve(res) {
  const { J, M, T } = res;

  let tlatex = '\\begin{pmatrix}';
  T.forEach((v,i) => {
    const label = i < M ? `T_{${i+1}}` : (i===M?'R_{Px}':(i===M+1?'R_{Py}':'R_{Ry}'));
    tlatex += `${label} \\\\ `;
  });
  tlatex += '\\end{pmatrix}';

  return makeStep('Solve the System — $\\mathbf{T} = -\\mathbf{A}^{-1}\\mathbf{L}$ — Eq.&thinsp;(11)', null, `
  <p>Rearranging $[\\mathbf{A}][\\mathbf{T}]+[\\mathbf{L}]=[\\mathbf{0}]$:</p>
  <div class="math-block">$$\\mathbf{T} = -\\mathbf{A}^{-1}\\,\\mathbf{L} \\tag{11}$$</div>
  <p>The system has $2J = ${2*J}$ equations in $M+3 = ${M+3}$ unknowns
     ($M = ${M}$ member tensions + 3 reaction forces).
     We solve via Gauss–Jordan elimination with partial pivoting.</p>
  <h5>Solution vector $\\mathbf{T}$:</h5>
  <table class="step-table">
    <thead><tr><th>Unknown</th><th>Value (oz)</th><th>Sign convention</th></tr></thead>
    <tbody>
      ${T.slice(0,M).map((v,i) =>
        `<tr><td>$T_{${i+1}}$ (m${i+1})</td>
             <td>${fmtF(v)}</td>
             <td>${Math.abs(v)<1e-9 ? 'Zero-force' : (v>0 ? 'Tension (T)' : 'Compression (C)')}</td></tr>`
      ).join('')}
      <tr><td>$R_{Px}$</td><td>${fmtF(T[M])}</td><td>Pin x-reaction</td></tr>
      <tr><td>$R_{Py}$</td><td>${fmtF(T[M+1])}</td><td>Pin y-reaction</td></tr>
      <tr><td>$R_{Ry}$</td><td>${fmtF(T[M+2])}</td><td>Roller y-reaction</td></tr>
    </tbody>
  </table>
  <div class="info-box">Positive $T_m$ → member in <strong>tension</strong>.
  Negative $T_m$ → member in <strong>compression</strong>.</div>
`);
}

/* ── Step 9: Forces summary ── */
function step9_forces(res) {
  const { truss, memberForces, buckle } = res;
  const rows = memberForces.map((Tm,m) => {
    const [j1,j2] = truss.members[m];
    const d = buckle[m];
    const tc = d.isZero ? 'ZFM' : (d.isCompression ? 'C' : 'T');
    return `<tr>
      <td>$m_{${m+1}}$ (J${j1+1}–J${j2+1})</td>
      <td>$T_{${m+1}} = ${fmtF(Tm)}$ oz</td>
      <td>${tc}</td>
    </tr>`;
  }).join('');

  return makeStep('Member Forces — Tension / Compression Identification', null, `
  <p>A member is in <strong>tension</strong> if its signed tension $T_m > 0$
     (the member is being pulled apart).
     It is in <strong>compression</strong> if $T_m < 0$
     (the member is being squeezed). $|T_m| < 10^{-9}$ oz → zero-force member (ZFM).</p>
  <table class="step-table">
    <thead><tr><th>Member</th><th>Internal Tension $T_m$</th><th>T / C / ZFM</th></tr></thead>
    <tbody>${rows}</tbody>
  </table>
  <div class="info-box">
    Equilibrium check — reactions must satisfy global statics:<br>
    $\\sum F_y = R_{Py} + R_{Ry} = ${fmtF(res.RPy)} + ${fmtF(res.RRy)} = ${fmtF(res.RPy+res.RRy)}$ oz
    $= W = ${truss.W}$ oz ✓
  </div>
`);
}

/* ── Step 10: Buckling Analysis ── */
function step10_buckling(res) {
  const { truss, lengths, memberForces, buckle, J, M } = res;
  const C_FIT = 37.5, L0 = 10, ALPHA = 2, U_FIT = 10;

  let compRows = '';
  buckle.forEach((d,m) => {
    if (!d.isCompression) return;
    const [j1,j2] = truss.members[m];
    compRows += `<tr>
      <td>$m_{${m+1}}$ (J${j1+1}–J${j2+1})</td>
      <td>${fmtL(lengths[m])}</td>
      <td>${fmtF(memberForces[m])} oz</td>
      <td>${fmtF(d.Pcrit)} oz</td>
    </tr>`;
  });

  return makeStep('Buckling Force — Eq.&thinsp;(12): $F = C_{\\text{fit}}(L_0/L)^{\\alpha} \\pm U_{\\text{fit}}$', null, `
  <p>For the Spring 2026 semester, the length-dependent buckling force is (Eq. 12):</p>
  <div class="math-block">
    $$F = C_{\\text{fit}}\\left(\\frac{L_0}{L}\\right)^{\\alpha} \\pm U_{\\text{fit}} \\tag{12}$$
  </div>
  <p>where:</p>
  <ul>
    <li>$C_{\\text{fit}} = ${C_FIT}$ oz (fit coefficient)</li>
    <li>$L_0 = ${L0}$ in (reference length)</li>
    <li>$\\alpha = ${ALPHA}$ (power law exponent)</li>
    <li>$U_{\\text{fit}} = ${U_FIT}$ oz (total fit uncertainty)</li>
  </ul>
  <p>Buckling is only relevant for compression members ($T_m < 0$).
     Only compression members are analyzed below:</p>
  <table class="step-table">
    <thead>
      <tr><th>Member</th><th>Length (in)</th><th>Force (oz)</th><th>$P_{\\text{crit}}$ (oz)</th></tr>
    </thead>
    <tbody>${compRows || '<tr><td colspan="4">No compression members.</td></tr>'}</tbody>
  </table>
`);
}

/* ── Step 11: Critical member & Wfail ── */
function step11_critical(res) {
  const { truss, lengths, memberForces, buckle, critMember, Wfail, deltaW } = res;
  const W = truss.W;

  let failRows = '';
  buckle.forEach((d,m) => {
    if (!d.isCompression) return;
    const [j1,j2] = truss.members[m];
    failRows += `<tr ${m===critMember ? 'style="background:#fffbeb;font-weight:700"' : ''}>
      <td>$m_{${m+1}}$ ${m===critMember ? '★' : ''}</td>
      <td>${fmtL(lengths[m])}</td>
      <td>${fmtF(memberForces[m])} oz</td>
      <td>${fmtF(d.Rm)}</td>
      <td>${fmtF(d.Wfail)} oz</td>
    </tr>`;
  });

  const cm = critMember >= 0 ? critMember : null;
  const cmLabel = cm !== null ? `m_{${cm+1}}` : '—';

  return makeStep('Critical Member and Maximum Load — Eq.&thinsp;(13),(16)', null, `
  <p>By linearity (Eq. 13), the tension in member $m$ is proportional to the applied load $W_\\ell$:</p>
  <div class="math-block">$$T_m = R_m \\times W_\\ell \\quad \\Rightarrow \\quad R_m = \\frac{T_m}{W_\\ell} \\tag{13}$$</div>
  <p>Failure occurs when a compression member reaches its buckling strength.
     The failure load for compression member $m$ is (Eq. 16):</p>
  <div class="math-block">
    $$W_{\\text{fail},m} = \\frac{-P_{\\text{crit},m}}{R_m} = \\frac{P_{\\text{crit},m}}{|T_m/W|} \\tag{16}$$
  </div>
  <p>Applying this to each compression member at $W = ${W}$ oz:</p>
  <table class="step-table">
    <thead>
      <tr><th>Member</th><th>L (in)</th><th>$T_m$ (oz)</th>
          <th>$R_m = T_m/W$</th><th>$W_{\\text{fail}}$ (oz)</th></tr>
    </thead>
    <tbody>${failRows || '<tr><td colspan="5">No compression members.</td></tr>'}</tbody>
  </table>
  ${cm !== null ? `
  <div class="ok-box">
    <strong>Critical member: $${cmLabel}$</strong>
    &nbsp;(smallest $W_{\\text{fail}}$)<br>
    $W_{\\text{fail}} = ${fmtF(Wfail)} \\pm ${fmtF(deltaW)}$ oz
    &nbsp; (uncertainty propagated from $U_{\\text{fit}}$)
  </div>
  ` : '<div class="info-box">No compression members — truss does not buckle under this load.</div>'}
`);
}

/* ── Step 12: Cost ── */
function step12_cost(res) {
  const { truss, J, M, lengths, Ltotal, cost, ratio, Wfail } = res;

  let termStr = lengths.map((l,m) => `${fmtL(l)}`).join(' + ');

  return makeStep('Truss Cost and Load-to-Cost Ratio — Eq.&thinsp;(2)', null, `
  <p>The hypothetical cost of the truss is (Eq. 2 of the assignment):</p>
  <div class="math-block">
    $$C = C_L L_{\\text{total}} + C_J J \\tag{2}$$
  </div>
  <p>where $C_L = 1$ unit/in and $C_J = 10$ units/joint.</p>
  <h5>Total member length:</h5>
  <div class="math-block">
    $$L_{\\text{total}} = ${termStr} = ${fmtL(Ltotal)} \\text{ in}$$
  </div>
  <h5>Cost:</h5>
  <div class="math-block">
    $$C = 1 \\times ${fmtL(Ltotal)} + 10 \\times ${J} = ${fmtL(Ltotal)} + ${10*J}
        = ${fmtC(cost)} \\text{ units}$$
  </div>
  <h5>Load-to-cost ratio:</h5>
  <div class="math-block">
    $$\\frac{W_{\\text{fail}}}{C} = \\frac{${fmtF(Wfail < Infinity ? Wfail : 0)}}{${fmtC(cost)}}
        = ${fmtF(ratio, 4)} \\text{ oz/unit}$$
  </div>
  <div class="ok-box">
    <strong>Final Metrics</strong><br>
    $J = ${J}$ joints &nbsp;|&nbsp; $M = ${M}$ members &nbsp;|&nbsp;
    $L_{\\text{total}} = ${fmtL(Ltotal)}$ in &nbsp;|&nbsp;
    $C = ${fmtC(cost)}$ units &nbsp;|&nbsp;
    $W_{\\text{fail}} = ${fmtF(Wfail < Infinity ? Wfail : 0)}$ oz &nbsp;|&nbsp;
    Load/Cost $= ${fmtF(ratio, 4)}$ oz/unit
  </div>
`);
}

/* ──────────────────────────────────────────────────────────────
   9.  UI ACTIONS
   ────────────────────────────────────────────────────────────── */

function loadPreset(idx) {
  currentTrussIdx = idx;
  currentResults  = solveTruss(PRESETS[idx]);
  hoveredMember   = -1;

  document.querySelectorAll('.preset-btn').forEach((b,i) => b.classList.toggle('active', i===idx));
  document.getElementById('an-title').textContent = PRESETS[idx].name;
  document.getElementById('an-sub').textContent   = PRESETS[idx].subtitle;

  renderSummary(currentResults);
  renderForcesTable(currentResults);
  renderMathSteps(currentResults);
  drawTruss(currentResults, hoveredMember);

  if (viewMode === 'matrix') renderMatrices(currentResults);
}

function setView(mode) {
  viewMode = mode;
  document.getElementById('vb-visual').classList.toggle('active', mode==='visual');
  document.getElementById('vb-matrix').classList.toggle('active', mode==='matrix');
  document.getElementById('visual-panel').style.display = mode==='visual' ? '' : 'none';
  document.getElementById('matrix-panel').style.display = mode==='matrix' ? '' : 'none';
  if (mode==='matrix' && currentResults) renderMatrices(currentResults);
}

function toggleStep(id) {
  const body = document.getElementById(id);
  const chev = document.getElementById('chev-'+id);
  body.classList.toggle('open');
  if (chev) chev.classList.toggle('open');
}

function switchCTab(tab) {
  document.getElementById('ctab-form').classList.toggle('active', tab==='form');
  document.getElementById('ctab-mat') .classList.toggle('active', tab==='mat');
  document.getElementById('cpanel-form').style.display = tab==='form' ? '' : 'none';
  document.getElementById('cpanel-mat') .style.display = tab==='mat'  ? '' : 'none';
}

/* ──────────────────────────────────────────────────────────────
   10.  CUSTOM TRUSS — INTERACTIVE FORM
   ────────────────────────────────────────────────────────────── */

function initCustomForm() {
  const J = parseInt(document.getElementById('cJ').value) || 5;
  const M = parseInt(document.getElementById('cM').value) || 7;
  let html = '';

  html += '<div class="cform-section"><h4>Joint Coordinates</h4><div class="cform-grid">';
  for (let j = 1; j <= J; j++) {
    html += `<div class="fg">
      <label>J${j}: x (in)</label>
      <input type="number" id="jx${j}" value="${(j-1)*28/(J-1||1)}" step="0.1">
    </div>
    <div class="fg">
      <label>J${j}: y (in)</label>
      <input type="number" id="jy${j}" value="0" step="0.1">
    </div>`;
  }
  html += '</div></div>';

  html += '<div class="cform-section"><h4>Member Connections</h4><div class="cform-grid">';
  for (let m = 1; m <= M; m++) {
    html += `<div class="fg">
      <label>m${m} from joint #:</label>
      <input type="number" id="mj1_${m}" value="1" min="1" max="${J}">
    </div>
    <div class="fg">
      <label>m${m} to joint #:</label>
      <input type="number" id="mj2_${m}" value="${Math.min(m+1,J)}" min="1" max="${J}">
    </div>`;
  }
  html += '</div></div>';

  html += `<div class="cform-section" id="support-load-section">
    <h4>Supports and Load</h4>
    <div class="cform-grid">
      <div class="fg"><label>Pin support at joint #</label><input type="number" id="cpinj" value="1" min="1" max="${J}"></div>
      <div class="fg"><label>Roller support at joint #</label><input type="number" id="crolj" value="${J}" min="1" max="${J}"></div>
      <div class="fg"><label>Load joint #</label><input type="number" id="cloadj" value="2" min="1" max="${J}"></div>
      <div class="fg"><label>Load W (oz)</label><input type="number" id="cW" value="32" step="0.5"></div>
    </div>
    <br>
    <button class="btn btn-primary" onclick="solveCustomInteractive()">Analyze Truss</button>
    <div id="cform-err" class="err-msg" style="display:none;margin-top:.5rem"></div>
  </div>`;

  document.getElementById('cform-body').innerHTML = html;
}

function solveCustomInteractive() {
  const J = parseInt(document.getElementById('cJ').value) || 5;
  const M = parseInt(document.getElementById('cM').value) || 7;
  const errDiv = document.getElementById('cform-err');
  errDiv.style.display = 'none';

  try {
    const joints  = [];
    for (let j = 1; j <= J; j++) {
      const x = parseFloat(document.getElementById('jx'+j).value);
      const y = parseFloat(document.getElementById('jy'+j).value);
      if (isNaN(x)||isNaN(y)) throw new Error(`Joint J${j}: invalid coordinates`);
      joints.push([x,y]);
    }

    const members = [];
    for (let m = 1; m <= M; m++) {
      const j1 = parseInt(document.getElementById('mj1_'+m).value) - 1;
      const j2 = parseInt(document.getElementById('mj2_'+m).value) - 1;
      if (j1<0||j2<0||j1>=J||j2>=J) throw new Error(`Member m${m}: joint index out of range`);
      if (j1===j2) throw new Error(`Member m${m}: both endpoints are the same joint`);
      members.push([j1,j2]);
    }

    const pinJoint    = parseInt(document.getElementById('cpinj').value)  - 1;
    const rollerJoint = parseInt(document.getElementById('crolj').value)  - 1;
    const loadJoint   = parseInt(document.getElementById('cloadj').value) - 1;
    const W           = parseFloat(document.getElementById('cW').value);

    const customTruss = { name:'Custom Truss', subtitle:'User-defined', joints, members, pinJoint, rollerJoint, loadJoint, W };
    const res = solveTruss(customTruss);
    currentResults = res;

    document.getElementById('an-title').textContent = 'Custom Truss';
    document.getElementById('an-sub').textContent   = `${J} Joints · ${M} Members`;

    renderSummary(res);
    renderForcesTable(res);
    renderMathSteps(res);
    drawTruss(res, -1);
    if (viewMode==='matrix') renderMatrices(res);
    setView('visual');
  } catch(e) {
    errDiv.textContent = '⚠ Error: ' + e.message;
    errDiv.style.display = 'block';
  }
}

/* ──────────────────────────────────────────────────────────────
   11.  CUSTOM TRUSS — MATRIX INPUT
   ────────────────────────────────────────────────────────────── */

function solveFromMatrix() {
  const errDiv = document.getElementById('mi-error');
  errDiv.style.display = 'none';
  try {
    const X = document.getElementById('mi-X').value.trim().split(/[\s,]+/).map(Number);
    const Y = document.getElementById('mi-Y').value.trim().split(/[\s,]+/).map(Number);
    if (X.length !== Y.length) throw new Error('X and Y must have the same length (one per joint)');
    const J = X.length;
    const joints = X.map((x,i) => [x, Y[i]]);

    const rawC = document.getElementById('mi-C').value.trim();
    const Crows = rawC.split(/\n|;/).map(r => r.trim().split(/\s+/).map(Number));
    if (Crows.length !== J) throw new Error(`C must have ${J} rows (one per joint)`);
    const M = Crows[0].length;
    if (Crows.some(r => r.length !== M)) throw new Error('All rows of C must have the same length');

    // Reconstruct members from C (each column has exactly 2 ones)
    const members = [];
    for (let m = 0; m < M; m++) {
      const jts = [];
      for (let j = 0; j < J; j++) if (Crows[j][m] === 1) jts.push(j);
      if (jts.length !== 2) throw new Error(`C column ${m+1} must have exactly two 1s`);
      members.push([jts[0], jts[1]]);
    }

    const pinJoint    = parseInt(document.getElementById('mi-pin').value)    - 1;
    const rollerJoint = parseInt(document.getElementById('mi-roller').value) - 1;
    const loadJoint   = parseInt(document.getElementById('mi-lj').value)     - 1;
    const W           = parseFloat(document.getElementById('mi-W').value);

    // Validate to ensure they aren't out of bounds before building the matrix
    if (pinJoint < 0 || rollerJoint < 0 || loadJoint < 0) {
        throw new Error("Joint indices must be 1 or greater.");
    }

    const t = { name:'Matrix Input Truss', subtitle:'From matrix definition', joints, members, pinJoint, rollerJoint, loadJoint, W };
    const res = solveTruss(t);
    currentResults = res;

    document.getElementById('an-title').textContent = 'Matrix Input Truss';
    document.getElementById('an-sub').textContent   = `${J} Joints · ${M} Members`;

    renderSummary(res);
    renderForcesTable(res);
    renderMathSteps(res);
    drawTruss(res, -1);
    if (viewMode==='matrix') renderMatrices(res);
    setView('visual');
  } catch(e) {
    errDiv.textContent = '⚠ Error: ' + e.message;
    errDiv.style.display = 'block';
  }
}

/* ──────────────────────────────────────────────────────────────
   12.  CANVAS HOVER INTERACTION
   ────────────────────────────────────────────────────────────── */

function initCanvasEvents() {
  const canvas = document.getElementById('trussCanvas');

  canvas.addEventListener('mousemove', e => {
    if (!currentResults || !canvasTransform) return;
    const rect = canvas.getBoundingClientRect();
    const mx = (e.clientX - rect.left) * canvas.width  / rect.width;
    const my = (e.clientY - rect.top)  * canvas.height / rect.height;

    const { truss, memberForces, buckle } = currentResults;
    const { tx, ty } = canvasTransform;

    let best = -1, bestDist = 11;
    truss.members.forEach(([j1,j2], m) => {
      const x1 = tx(truss.joints[j1][0]), y1 = ty(truss.joints[j1][1]);
      const x2 = tx(truss.joints[j2][0]), y2 = ty(truss.joints[j2][1]);
      const d  = distToSeg(mx, my, x1, y1, x2, y2);
      if (d < bestDist) { bestDist = d; best = m; }
    });

    if (best !== hoveredMember) {
      hoveredMember = best;
      drawTruss(currentResults, hoveredMember);

      const tip = document.getElementById('hover-tip');
      if (best >= 0) {
        const Tm = memberForces[best];
        const d  = buckle[best];
        const [j1,j2] = truss.members[best];
        const tc = d.isZero ? 'ZFM' : (d.isCompression ? 'Compression' : 'Tension');
        let txt = `m${best+1} (J${j1+1}–J${j2+1}) · L = ${fmtL(currentResults.lengths[best])} in · |T| = ${fmtF(Math.abs(Tm))} oz · ${tc}`;
        if (d.isCompression) txt += ` · Wfail = ${fmtF(d.Wfail)} oz`;
        if (best === currentResults.critMember) txt += ' · ★ CRITICAL';
        tip.textContent = txt;
        tip.style.display = 'inline-block';
      } else {
        tip.style.display = 'none';
      }
    }
  });

  canvas.addEventListener('mouseleave', () => {
    hoveredMember = -1;
    if (currentResults) drawTruss(currentResults, -1);
    document.getElementById('hover-tip').style.display = 'none';
  });
}

/* ──────────────────────────────────────────────────────────────
   13.  MATHJAX TYPESETTING
   ────────────────────────────────────────────────────────────── */

let typesetPending = false;
function scheduleTypeset(el) {
  if (!window.MathJax) return;
  if (typesetPending) return;
  typesetPending = true;
  requestAnimationFrame(() => {
    MathJax.typesetPromise(el ? [el] : undefined)
      .catch(console.error)
      .finally(() => { typesetPending = false; });
  });
}

/* ──────────────────────────────────────────────────────────────
   14.  INIT
   ────────────────────────────────────────────────────────────── */

document.addEventListener('DOMContentLoaded', () => {
  initCanvasEvents();
  initCustomForm();   // pre-populate the form
  loadPreset(0);      // load the small pratt truss by default
});
