/* ── Step 1: Validity & Addendum Constraints ── */
function step1_validity(res) {
  const { J, M, truss, lengths } = res;
  const { joints, pinJoint, rollerJoint } = truss;
  const isStaticallyDeterminate = (M === 2*J - 3);
  
  // -- NEW: 2026 Addendum Constraint Checks --
  let constraintsHtml = `<h5>Addendum Constraints Check:</h5><ul>`;
  let allValid = true;
  
  // 1. Joint-to-joint span
  lengths.forEach((L, m) => {
    if (L < 6 || L > 14) {
      constraintsHtml += `<li style="color:#dc2626">Member m${m+1} length ($L = ${fmtL(L)}$ in) is out of bounds (6-14 in).</li>`;
      allValid = false;
    }
  });

  const px = joints[pinJoint][0];
  const py = joints[pinJoint][1];
  const rx = joints[rollerJoint][0];
  const ry = joints[rollerJoint][1];
  
  // 2. Truss span
  const trussSpan = Math.abs(rx - px);
  if (trussSpan < 26 || trussSpan > 30) {
    constraintsHtml += `<li style="color:#dc2626">Truss span ($${fmtL(trussSpan)}$ in) is out of bounds (26-30 in).</li>`;
    allValid = false;
  }

  // 3. Vertical Offset (Delta y)
  const deltaY = Math.abs(ry - py);
  if (deltaY < 0 || deltaY > 2) {
    constraintsHtml += `<li style="color:#dc2626">Vertical offset $\\Delta y$ ($${fmtL(deltaY)}$ in) is out of bounds (0-2 in).</li>`;
    allValid = false;
  }
  
  if (allValid) {
    constraintsHtml += `<li style="color:#059669">All member lengths, truss span, and vertical offset constraints are satisfied!</li>`;
  }
  constraintsHtml += `</ul>`;

  let html = `<p>Eq. (1) checks for static determinacy: $M = 2J - 3$</p>
  <div class="math-block">
    $$M = ${M}$$
    $$2J - 3 = 2(${J}) - 3 = ${2*J - 3}$$
  </div>
  ${isStaticallyDeterminate 
    ? `<div class="ok-box" style="margin-bottom:1rem"><strong>Valid:</strong> $${M} = ${2*J - 3}$. The truss is statically determinate.</div>` 
    : `<div class="err-box" style="margin-bottom:1rem"><strong>Invalid:</strong> $${M} \\neq ${2*J - 3}$. The truss is NOT statically determinate. Analysis may be invalid.</div>`
  }
  ${constraintsHtml}`;

  return makeStep('Static Determinacy & Constraints', 
    isStaticallyDeterminate && allValid ? {text:'Valid', bg:'#10b981', col:'#fff'} : {text:'Invalid', bg:'#ef4444', col:'#fff'}, 
    html);
}
