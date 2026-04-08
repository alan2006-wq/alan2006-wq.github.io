// ========================================
// EK301 Truss Design Project - JavaScript
// ========================================

// Global State
let currentProject = null;
let currentView = 'overview';
let analysisMode = 'matrix'; // 'matrix' or 'joints'

// ========================================
// Built-in Project Data
// ========================================

const PROJECTS = {
    easy: {
        name: "Easy Verification: Triangle Truss",
        description: "3-joint simple triangle truss for basic verification",
        type: "verification",
        level: "easy",
        joints: [
            { id: 1, x: 0, y: 0, label: "A" },
            { id: 2, x: 10, y: 0, label: "B" },
            { id: 3, x: 5, y: 6, label: "C" }
        ],
        members: [
            { id: 1, jointA: 1, jointB: 2, label: "m1" },
            { id: 2, jointA: 2, jointB: 3, label: "m2" },
            { id: 3, jointA: 3, jointB: 1, label: "m3" }
        ],
        supports: {
            pinJoint: 1,
            rollerJoint: 2
        },
        loads: [
            { jointId: 3, fx: 0, fy: -30, label: "W" }
        ],
        appliedLoad: 30
    },
    
    medium: {
        name: "Medium Verification: 5-Joint Truss",
        description: "Simple truss with 5 joints for comprehensive verification",
        type: "verification",
        level: "medium",
        joints: [
            { id: 1, x: 0, y: 0, label: "A" },
            { id: 2, x: 10, y: 2, label: "B" },
            { id: 3, x: 5, y: 6, label: "C" },
            { id: 4, x: 15, y: 6, label: "D" },
            { id: 5, x: 20, y: 0, label: "E" }
        ],
        members: [
            { id: 1, jointA: 1, jointB: 2, label: "m1" },
            { id: 2, jointA: 1, jointB: 3, label: "m2" },
            { id: 3, jointA: 2, jointB: 3, label: "m3" },
            { id: 4, jointA: 2, jointB: 4, label: "m4" },
            { id: 5, jointA: 3, jointB: 4, label: "m5" },
            { id: 6, jointA: 4, jointB: 5, label: "m6" },
            { id: 7, jointA: 2, jointB: 5, label: "m7" }
        ],
        supports: {
            pinJoint: 1,
            rollerJoint: 5
        },
        loads: [
            { jointId: 2, fx: 0, fy: -30, label: "W" }
        ],
        appliedLoad: 30
    },
    
    challenge: {
        name: "Challenge Verification: 10-Joint Truss",
        description: "Complex truss with 10 joints for advanced verification",
        type: "verification",
        level: "challenge",
        joints: [
            { id: 1, x: 0, y: 0, label: "A" },
            { id: 2, x: 10, y: 2, label: "B" },
            { id: 3, x: 5, y: 6, label: "C" },
            { id: 4, x: 15, y: 6, label: "D" },
            { id: 5, x: 20, y: 2, label: "E" },
            { id: 6, x: 30, y: 0, label: "F" },
            { id: 7, x: 10, y: 8, label: "G" },
            { id: 8, x: 20, y: 8, label: "H" },
            { id: 9, x: 25, y: 4, label: "I" },
            { id: 10, x: 15, y: 0, label: "J" }
        ],
        members: [
            { id: 1, jointA: 1, jointB: 2, label: "m1" },
            { id: 2, jointA: 1, jointB: 3, label: "m2" },
            { id: 3, jointA: 2, jointB: 3, label: "m3" },
            { id: 4, jointA: 2, jointB: 4, label: "m4" },
            { id: 5, jointA: 2, jointB: 7, label: "m5" },
            { id: 6, jointA: 3, jointB: 4, label: "m6" },
            { id: 7, jointA: 3, jointB: 7, label: "m7" },
            { id: 8, jointA: 4, jointB: 7, label: "m8" },
            { id: 9, jointA: 4, jointB: 8, label: "m9" },
            { id: 10, jointA: 4, jointB: 5, label: "m10" },
            { id: 11, jointA: 5, jointB: 8, label: "m11" },
            { id: 12, jointA: 5, jointB: 9, label: "m12" },
            { id: 13, jointA: 5, jointB: 10, label: "m13" },
            { id: 14, jointA: 6, jointB: 9, label: "m14" },
            { id: 15, jointA: 8, jointB: 9, label: "m15" },
            { id: 16, jointA: 9, jointB: 10, label: "m16" },
            { id: 17, jointA: 10, jointB: 6, label: "m17" }
        ],
        supports: {
            pinJoint: 1,
            rollerJoint: 6
        },
        loads: [
            { jointId: 2, fx: 0, fy: -30, label: "W" }
        ],
        appliedLoad: 30
    },
    
    design1: {
        name: "Preliminary Design 1",
        description: "First candidate truss design - customizable",
        type: "design",
        joints: [
            { id: 1, x: 0, y: 0, label: "A (Pin)" },
            { id: 2, x: 10, y: 2, label: "B (Load)" },
            { id: 3, x: 6, y: 7, label: "C" },
            { id: 4, x: 14, y: 7, label: "D" },
            { id: 5, x: 20, y: 2, label: "E" },
            { id: 6, x: 30, y: 0, label: "F (Roller)" },
            { id: 7, x: 10, y: 10, label: "G" },
            { id: 8, x: 20, y: 10, label: "H" }
        ],
        members: [
            { id: 1, jointA: 1, jointB: 2, label: "m1" },
            { id: 2, jointA: 1, jointB: 3, label: "m2" },
            { id: 3, jointA: 2, jointB: 3, label: "m3" },
            { id: 4, jointA: 2, jointB: 4, label: "m4" },
            { id: 5, jointA: 2, jointB: 7, label: "m5" },
            { id: 6, jointA: 3, jointB: 7, label: "m6" },
            { id: 7, jointA: 3, jointB: 4, label: "m7" },
            { id: 8, jointA: 4, jointB: 7, label: "m8" },
            { id: 9, jointA: 4, jointB: 8, label: "m9" },
            { id: 10, jointA: 4, jointB: 5, label: "m10" },
            { id: 11, jointA: 5, jointB: 8, label: "m11" },
            { id: 12, jointA: 5, jointB: 6, label: "m12" },
            { id: 13, jointA: 7, jointB: 8, label: "m13" }
        ],
        supports: {
            pinJoint: 1,
            rollerJoint: 6
        },
        loads: [
            { jointId: 2, fx: 0, fy: -32, label: "W" }
        ],
        appliedLoad: 32
    },
    
    design2: {
        name: "Preliminary Design 2",
        description: "Second candidate truss design - customizable",
        type: "design",
        joints: [
            { id: 1, x: 0, y: 0, label: "A (Pin)" },
            { id: 2, x: 10, y: 2, label: "B (Load)" },
            { id: 3, x: 8, y: 8, label: "C" },
            { id: 4, x: 15, y: 10, label: "D" },
            { id: 5, x: 22, y: 8, label: "E" },
            { id: 6, x: 20, y: 2, label: "F" },
            { id: 7, x: 30, y: 0, label: "G (Roller)" }
        ],
        members: [
            { id: 1, jointA: 1, jointB: 2, label: "m1" },
            { id: 2, jointA: 1, jointB: 3, label: "m2" },
            { id: 3, jointA: 2, jointB: 3, label: "m3" },
            { id: 4, jointA: 2, jointB: 4, label: "m4" },
            { id: 5, jointA: 3, jointB: 4, label: "m5" },
            { id: 6, jointA: 4, jointB: 5, label: "m6" },
            { id: 7, jointA: 4, jointB: 6, label: "m7" },
            { id: 8, jointA: 5, jointB: 6, label: "m8" },
            { id: 9, jointA: 6, jointB: 7, label: "m9" },
            { id: 10, jointA: 5, jointB: 7, label: "m10" },
            { id: 11, jointA: 2, jointB: 6, label: "m11" }
        ],
        supports: {
            pinJoint: 1,
            rollerJoint: 7
        },
        loads: [
            { jointId: 2, fx: 0, fy: -32, label: "W" }
        ],
        appliedLoad: 32
    }
};

// Buckling parameters (from Spring 2026 specification)
const BUCKLING_PARAMS = {
    C: 37.5,      // oz
    L0: 10,       // inches
    alpha: 2,
    Ufit: 10      // oz
};

// Cost parameters
const COST_PARAMS = {
    CL: 1,        // units/inch
    CJ: 10        // units/joint
};

// ========================================
// Initialization
// ========================================

document.addEventListener('DOMContentLoaded', () => {
    initializeNavigation();
    initializeModeSwitching();
    initializeTabs();
    showView('overview');
});

// ========================================
// Navigation
// ========================================

function initializeNavigation() {
    // Project navigation
    document.querySelectorAll('.nav-item[data-project]').forEach(item => {
        item.addEventListener('click', (e) => {
            const projectId = e.target.dataset.project;
            loadProject(projectId);
            updateActiveNav(e.target);
        });
    });
    
    // View navigation
    document.querySelectorAll('.nav-item[data-view]').forEach(item => {
        item.addEventListener('click', (e) => {
            const viewName = e.target.dataset.view;
            showView(viewName);
            updateActiveNav(e.target);
        });
    });
}

function updateActiveNav(activeItem) {
    document.querySelectorAll('.nav-item').forEach(item => {
        item.classList.remove('active');
    });
    activeItem.classList.add('active');
}

function showView(viewName) {
    currentView = viewName;
    
    // Hide all views
    document.querySelectorAll('.content-view').forEach(view => {
        view.classList.remove('active');
    });
    
    // Show the requested view
    const viewMap = {
        'overview': 'overviewView',
        'method': 'methodView',
        'formulas': 'formulasView',
        'project': 'projectView'
    };
    
    const viewId = viewMap[viewName];
    if (viewId) {
        document.getElementById(viewId).classList.add('active');
    }
    
    // Update page title
    const titleMap = {
        'overview': 'Project Overview',
        'method': 'Analysis Method',
        'formulas': 'Key Formulas'
    };
    
    document.getElementById('pageTitle').textContent = titleMap[viewName] || 'Truss Analysis';
    
    // Trigger MathJax rendering for math content
    if (typeof MathJax !== 'undefined') {
        MathJax.typesetPromise();
    }
}

// ========================================
// Mode Switching
// ========================================

function initializeModeSwitching() {
    document.getElementById('matrixModeBtn').addEventListener('click', () => {
        analysisMode = 'matrix';
        updateModeButtons();
        if (currentProject) {
            displayAnalysis(currentProject);
        }
    });
    
    document.getElementById('jointsModeBtn').addEventListener('click', () => {
        analysisMode = 'joints';
        updateModeButtons();
        if (currentProject) {
            displayAnalysis(currentProject);
        }
    });
}

function updateModeButtons() {
    const matrixBtn = document.getElementById('matrixModeBtn');
    const jointsBtn = document.getElementById('jointsModeBtn');
    
    if (analysisMode === 'matrix') {
        matrixBtn.classList.add('active');
        jointsBtn.classList.remove('active');
        document.getElementById('matrixAnalysis').style.display = 'block';
        document.getElementById('jointsAnalysis').style.display = 'none';
    } else {
        matrixBtn.classList.remove('active');
        jointsBtn.classList.add('active');
        document.getElementById('matrixAnalysis').style.display = 'none';
        document.getElementById('jointsAnalysis').style.display = 'block';
    }
}

// ========================================
// Tabs
// ========================================

function initializeTabs() {
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const tabName = e.target.dataset.tab;
            switchTab(tabName);
        });
    });
}

function switchTab(tabName) {
    // Update button states
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    document.querySelector(`[data-tab="${tabName}"]`).classList.add('active');
    
    // Update content visibility
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
    });
    document.getElementById(`${tabName}Tab`).classList.add('active');
}

// ========================================
// Project Loading
// ========================================

function loadProject(projectId) {
    currentProject = JSON.parse(JSON.stringify(PROJECTS[projectId])); // Deep copy
    showView('project');
    
    // Update project header
    document.getElementById('projectName').textContent = currentProject.name;
    document.getElementById('projectDescription').textContent = currentProject.description;
    
    // Load applied load parameter
    document.getElementById('appliedLoad').value = currentProject.appliedLoad;
    
    // Setup recalculate button
    document.getElementById('recalculate').addEventListener('click', () => {
        recalculateProject();
    });
    
    // Display all components
    displayTrussDiagram(currentProject);
    displayInputData(currentProject);
    performAnalysis(currentProject);
    displayAnalysis(currentProject);
    displayResults(currentProject);
    
    // Trigger MathJax
    if (typeof MathJax !== 'undefined') {
        MathJax.typesetPromise();
    }
}

function recalculateProject() {
    // Update applied load
    currentProject.appliedLoad = parseFloat(document.getElementById('appliedLoad').value);
    
    // Update buckling parameters
    BUCKLING_PARAMS.C = parseFloat(document.getElementById('bucklingC').value);
    BUCKLING_PARAMS.alpha = parseFloat(document.getElementById('bucklingAlpha').value);
    BUCKLING_PARAMS.L0 = parseFloat(document.getElementById('bucklingL0').value);
    BUCKLING_PARAMS.Ufit = parseFloat(document.getElementById('bucklingU').value);
    
    // Re-run analysis
    performAnalysis(currentProject);
    displayResults(currentProject);
    
    // Trigger MathJax
    if (typeof MathJax !== 'undefined') {
        MathJax.typesetPromise();
    }
}

// ========================================
// Truss Diagram Rendering
// ========================================

function displayTrussDiagram(project) {
    const container = document.getElementById('trussDiagram');
    
    // Calculate bounds
    const joints = project.joints;
    const xCoords = joints.map(j => j.x);
    const yCoords = joints.map(j => j.y);
    const minX = Math.min(...xCoords);
    const maxX = Math.max(...xCoords);
    const minY = Math.min(...yCoords);
    const maxY = Math.max(...yCoords);
    
    const padding = 40;
    const width = maxX - minX + 2 * padding;
    const height = maxY - minY + 2 * padding;
    
    // Create SVG
    let svg = `
        <svg width="100%" viewBox="0 0 ${width} ${height}" xmlns="http://www.w3.org/2000/svg">
            <defs>
                <marker id="arrowhead" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto">
                    <polygon points="0 0, 10 3, 0 6" fill="#ff8c42" />
                </marker>
            </defs>
    `;
    
    // Transform coordinates
    const transform = (x, y) => ({
        x: x - minX + padding,
        y: height - (y - minY + padding)
    });
    
    // Draw members
    project.members.forEach(member => {
        const jointA = joints.find(j => j.id === member.jointA);
        const jointB = joints.find(j => j.id === member.jointB);
        const pA = transform(jointA.x, jointA.y);
        const pB = transform(jointB.x, jointB.y);
        
        // Check if critical member
        const isCritical = project.analysis && project.analysis.criticalMemberId === member.id;
        const strokeColor = isCritical ? '#e67829' : '#1a5490';
        const strokeWidth = isCritical ? 3 : 2;
        
        svg += `<line x1="${pA.x}" y1="${pA.y}" x2="${pB.x}" y2="${pB.y}" 
                      stroke="${strokeColor}" stroke-width="${strokeWidth}" />`;
        
        // Member label
        const midX = (pA.x + pB.x) / 2;
        const midY = (pA.y + pB.y) / 2;
        svg += `<text x="${midX}" y="${midY - 5}" fill="#1f2937" font-size="11" 
                      font-family="IBM Plex Mono, monospace" text-anchor="middle">${member.label}</text>`;
    });
    
    // Draw joints
    joints.forEach(joint => {
        const p = transform(joint.x, joint.y);
        const isPin = project.supports.pinJoint === joint.id;
        const isRoller = project.supports.rollerJoint === joint.id;
        
        // Joint circle
        svg += `<circle cx="${p.x}" cy="${p.y}" r="5" fill="white" stroke="#1a5490" stroke-width="2" />`;
        
        // Joint label
        svg += `<text x="${p.x}" y="${p.y - 12}" fill="#1a5490" font-size="12" 
                      font-family="IBM Plex Sans, sans-serif" font-weight="600" 
                      text-anchor="middle">${joint.label}</text>`;
        
        // Support symbols
        if (isPin) {
            svg += `<path d="M ${p.x - 8} ${p.y + 8} L ${p.x} ${p.y} L ${p.x + 8} ${p.y + 8} Z" 
                          fill="none" stroke="#1a5490" stroke-width="2" />`;
            svg += `<line x1="${p.x - 10}" y1="${p.y + 10}" x2="${p.x + 10}" y2="${p.y + 10}" 
                          stroke="#1a5490" stroke-width="2" />`;
        }
        
        if (isRoller) {
            svg += `<circle cx="${p.x - 6}" cy="${p.y + 10}" r="3" fill="none" stroke="#1a5490" stroke-width="2" />`;
            svg += `<circle cx="${p.x + 6}" cy="${p.y + 10}" r="3" fill="none" stroke="#1a5490" stroke-width="2" />`;
            svg += `<line x1="${p.x - 10}" y1="${p.y + 15}" x2="${p.x + 10}" y2="${p.y + 15}" 
                          stroke="#1a5490" stroke-width="2" />`;
        }
    });
    
    // Draw loads
    project.loads.forEach(load => {
        const joint = joints.find(j => j.id === load.jointId);
        const p = transform(joint.x, joint.y);
        
        if (load.fy < 0) {
            // Downward arrow
            svg += `<line x1="${p.x}" y1="${p.y - 30}" x2="${p.x}" y2="${p.y - 8}" 
                          stroke="#ff8c42" stroke-width="2" marker-end="url(#arrowhead)" />`;
            svg += `<text x="${p.x + 8}" y="${p.y - 20}" fill="#ff8c42" font-size="12" 
                          font-family="IBM Plex Sans, sans-serif" font-weight="600">${load.label}</text>`;
        }
    });
    
    svg += '</svg>';
    container.innerHTML = svg;
}

// ========================================
// Input Data Display
// ========================================

function displayInputData(project) {
    // Joints table
    let jointsHTML = '<table><thead><tr><th>ID</th><th>Label</th><th>X (in)</th><th>Y (in)</th></tr></thead><tbody>';
    project.joints.forEach(joint => {
        jointsHTML += `<tr>
            <td>${joint.id}</td>
            <td>${joint.label}</td>
            <td>${joint.x.toFixed(2)}</td>
            <td>${joint.y.toFixed(2)}</td>
        </tr>`;
    });
    jointsHTML += '</tbody></table>';
    document.getElementById('jointsTable').innerHTML = jointsHTML;
    
    // Members table
    let membersHTML = '<table><thead><tr><th>ID</th><th>Label</th><th>Joint A</th><th>Joint B</th><th>Length (in)</th></tr></thead><tbody>';
    project.members.forEach(member => {
        const jointA = project.joints.find(j => j.id === member.jointA);
        const jointB = project.joints.find(j => j.id === member.jointB);
        const length = calculateDistance(jointA, jointB);
        membersHTML += `<tr>
            <td>${member.id}</td>
            <td>${member.label}</td>
            <td>${jointA.label}</td>
            <td>${jointB.label}</td>
            <td>${length.toFixed(3)}</td>
        </tr>`;
    });
    membersHTML += '</tbody></table>';
    document.getElementById('membersTable').innerHTML = membersHTML;
    
    // Supports table
    const pinJoint = project.joints.find(j => j.id === project.supports.pinJoint);
    const rollerJoint = project.joints.find(j => j.id === project.supports.rollerJoint);
    let supportsHTML = '<table><thead><tr><th>Type</th><th>Joint</th><th>Reactions</th></tr></thead><tbody>';
    supportsHTML += `<tr><td>Pin</td><td>${pinJoint.label}</td><td>Sx₁, Sy₁ (x and y)</td></tr>`;
    supportsHTML += `<tr><td>Roller</td><td>${rollerJoint.label}</td><td>Sy₂ (y only)</td></tr>`;
    supportsHTML += '</tbody></table>';
    document.getElementById('supportsTable').innerHTML = supportsHTML;
    
    // Loads table
    let loadsHTML = '<table><thead><tr><th>Joint</th><th>Fx (oz)</th><th>Fy (oz)</th><th>Label</th></tr></thead><tbody>';
    project.loads.forEach(load => {
        const joint = project.joints.find(j => j.id === load.jointId);
        loadsHTML += `<tr>
            <td>${joint.label}</td>
            <td>${load.fx.toFixed(2)}</td>
            <td>${load.fy.toFixed(2)}</td>
            <td>${load.label}</td>
        </tr>`;
    });
    loadsHTML += '</tbody></table>';
    document.getElementById('loadsTable').innerHTML = loadsHTML;
}

// ========================================
// Truss Analysis Calculations
// ========================================

function performAnalysis(project) {
    const J = project.joints.length;
    const M = project.members.length;
    
    // Build connection matrix C (J x M)
    const C = Array(J).fill(0).map(() => Array(M).fill(0));
    project.members.forEach((member, m) => {
        const aIdx = project.joints.findIndex(j => j.id === member.jointA);
        const bIdx = project.joints.findIndex(j => j.id === member.jointB);
        C[aIdx][m] = 1;
        C[bIdx][m] = 1;
    });
    
    // Build support matrices Sx and Sy (J x 3)
    const Sx = Array(J).fill(0).map(() => [0, 0, 0]);
    const Sy = Array(J).fill(0).map(() => [0, 0, 0]);
    
    const pinIdx = project.joints.findIndex(j => j.id === project.supports.pinJoint);
    const rollerIdx = project.joints.findIndex(j => j.id === project.supports.rollerJoint);
    
    Sx[pinIdx][0] = 1;   // Sx1 at pin
    Sy[pinIdx][1] = 1;   // Sy1 at pin
    Sy[rollerIdx][2] = 1; // Sy2 at roller
    
    // Build load vector L (2J x 1)
    const L = Array(2 * J).fill(0);
    project.loads.forEach(load => {
        const jointIdx = project.joints.findIndex(j => j.id === load.jointId);
        L[jointIdx] = load.fx;              // x-component
        L[J + jointIdx] = load.fy;          // y-component
    });
    
    // Build equilibrium matrix A (2J x M+3)
    const A = Array(2 * J).fill(0).map(() => Array(M + 3).fill(0));
    
    // Fill member force coefficients
    for (let m = 0; m < M; m++) {
        const member = project.members[m];
        const jointA = project.joints.find(j => j.id === member.jointA);
        const jointB = project.joints.find(j => j.id === member.jointB);
        
        const dx = jointB.x - jointA.x;
        const dy = jointB.y - jointA.y;
        const L = Math.sqrt(dx * dx + dy * dy);
        
        const ux = dx / L;  // Unit vector x
        const uy = dy / L;  // Unit vector y
        
        // For each joint connected to this member
        project.joints.forEach((joint, j) => {
            if (joint.id === member.jointA) {
                A[j][m] = ux;           // x-equilibrium
                A[J + j][m] = uy;       // y-equilibrium
            } else if (joint.id === member.jointB) {
                A[j][m] = -ux;          // x-equilibrium
                A[J + j][m] = -uy;      // y-equilibrium
            }
        });
    }
    
    // Fill support reaction coefficients
    for (let j = 0; j < J; j++) {
        A[j][M] = Sx[j][0];       // Sx1
        A[j][M + 1] = Sx[j][1];   // (always 0)
        A[j][M + 2] = Sx[j][2];   // (always 0)
        A[J + j][M] = Sy[j][0];   // (always 0)
        A[J + j][M + 1] = Sy[j][1]; // Sy1
        A[J + j][M + 2] = Sy[j][2]; // Sy2
    }
    
    // Solve: T = -A^(-1) * L
    const T = solveLinearSystem(A, L);
    
    // Extract member forces and reactions
    const memberForces = T.slice(0, M);
    const reactions = {
        Sx1: T[M],
        Sy1: T[M + 1],
        Sy2: T[M + 2]
    };
    
    // Calculate R values (force per unit load)
    const R = memberForces.map(force => force / project.appliedLoad);
    
    // Calculate member lengths and buckling strengths
    const memberData = project.members.map((member, m) => {
        const jointA = project.joints.find(j => j.id === member.jointA);
        const jointB = project.joints.find(j => j.id === member.jointB);
        const length = calculateDistance(jointA, jointB);
        const bucklingStrength = calculateBucklingStrength(length);
        const force = memberForces[m];
        const isCompression = R[m] < 0;
        
        return {
            id: member.id,
            label: member.label,
            length: length,
            force: force,
            R: R[m],
            isCompression: isCompression,
            bucklingStrength: bucklingStrength,
            Wfailure: isCompression ? -bucklingStrength / R[m] : Infinity
        };
    });
    
    // Find critical member
    const compressionMembers = memberData.filter(m => m.isCompression);
    let criticalMember = null;
    let maxLoad = Infinity;
    
    if (compressionMembers.length > 0) {
        compressionMembers.forEach(member => {
            if (member.Wfailure < maxLoad) {
                maxLoad = member.Wfailure;
                criticalMember = member;
            }
        });
    }
    
    // Calculate cost
    const totalLength = memberData.reduce((sum, m) => sum + m.length, 0);
    const cost = COST_PARAMS.CL * totalLength + COST_PARAMS.CJ * J;
    const efficiency = maxLoad / cost;
    
    // Store analysis results
    project.analysis = {
        J: J,
        M: M,
        C: C,
        Sx: Sx,
        Sy: Sy,
        L: L,
        A: A,
        T: T,
        memberForces: memberForces,
        reactions: reactions,
        R: R,
        memberData: memberData,
        criticalMember: criticalMember,
        criticalMemberId: criticalMember ? criticalMember.id : null,
        maxLoad: maxLoad,
        totalLength: totalLength,
        cost: cost,
        efficiency: efficiency
    };
}

function calculateDistance(jointA, jointB) {
    const dx = jointB.x - jointA.x;
    const dy = jointB.y - jointA.y;
    return Math.sqrt(dx * dx + dy * dy);
}

function calculateBucklingStrength(length) {
    // F = C * (L0/L)^α ± Ufit
    const { C, L0, alpha, Ufit } = BUCKLING_PARAMS;
    const F = C * Math.pow(L0 / length, alpha);
    return F; // Using nominal value (without uncertainty for simplicity)
}

// Simple matrix solver using Gaussian elimination
function solveLinearSystem(A, b) {
    const n = A.length;
    const m = A[0].length;
    
    // Create augmented matrix
    const aug = A.map((row, i) => [...row, -b[i]]);
    
    // Forward elimination
    for (let i = 0; i < n; i++) {
        // Find pivot
        let maxRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) {
                maxRow = k;
            }
        }
        
        // Swap rows
        [aug[i], aug[maxRow]] = [aug[maxRow], aug[i]];
        
        // Make all rows below this one 0 in current column
        for (let k = i + 1; k < n; k++) {
            const factor = aug[k][i] / aug[i][i];
            for (let j = i; j <= m; j++) {
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }
    
    // Back substitution
    const x = Array(m).fill(0);
    for (let i = m - 1; i >= 0; i--) {
        x[i] = aug[i][m];
        for (let j = i + 1; j < m; j++) {
            x[i] -= aug[i][j] * x[j];
        }
        x[i] /= aug[i][i];
    }
    
    return x;
}

// ========================================
// Analysis Display
// ========================================

function displayAnalysis(project) {
    if (!project.analysis) return;
    
    const { J, M, C, Sx, Sy, L, A } = project.analysis;
    
    // Update counts
    document.getElementById('jCount').textContent = J;
    document.getElementById('mCount').textContent = M;
    
    if (analysisMode === 'matrix') {
        displayMatrixAnalysis(project);
    } else {
        displayJointsAnalysis(project);
    }
}

function displayMatrixAnalysis(project) {
    const { J, M, C, Sx, Sy, L, A } = project.analysis;
    
    // Geometry vectors
    const X = project.joints.map(j => j.x.toFixed(2)).join(', ');
    const Y = project.joints.map(j => j.y.toFixed(2)).join(', ');
    document.getElementById('geometryVectors').innerHTML = `
        <div>\\mathbf{X} = [${X}]</div>
        <div>\\mathbf{Y} = [${Y}]</div>
    `;
    
    // Connection matrix
    let cMatrix = '\\mathbf{C} = \\begin{bmatrix}';
    C.forEach((row, i) => {
        cMatrix += row.join(' & ');
        if (i < C.length - 1) cMatrix += ' \\\\ ';
    });
    cMatrix += '\\end{bmatrix}';
    document.getElementById('connectionMatrix').innerHTML = `$$${cMatrix}$$`;
    
    // Support matrices
    let sxMatrix = '\\mathbf{S}_x = \\begin{bmatrix}';
    Sx.forEach((row, i) => {
        sxMatrix += row.join(' & ');
        if (i < Sx.length - 1) sxMatrix += ' \\\\ ';
    });
    sxMatrix += '\\end{bmatrix}';
    
    let syMatrix = '\\mathbf{S}_y = \\begin{bmatrix}';
    Sy.forEach((row, i) => {
        syMatrix += row.join(' & ');
        if (i < Sy.length - 1) syMatrix += ' \\\\ ';
    });
    syMatrix += '\\end{bmatrix}';
    
    document.getElementById('supportMatrices').innerHTML = `$$${sxMatrix} \\quad ${syMatrix}$$`;
    
    // Load vector
    let lVector = '\\mathbf{L} = \\begin{bmatrix}';
    L.forEach((val, i) => {
        lVector += val.toFixed(2);
        if (i < L.length - 1) lVector += ' \\\\ ';
    });
    lVector += '\\end{bmatrix}';
    document.getElementById('loadVector').innerHTML = `$$${lVector}$$`;
    
    // Equilibrium matrix (show first few rows due to size)
    const rowsToShow = Math.min(6, A.length);
    let aMatrix = '\\mathbf{A} = \\begin{bmatrix}';
    for (let i = 0; i < rowsToShow; i++) {
        aMatrix += A[i].map(v => v.toFixed(3)).join(' & ');
        if (i < rowsToShow - 1) aMatrix += ' \\\\ ';
    }
    if (A.length > rowsToShow) {
        aMatrix += ' \\\\ \\vdots';
    }
    aMatrix += '\\end{bmatrix}';
    document.getElementById('equilibriumMatrix').innerHTML = `$$${aMatrix}$$`;
    
    // Trigger MathJax
    if (typeof MathJax !== 'undefined') {
        MathJax.typesetPromise();
    }
}

function displayJointsAnalysis(project) {
    let html = '';
    
    project.joints.forEach((joint, idx) => {
        html += `<div class="joint-equations">`;
        html += `<h6>Joint ${joint.label} (${joint.x}, ${joint.y})</h6>`;
        
        // Find connected members
        const connectedMembers = project.members.filter(m => 
            m.jointA === joint.id || m.jointB === joint.id
        );
        
        // Build x-equilibrium equation
        let xEq = '\\sum F_x = ';
        const xTerms = [];
        
        connectedMembers.forEach(member => {
            const otherJoint = member.jointA === joint.id ? 
                project.joints.find(j => j.id === member.jointB) :
                project.joints.find(j => j.id === member.jointA);
            
            const dx = otherJoint.x - joint.x;
            const dy = otherJoint.y - joint.y;
            const L = Math.sqrt(dx * dx + dy * dy);
            const ux = dx / L;
            
            const sign = member.jointA === joint.id ? '' : '-';
            xTerms.push(`${sign}${ux.toFixed(3)} T_{${member.label}}`);
        });
        
        // Add support reactions
        if (project.supports.pinJoint === joint.id) {
            xTerms.push('S_{x1}');
        }
        
        xEq += xTerms.join(' + ') + ' = 0';
        
        // Build y-equilibrium equation
        let yEq = '\\sum F_y = ';
        const yTerms = [];
        
        connectedMembers.forEach(member => {
            const otherJoint = member.jointA === joint.id ? 
                project.joints.find(j => j.id === member.jointB) :
                project.joints.find(j => j.id === member.jointA);
            
            const dx = otherJoint.x - joint.x;
            const dy = otherJoint.y - joint.y;
            const L = Math.sqrt(dx * dx + dy * dy);
            const uy = dy / L;
            
            const sign = member.jointA === joint.id ? '' : '-';
            yTerms.push(`${sign}${uy.toFixed(3)} T_{${member.label}}`);
        });
        
        // Add support reactions
        if (project.supports.pinJoint === joint.id) {
            yTerms.push('S_{y1}');
        } else if (project.supports.rollerJoint === joint.id) {
            yTerms.push('S_{y2}');
        }
        
        // Add external loads
        const load = project.loads.find(l => l.jointId === joint.id);
        if (load) {
            if (load.fy !== 0) yTerms.push(`${load.fy.toFixed(2)}`);
        }
        
        yEq += yTerms.join(' + ') + ' = 0';
        
        html += `<div class="math-display">$$${xEq}$$</div>`;
        html += `<div class="math-display">$$${yEq}$$</div>`;
        html += `</div>`;
    });
    
    document.getElementById('jointsEquations').innerHTML = html;
    
    // Trigger MathJax
    if (typeof MathJax !== 'undefined') {
        MathJax.typesetPromise();
    }
}

// ========================================
// Results Display
// ========================================

function displayResults(project) {
    if (!project.analysis) return;
    
    const { maxLoad, cost, efficiency, criticalMember, memberData, reactions } = project.analysis;
    
    // Summary cards
    document.getElementById('maxLoad').textContent = maxLoad.toFixed(2);
    document.getElementById('trussCost').textContent = cost.toFixed(2);
    document.getElementById('efficiency').textContent = efficiency.toFixed(4);
    document.getElementById('criticalMember').textContent = criticalMember ? criticalMember.label : 'None';
    
    // Member forces table
    let memberHTML = '<table><thead><tr><th>Member</th><th>Length (in)</th><th>Force (oz)</th><th>Type</th><th>R</th><th>Buckling (oz)</th><th>W_fail (oz)</th></tr></thead><tbody>';
    memberData.forEach(member => {
        const isCritical = criticalMember && member.id === criticalMember.id;
        const rowClass = isCritical ? 'critical-row' : '';
        const forceClass = member.isCompression ? 'force-compression' : 'force-tension';
        const forceType = member.isCompression ? '(C)' : '(T)';
        
        memberHTML += `<tr class="${rowClass}">
            <td>${member.label}</td>
            <td>${member.length.toFixed(3)}</td>
            <td class="${forceClass}">${Math.abs(member.force).toFixed(3)}</td>
            <td>${forceType}</td>
            <td>${member.R.toFixed(4)}</td>
            <td>${member.isCompression ? member.bucklingStrength.toFixed(2) : 'N/A'}</td>
            <td>${member.isCompression ? member.Wfailure.toFixed(2) : 'N/A'}</td>
        </tr>`;
    });
    memberHTML += '</tbody></table>';
    document.getElementById('memberForcesTable').innerHTML = memberHTML;
    
    // Reaction forces table
    let reactionHTML = '<table><thead><tr><th>Reaction</th><th>Force (oz)</th></tr></thead><tbody>';
    reactionHTML += `<tr><td>Sx₁ (Pin horizontal)</td><td>${reactions.Sx1.toFixed(3)}</td></tr>`;
    reactionHTML += `<tr><td>Sy₁ (Pin vertical)</td><td>${reactions.Sy1.toFixed(3)}</td></tr>`;
    reactionHTML += `<tr><td>Sy₂ (Roller vertical)</td><td>${reactions.Sy2.toFixed(3)}</td></tr>`;
    reactionHTML += '</tbody></table>';
    document.getElementById('reactionForcesTable').innerHTML = reactionHTML;
}
