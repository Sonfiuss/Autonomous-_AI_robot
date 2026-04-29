/**
 * Ultrasonic Radar Visualization
 * Classic phosphor-persistence radar style
 * Green sweep trail + Red obstacle zones
 */

// ============== CONFIGURATION ==============
const CONFIG = {
    minAngle: -30,
    maxAngle: 30,
    maxRange: 150,         // cm
    decayAlpha: 0.04,      // phosphor fade speed per frame
    gridColor: 'rgba(0, 255, 0, 0.25)',
    gridLineColor: 'rgba(0, 255, 0, 0.5)',
    textColor: '#00ff00',
    bgColor: '#0a0a0a'
};

// ============== CANVAS SETUP ==============
const canvas = document.getElementById('radarCanvas');
const ctx = canvas.getContext('2d');
const W = canvas.width;
const H = canvas.height;

const centerX = W / 2;
const centerY = H;
const maxRadius = H - 40;

// ============== STATE ==============
let socket        = null;
let currentAngle  = CONFIG.minAngle;
let currentDist   = -1;
let isConnected   = false;

// ============== DOM ELEMENTS ==============
const portSelect      = document.getElementById('portSelect');
const connectBtn      = document.getElementById('connectBtn');
const disconnectBtn   = document.getElementById('disconnectBtn');
const refreshBtn      = document.getElementById('refreshBtn');
const testBtn         = document.getElementById('testBtn');
const statusEl        = document.getElementById('status');
const angleDisplay    = document.getElementById('angleDisplay');
const distanceDisplay = document.getElementById('distanceDisplay');

// ============== COORDINATE CONVERSION ==============
function angleToRad(deg) {
    return (deg - 90) * Math.PI / 180;
}

function polarXY(deg, dist) {
    const r   = (dist / CONFIG.maxRange) * maxRadius;
    const rad = angleToRad(deg);
    return { x: centerX + r * Math.cos(rad), y: centerY + r * Math.sin(rad) };
}

// ============== GRID ==============
function drawGrid() {
    ctx.save();
    ctx.lineWidth = 1;

    const ranges = [50, 100, 150];
    ranges.forEach(r => {
        const px = (r / CONFIG.maxRange) * maxRadius;
        ctx.strokeStyle = CONFIG.gridColor;
        ctx.beginPath();
        ctx.arc(centerX, centerY, px, Math.PI, 2 * Math.PI);
        ctx.stroke();

        const labelX = centerX + px * Math.cos(angleToRad(22));
        const labelY = centerY + px * Math.sin(angleToRad(22));
        ctx.fillStyle = CONFIG.textColor;
        ctx.font = '11px Courier New';
        ctx.textAlign = 'left';
        ctx.fillText(`${r} cm`, labelX + 4, labelY - 3);
    });

    ctx.strokeStyle = CONFIG.gridColor;
    for (let a = CONFIG.minAngle; a <= CONFIG.maxAngle; a += 15) {
        const rad = angleToRad(a);
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(centerX + maxRadius * Math.cos(rad), centerY + maxRadius * Math.sin(rad));
        ctx.stroke();

        const lr = maxRadius + 18;
        ctx.fillStyle = CONFIG.textColor;
        ctx.font = '13px Courier New';
        ctx.textAlign = 'center';
        ctx.fillText(`${a}°`, centerX + lr * Math.cos(rad), centerY + lr * Math.sin(rad));
    }

    ctx.strokeStyle = CONFIG.gridLineColor;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(centerX - maxRadius - 25, centerY);
    ctx.lineTo(centerX + maxRadius + 25, centerY);
    ctx.stroke();

    ctx.restore();
}

// ============== SWEEP LINE ==============
function drawSweepLine() {
    const rad  = angleToRad(currentAngle);
    const endX = centerX + maxRadius * Math.cos(rad);
    const endY = centerY + maxRadius * Math.sin(rad);

    ctx.save();
    const grad = ctx.createLinearGradient(centerX, centerY, endX, endY);
    grad.addColorStop(0, 'rgba(0,255,0,0.0)');
    grad.addColorStop(1, 'rgba(0,255,0,1.0)');
    ctx.shadowColor = '#00ff00';
    ctx.shadowBlur  = 14;
    ctx.strokeStyle = grad;
    ctx.lineWidth   = 2.5;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(endX, endY);
    ctx.stroke();
    ctx.restore();
}

// ============== INFO OVERLAY ==============
function drawInfo() {
    ctx.save();
    ctx.fillStyle = CONFIG.textColor;
    ctx.font = 'bold 13px Courier New';
    ctx.textAlign = 'left';
    ctx.fillText(`Angle    \u2014 ${currentAngle}\u00b0`, 18, 24);
    ctx.fillText(`Distance \u2014 ${currentDist > 0 ? currentDist.toFixed(1) + ' cm' : '--'}`, 18, 42);
    ctx.restore();
}

// ============== PHOSPHOR SECTOR (called per data point) ==============
function drawSector(angle, distance) {
    const halfStep = 1.2 * Math.PI / 180;
    const rad      = angleToRad(angle);

    if (distance <= 0 || distance > CONFIG.maxRange) {
        // Clear path — green sector to edge
        ctx.save();
        ctx.fillStyle = 'rgba(0, 255, 0, 0.18)';
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.arc(centerX, centerY, maxRadius, rad - halfStep, rad + halfStep);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
    } else {
        const clearR = (distance / CONFIG.maxRange) * maxRadius;

        // Clear path — green
        if (clearR > 2) {
            ctx.save();
            ctx.fillStyle = 'rgba(0, 255, 0, 0.15)';
            ctx.beginPath();
            ctx.moveTo(centerX, centerY);
            ctx.arc(centerX, centerY, clearR, rad - halfStep, rad + halfStep);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }

        // Obstacle band — red
        const thick = Math.max(10, maxRadius * 0.05);
        ctx.save();
        ctx.fillStyle   = 'rgba(255, 35, 0, 0.85)';
        ctx.shadowColor = '#ff2000';
        ctx.shadowBlur  = 6;
        ctx.beginPath();
        ctx.arc(centerX, centerY, clearR + thick,           rad - halfStep, rad + halfStep);
        ctx.arc(centerX, centerY, Math.max(0, clearR - 3),  rad + halfStep, rad - halfStep, true);
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        // Bright dot at impact
        const pt = polarXY(angle, distance);
        ctx.save();
        ctx.fillStyle   = '#ff4400';
        ctx.shadowColor = '#ff2000';
        ctx.shadowBlur  = 12;
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 3, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
    }
}

// ============== RENDER LOOP ==============
function render() {
    // 1. Phosphor decay
    ctx.fillStyle = `rgba(10, 10, 10, ${CONFIG.decayAlpha})`;
    ctx.fillRect(0, 0, W, H);

    // 2. Crisp grid on top
    drawGrid();

    // 3. Bright sweep line
    drawSweepLine();

    // 4. Info text
    drawInfo();

    requestAnimationFrame(render);
}

// ============== DATA HANDLING ==============
function processRadarData(data) {
    if (data.startsWith('RADAR:')) {
        const parts = data.substring(6).split(',');
        if (parts.length === 2) {
            const angle    = parseInt(parts[0]);
            const distance = parseFloat(parts[1]);

            currentAngle = angle;
            currentDist  = distance;

            drawSector(angle, distance);

            angleDisplay.textContent    = `G\u00f3c: ${angle}\u00b0`;
            distanceDisplay.textContent = distance > 0
                ? `Kho\u1ea3ng c\u00e1ch: ${distance.toFixed(1)} cm`
                : `Kho\u1ea3ng c\u00e1ch: -- cm`;
        }
    } else if (data.startsWith('RADAR_CONFIG:')) {
        const parts = data.substring(13).split(',');
        if (parts.length === 3) {
            CONFIG.minAngle = parseInt(parts[0]);
            CONFIG.maxAngle = parseInt(parts[1]);
        }
    }
}

// ============== SOCKET.IO ==============
function initSocket() {
    socket = io();

    socket.on('connect', () => { statusEl.textContent = '\u0110\u00e3 k\u1ebft n\u1ed1i server'; });

    socket.on('disconnect', () => {
        isConnected = false;
        statusEl.textContent = 'M\u1ea5t k\u1ebft n\u1ed1i server';
        statusEl.classList.remove('connected');
        updateButtons(false);
    });

    socket.on('status', (data) => {
        isConnected = data.connected;
        if (data.connected) {
            statusEl.textContent = `\u0110\u00e3 k\u1ebft n\u1ed1i: ${data.port}`;
            statusEl.classList.add('connected');
            updateButtons(true);
        } else {
            statusEl.textContent = data.error ? `L\u1ed7i: ${data.error}` : (data.message || '\u0110\u00e3 ng\u1eaft k\u1ebft n\u1ed1i');
            statusEl.classList.remove('connected');
            updateButtons(false);
        }
    });

    socket.on('serial_data', (data) => { if (data.data) processRadarData(data.data); });
}

function updateButtons(connected) {
    connectBtn.disabled    = connected;
    testBtn.disabled       = connected;
    disconnectBtn.disabled = !connected;
    portSelect.disabled    = connected;
}

function connect(port, testMode = false) {
    socket.emit('connect_serial', { port, baud: 115200, test: testMode });
}

function disconnect() { socket.emit('disconnect_serial'); }

async function loadPorts() {
    try {
        const ports = await (await fetch('/ports')).json();
        portSelect.innerHTML = '<option value="">-- Ch\u1ecdn port --</option>';
        ports.forEach(p => {
            const opt = document.createElement('option');
            opt.value = p.device;
            opt.textContent = `${p.device} - ${p.description}`;
            portSelect.appendChild(opt);
        });
    } catch { statusEl.textContent = 'Kh\u00f4ng th\u1ec3 t\u1ea3i danh s\u00e1ch ports.'; }
}

// ============== EVENT LISTENERS ==============
connectBtn.addEventListener('click', () => {
    const port = portSelect.value;
    if (port) connect(port, false);
    else alert('Vui l\u00f2ng ch\u1ecdn COM port');
});
disconnectBtn.addEventListener('click', disconnect);
refreshBtn.addEventListener('click', loadPorts);
testBtn.addEventListener('click', () => connect('TEST', true));

// ============== INIT ==============
ctx.fillStyle = CONFIG.bgColor;
ctx.fillRect(0, 0, W, H);
initSocket();
loadPorts();
render();

