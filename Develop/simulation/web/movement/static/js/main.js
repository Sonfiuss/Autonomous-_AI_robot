/**
 * Main Controller - Simplified: Click to move robot
 */

class Simulator {
    constructor() {
        this.canvas = document.getElementById('simulatorCanvas');
        this.ctx = this.canvas.getContext('2d');
        
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
        
        this.grid = new Grid(this.canvas, {
            width: this.canvas.width,
            height: this.canvas.height,
            cellSize: 20,
            scale: 2
        });
        
        this.grid.setOrigin(this.canvas.width / 2, this.canvas.height - 50);
        
        this.robot = new Robot(this.grid, {
            outerRadius: 19,
            innerRadius: 14.4,
            x: 0,
            y: 0,
            theta: Math.PI / 2
        });
        
        this.obstacleManager = new ObstacleManager(this.grid);
        this.pathFinder = new PathFinder(this.grid, this.obstacleManager);
        this.pathRenderer = new PathRenderer(this.grid);
        
        this.kinematics = new Kinematics({
            wheelDistance: 14.4,
            wheelRadius: 4.1
        });
        
        this.goalPos = null;
        this.currentPath = [];
        this.isAnimating = false;
        this.animationFrame = null;
        this.pathProgress = 0;
        this.obstacleMode = null;
        this.isDrawingObstacle = false;
        this.drawStartPos = null;
        this.drawCurrentPos = null;
        this.animationSpeed = 30;
        this.maxOmega = 0.5;
        this.lastTimestamp = 0;
        this.smoothPath = [];
        
        this.setupEventListeners();
        this.render();
        this.log('Ready. Click anywhere to move robot.', 'info');
    }
    
    resizeCanvas() {
        const container = this.canvas.parentElement;
        const header = container.querySelector('.canvas-header');
        const headerHeight = header ? header.offsetHeight : 40;
        
        this.canvas.width = container.clientWidth - 20;
        this.canvas.height = container.clientHeight - headerHeight - 20;
        
        if (this.grid) {
            this.grid.width = this.canvas.width;
            this.grid.height = this.canvas.height;
            this.grid.setOrigin(this.canvas.width / 2, this.canvas.height - 50);
            this.render();
        }
    }
    
    setupEventListeners() {
        this.canvas.addEventListener('mousedown', (e) => this.handleMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        this.canvas.addEventListener('mouseup', (e) => this.handleMouseUp(e));
        this.canvas.addEventListener('mouseleave', (e) => this.handleMouseLeave(e));
        this.canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            this.handleRightClick(e);
        });
        
        document.querySelectorAll('.tool-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const tool = e.target.dataset.tool;
                document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
                e.target.classList.add('active');
                this.obstacleMode = tool;
                this.canvas.style.cursor = 'crosshair';
                this.log(`Draw ${tool}: Click and drag on map`, 'info');
            });
        });
        
        document.getElementById('stopSim').addEventListener('click', () => this.stopAnimation());
        document.getElementById('resetSim').addEventListener('click', () => this.reset());
        document.getElementById('clearObstacles').addEventListener('click', () => {
            this.obstacleManager.clear();
            this.render();
            this.log('Obstacles cleared', 'info');
        });
        
        document.getElementById('outerRadius').addEventListener('change', (e) => {
            this.robot.setConfig({ outerRadius: parseFloat(e.target.value) });
            this.render();
        });
        document.getElementById('innerRadius').addEventListener('change', (e) => {
            this.robot.setConfig({ innerRadius: parseFloat(e.target.value) });
            this.render();
        });
        document.getElementById('maxVelocity').addEventListener('change', (e) => {
            this.animationSpeed = parseFloat(e.target.value) * 100;
        });
        document.getElementById('maxOmega').addEventListener('change', (e) => {
            this.maxOmega = parseFloat(e.target.value);
        });
        document.getElementById('safetyMargin').addEventListener('change', (e) => {
            this.robot.setConfig({ safetyMargin: parseFloat(e.target.value) });
            this.render();
        });
        document.getElementById('cellSize').addEventListener('change', (e) => {
            this.grid.setConfig({ cellSize: parseInt(e.target.value) });
            this.render();
        });
        document.getElementById('gridScale').addEventListener('change', (e) => {
            this.grid.setConfig({ scale: parseFloat(e.target.value) });
            this.render();
        });
        document.getElementById('obstacleSize').addEventListener('change', (e) => {
            this.obstacleManager.defaultSize = parseFloat(e.target.value);
        });
        
        document.getElementById('wheelDistance').addEventListener('change', (e) => {
            this.kinematics.setConfig({ wheelDistance: parseFloat(e.target.value) });
        });
        document.getElementById('wheelRadius').addEventListener('change', (e) => {
            this.kinematics.setConfig({ wheelRadius: parseFloat(e.target.value) });
        });
    }
    
    clearObstacleMode() {
        this.obstacleMode = null;
        this.isDrawingObstacle = false;
        this.drawStartPos = null;
        this.drawCurrentPos = null;
        document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
        this.canvas.style.cursor = 'crosshair';
    }
    
    handleMouseDown(event) {
        if (event.button !== 0) return;
        const rect = this.canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const world = this.grid.canvasToWorld(px, py);
        if (this.obstacleMode) {
            this.isDrawingObstacle = true;
            this.drawStartPos = { x: world.x, y: world.y };
            this.drawCurrentPos = { x: world.x, y: world.y };
        }
    }
    
    handleMouseUp(event) {
        if (event.button !== 0) return;
        const rect = this.canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const world = this.grid.canvasToWorld(px, py);
        
        if (this.isDrawingObstacle && this.drawStartPos) {
            const dx = Math.abs(world.x - this.drawStartPos.x);
            const dy = Math.abs(world.y - this.drawStartPos.y);
            const cx = (this.drawStartPos.x + world.x) / 2;
            const cy = (this.drawStartPos.y + world.y) / 2;
            this.addObstacleWithSize(cx, cy, this.obstacleMode, dx, dy);
            this.log(`Created ${this.obstacleMode} at (${cx.toFixed(0)}, ${cy.toFixed(0)})`, 'success');
            this.clearObstacleMode();
            this.log('Back to move mode. Click to move robot.', 'info');
        } else if (!this.obstacleMode) {
            this.moveRobotTo(world.x, world.y);
        }
        
        this.isDrawingObstacle = false;
        this.drawStartPos = null;
        this.drawCurrentPos = null;
        this.render();
    }
    
    handleMouseLeave() {
        if (this.isDrawingObstacle) {
            this.isDrawingObstacle = false;
            this.drawStartPos = null;
            this.drawCurrentPos = null;
            this.render();
        }
    }
    
    addObstacleWithSize(cx, cy, type, width, height) {
        const minSize = 10;
        width = Math.max(width, minSize);
        height = Math.max(height, minSize);
        let obstacle;
        switch (type) {
            case 'circle':
                obstacle = new CircleObstacle(cx, cy, Math.max(width, height) / 2);
                break;
            case 'rectangle':
                obstacle = new RectangleObstacle(cx, cy, width, height);
                break;
            case 'parallelogram':
                obstacle = new ParallelogramObstacle(cx, cy, width, height, 0.3);
                break;
        }
        if (obstacle) {
            this.obstacleManager.obstacles.push(obstacle);
        }
    }

    handleRightClick(event) {
        const rect = this.canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const world = this.grid.canvasToWorld(px, py);
        if (this.obstacleManager.removeObstacleAt(world.x, world.y)) {
            this.log('Obstacle removed', 'info');
            this.render();
        }
    }
    
    handleMouseMove(event) {
        const rect = this.canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const world = this.grid.canvasToWorld(px, py);
        document.getElementById('mousePos').textContent = `Mouse: (${world.x.toFixed(0)}, ${world.y.toFixed(0)}) cm`;
        if (this.isDrawingObstacle && this.drawStartPos) {
            this.drawCurrentPos = { x: world.x, y: world.y };
            this.render();
            this.drawObstaclePreview();
        }
    }
    
    drawObstaclePreview() {
        if (!this.drawStartPos || !this.drawCurrentPos) return;
        const ctx = this.ctx;
        const start = this.grid.worldToCanvas(this.drawStartPos.x, this.drawStartPos.y);
        const current = this.grid.worldToCanvas(this.drawCurrentPos.x, this.drawCurrentPos.y);
        const cx = (start.x + current.x) / 2;
        const cy = (start.y + current.y) / 2;
        const width = Math.abs(current.x - start.x);
        const height = Math.abs(current.y - start.y);
        
        ctx.save();
        ctx.strokeStyle = '#ffff00';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.fillStyle = 'rgba(255, 255, 0, 0.2)';
        
        switch (this.obstacleMode) {
            case 'circle': {
                const radius = Math.max(width, height) / 2;
                ctx.beginPath();
                ctx.arc(cx, cy, radius, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();
                break;
            }
            case 'rectangle':
                ctx.beginPath();
                ctx.rect(cx - width/2, cy - height/2, width, height);
                ctx.fill();
                ctx.stroke();
                break;
            case 'parallelogram': {
                const skew = width * 0.3;
                ctx.beginPath();
                ctx.moveTo(cx - width/2 + skew, cy - height/2);
                ctx.lineTo(cx + width/2 + skew, cy - height/2);
                ctx.lineTo(cx + width/2 - skew, cy + height/2);
                ctx.lineTo(cx - width/2 - skew, cy + height/2);
                ctx.closePath();
                ctx.fill();
                ctx.stroke();
                break;
            }
        }
        
        ctx.restore();
        const worldWidth = Math.abs(this.drawCurrentPos.x - this.drawStartPos.x);
        const worldHeight = Math.abs(this.drawCurrentPos.y - this.drawStartPos.y);
        ctx.fillStyle = '#ffff00';
        ctx.font = '12px Arial';
        ctx.fillText(`${worldWidth.toFixed(0)} x ${worldHeight.toFixed(0)} cm`, current.x + 10, current.y - 10);
    }
    
    moveRobotTo(targetX, targetY) {
        this.stopAnimation();
        this.goalPos = { x: targetX, y: targetY };
        this.updateKinematics(targetX, targetY);
        const startPos = { x: this.robot.x, y: this.robot.y };
        const path = this.pathFinder.findPath(startPos, this.goalPos, this.robot.getBoundingRadius());
        if (path && path.length > 1) {
            this.currentPath = path;
            this.pathRenderer.setPath(path);
            const length = this.pathRenderer.getPathLength();
            this.log(`Moving to (${targetX.toFixed(0)}, ${targetY.toFixed(0)}). Distance: ${length.toFixed(0)}cm`, 'success');
            this.startAnimation();
        } else if (path && path.length === 1) {
            this.log('Already at destination', 'info');
        } else {
            this.log('No path found! Obstacle in the way.', 'error');
            this.currentPath = [];
            this.pathRenderer.setPath([]);
        }
        this.render();
    }
    
    updateKinematics(targetX, targetY) {
        const result = this.kinematics.calculateFromTarget(
            this.robot.x,
            this.robot.y,
            this.robot.theta,
            targetX,
            targetY,
            this.animationSpeed,
            true
        );
        document.getElementById('kinAngle').textContent = result.angleDegrees.toFixed(1) + '°';
        document.getElementById('kinDistance').textContent = result.distance.toFixed(1) + ' cm';
        document.getElementById('kinVx').textContent = result.Vx_robot.toFixed(2) + ' cm/s';
        document.getElementById('kinVy').textContent = result.Vy_robot.toFixed(2) + ' cm/s';
        const wa = result.wheelAngularVelocities;
        document.getElementById('kinOmegaLeft').textContent = wa.omega1_left.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaBack').textContent = wa.omega2_back.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaRight').textContent = wa.omega3_right.toFixed(3) + ' rad/s';
        this.currentKinematics = result;
        return result;
    }
    
    updateKinematicsRealtimeDisplay() {
        if (!this.isAnimating || this.currentPath.length < 2) return;
        let nextPoint = this.goalPos;
        let totalDist = 0;
        const segmentLengths = [];
        
        for (let i = 1; i < this.currentPath.length; i++) {
            const dx = this.currentPath[i].x - this.currentPath[i - 1].x;
            const dy = this.currentPath[i].y - this.currentPath[i - 1].y;
            segmentLengths.push(Math.sqrt(dx * dx + dy * dy));
            totalDist += segmentLengths[segmentLengths.length - 1];
        }
        
        let accumulated = 0;
        for (let i = 0; i < segmentLengths.length; i++) {
            if (accumulated + segmentLengths[i] >= this.pathProgress) {
                nextPoint = this.currentPath[i + 1];
                break;
            }
            accumulated += segmentLengths[i];
        }
        
        const robotX = this.robot.x;
        const robotY = this.robot.y;
        const robotTheta = this.robot.theta;
        const dx = nextPoint.x - robotX;
        const dy = nextPoint.y - robotY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        const remainingDist = totalDist - this.pathProgress;
        const targetAngle = Math.atan2(dy, dx);
        this.kinematics.setRobotHeading(robotTheta);
        const Vx_world = (distance > 0.1) ? (dx / distance) * this.animationSpeed : 0;
        const Vy_world = (distance > 0.1) ? (dy / distance) * this.animationSpeed : 0;
        let omega = 0;
        if (distance > 5) {
            const angleDiff = this.normalizeAngle(targetAngle - robotTheta);
            const kp = 0.8;
            omega = Math.max(-this.maxOmega, Math.min(this.maxOmega, kp * angleDiff));
        }
        const robotVel = this.kinematics.worldToMobile(Vx_world, Vy_world);
        const wheelVel = this.kinematics.inverseMobile(robotVel.Vxm, robotVel.Vym, omega);
        const wheelOmega = this.kinematics.getWheelAngularVelocities(wheelVel);
        document.getElementById('kinAngle').textContent = (targetAngle * 180 / Math.PI).toFixed(1) + '°';
        document.getElementById('kinDistance').textContent = remainingDist.toFixed(1) + ' cm';
        document.getElementById('kinVx').textContent = robotVel.Vxm.toFixed(2) + ' cm/s';
        document.getElementById('kinVy').textContent = robotVel.Vym.toFixed(2) + ' cm/s';
        document.getElementById('kinOmegaRobot').textContent = omega.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaLeft').textContent = wheelOmega.omega1_left.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaBack').textContent = wheelOmega.omega2_back.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaRight').textContent = wheelOmega.omega3_right.toFixed(3) + ' rad/s';
        this.robot.setWheelOmega(wheelOmega.omega1_left, wheelOmega.omega2_back, wheelOmega.omega3_right);
        this.currentOmega = omega;
        this.robot.omega = omega;
    }
    
    normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    startAnimation() {
        if (this.currentPath.length < 2 || this.isAnimating) return;
        this.isAnimating = true;
        this.pathProgress = 0;
        this.lastTimestamp = performance.now();
        this.smoothPath = this.generateSmoothPath(this.currentPath);
        this.smoothPathLength = this.calculateSmoothPathLength();
        this.animate();
    }
    
    generateSmoothPath(waypoints) {
        if (waypoints.length < 2) return waypoints;
        const smoothPoints = [];
        const tension = 0.3;
        const segments = 20;
        for (let i = 0; i < waypoints.length - 1; i++) {
            const p0 = waypoints[Math.max(0, i - 1)];
            const p1 = waypoints[i];
            const p2 = waypoints[i + 1];
            const p3 = waypoints[Math.min(waypoints.length - 1, i + 2)];
            for (let j = 0; j < segments; j++) {
                const t = j / segments;
                const point = this.catmullRom(p0, p1, p2, p3, t, tension);
                smoothPoints.push(point);
            }
        }
        smoothPoints.push(waypoints[waypoints.length - 1]);
        return smoothPoints;
    }
    
    catmullRom(p0, p1, p2, p3, t) {
        const t2 = t * t;
        const t3 = t2 * t;
        const x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2*p0.x - 5*p1.x + 4*p2.x - p3.x) * t2 + (-p0.x + 3*p1.x - 3*p2.x + p3.x) * t3);
        const y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2*p0.y - 5*p1.y + 4*p2.y - p3.y) * t2 + (-p0.y + 3*p1.y - 3*p2.y + p3.y) * t3);
        return { x, y };
    }
    
    calculateSmoothPathLength() {
        let length = 0;
        for (let i = 1; i < this.smoothPath.length; i++) {
            const dx = this.smoothPath[i].x - this.smoothPath[i-1].x;
            const dy = this.smoothPath[i].y - this.smoothPath[i-1].y;
            length += Math.sqrt(dx*dx + dy*dy);
        }
        return length;
    }
    
    animate(timestamp = performance.now()) {
        if (!this.isAnimating) return;
        const dt = (timestamp - this.lastTimestamp) / 1000;
        this.lastTimestamp = timestamp;
        const path = this.smoothPath || this.currentPath;
        let totalLength = 0;
        const segmentLengths = [];
        
        for (let i = 1; i < path.length; i++) {
            const dx = path[i].x - path[i - 1].x;
            const dy = path[i].y - path[i - 1].y;
            const len = Math.sqrt(dx * dx + dy * dy);
            segmentLengths.push(len);
            totalLength += len;
        }
        
        this.pathProgress += this.animationSpeed * dt;
        
        if (this.pathProgress >= totalLength) {
            this.robot.setPosition(this.goalPos.x, this.goalPos.y);
            this.robot.vx = 0;
            this.robot.vy = 0;
            this.robot.omega = 0;
            this.robot.setWheelOmega(0, 0, 0);
            this.isAnimating = false;
            this.currentPath = [];
            this.smoothPath = [];
            this.pathRenderer.setPath([]);
            this.goalPos = null;
            this.log('Arrived!', 'success');
            this.updateKinematicsDisplayStopped();
            this.render();
            return;
        }
        
        let accumulated = 0;
        for (let i = 0; i < segmentLengths.length; i++) {
            if (accumulated + segmentLengths[i] >= this.pathProgress) {
                const t = (this.pathProgress - accumulated) / segmentLengths[i];
                const p1 = path[i];
                const p2 = path[i + 1];
                const x = p1.x + (p2.x - p1.x) * t;
                const y = p1.y + (p2.y - p1.y) * t;
                const tangentAngle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
                const targetTheta = tangentAngle;
                const currentTheta = this.robot.theta;
                let angleDiff = this.normalizeAngle(targetTheta - currentTheta);
                const maxRotation = this.maxOmega * dt;
                if (Math.abs(angleDiff) > maxRotation) {
                    angleDiff = Math.sign(angleDiff) * maxRotation;
                }
                const newTheta = currentTheta + angleDiff;
                this.robot.setPosition(x, y, newTheta);
                this.robot.omega = angleDiff / dt;
                const speed = this.animationSpeed;
                this.robot.vx = (p2.x - p1.x) / segmentLengths[i] * speed;
                this.robot.vy = (p2.y - p1.y) / segmentLengths[i] * speed;
                break;
            }
            accumulated += segmentLengths[i];
        }
        
        this.updateKinematicsRealtimeDisplay();
        this.renderSmoothPath();
        this.render();
        this.animationFrame = requestAnimationFrame((t) => this.animate(t));
    }
    
    renderSmoothPath() {
        if (!this.smoothPath || this.smoothPath.length < 2) return;
        const ctx = this.ctx;
        ctx.save();
        ctx.strokeStyle = '#00ffff';
        ctx.lineWidth = 2;
        ctx.setLineDash([]);
        ctx.beginPath();
        const start = this.grid.worldToCanvas(this.smoothPath[0].x, this.smoothPath[0].y);
        ctx.moveTo(start.x, start.y);
        for (let i = 1; i < this.smoothPath.length; i++) {
            const p = this.grid.worldToCanvas(this.smoothPath[i].x, this.smoothPath[i].y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.restore();
    }
    
    updateKinematicsDisplayStopped() {
        document.getElementById('kinAngle').textContent = '--°';
        document.getElementById('kinDistance').textContent = '0 cm';
        document.getElementById('kinVx').textContent = '0 cm/s';
        document.getElementById('kinVy').textContent = '0 cm/s';
        document.getElementById('kinOmegaRobot').textContent = '0 rad/s';
        document.getElementById('kinOmegaLeft').textContent = '0 rad/s';
        document.getElementById('kinOmegaBack').textContent = '0 rad/s';
        document.getElementById('kinOmegaRight').textContent = '0 rad/s';
    }
    
    stopAnimation() {
        this.isAnimating = false;
        if (this.animationFrame) {
            cancelAnimationFrame(this.animationFrame);
            this.animationFrame = null;
        }
        this.robot.vx = 0;
        this.robot.vy = 0;
        this.robot.omega = 0;
        this.robot.setWheelOmega(0, 0, 0);
    }
    
    reset() {
        this.stopAnimation();
        this.pathProgress = 0;
        this.currentPath = [];
        this.smoothPath = [];
        this.pathRenderer.setPath([]);
        this.goalPos = null;
        this.robot.setPosition(0, 0, Math.PI / 2);
        this.robot.vx = 0;
        this.robot.vy = 0;
        this.robot.omega = 0;
        this.robot.setWheelOmega(0, 0, 0);
        document.getElementById('kinAngle').textContent = '--°';
        document.getElementById('kinDistance').textContent = '-- cm';
        document.getElementById('kinVx').textContent = '-- cm/s';
        document.getElementById('kinVy').textContent = '-- cm/s';
        document.getElementById('kinOmegaRobot').textContent = '-- rad/s';
        document.getElementById('kinOmegaLeft').textContent = '-- rad/s';
        document.getElementById('kinOmegaBack').textContent = '-- rad/s';
        document.getElementById('kinOmegaRight').textContent = '-- rad/s';
        this.log('Robot reset to origin', 'info');
        this.render();
    }
    
    render() {
        this.grid.draw();
        this.obstacleManager.draw();
        this.pathRenderer.draw();
        if (this.smoothPath && this.smoothPath.length > 1) {
            this.renderSmoothPath();
        }
        if (this.goalPos) {
            this.drawGoalMarker();
        }
        this.robot.draw(true);
        if (this.isAnimating) {
            this.robot.drawVelocityVector();
        }
        document.getElementById('robotPos').textContent = `Robot: (${this.robot.x.toFixed(0)}, ${this.robot.y.toFixed(0)}) θ=${(this.robot.theta * 180 / Math.PI).toFixed(0)}°`;
    }
    
    drawGoalMarker() {
        const pos = this.grid.worldToCanvas(this.goalPos.x, this.goalPos.y);
        const ctx = this.ctx;
        ctx.strokeStyle = '#ff0000';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 12, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 6, 0, Math.PI * 2);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 2, 0, Math.PI * 2);
        ctx.fillStyle = '#ff0000';
        ctx.fill();
    }
    
    log(message, type = 'info') {
        const logDiv = document.getElementById('statusLog');
        const p = document.createElement('p');
        p.className = type;
        p.textContent = message;
        logDiv.insertBefore(p, logDiv.firstChild);
        while (logDiv.children.length > 8) {
            logDiv.removeChild(logDiv.lastChild);
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.simulator = new Simulator();
});
