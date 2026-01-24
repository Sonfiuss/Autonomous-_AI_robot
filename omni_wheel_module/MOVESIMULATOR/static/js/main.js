/**
 * Main Controller - Simplified: Click to move robot
 */

class Simulator {
    constructor() {
        this.canvas = document.getElementById('simulatorCanvas');
        this.ctx = this.canvas.getContext('2d');
        
        // Resize canvas to fit container
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
        
        // Initialize modules
        this.grid = new Grid(this.canvas, {
            width: this.canvas.width,
            height: this.canvas.height,
            cellSize: 20,
            scale: 2
        });
        
        // Set origin to bottom center
        this.grid.setOrigin(this.canvas.width / 2, this.canvas.height - 50);
        
        this.robot = new Robot(this.grid, {
            outerRadius: 19,
            innerRadius: 14.4,
            x: 0,
            y: 0,  // Start at origin (bottom center)
            theta: Math.PI / 2  // Facing up
        });
        
        this.obstacleManager = new ObstacleManager(this.grid);
        this.pathFinder = new PathFinder(this.grid, this.obstacleManager);
        this.pathRenderer = new PathRenderer(this.grid);
        
        // Initialize kinematics
        this.kinematics = new Kinematics({
            wheelDistance: 14.4,  // cm
            wheelRadius: 4.1      // cm - 82mm diameter
        });
        
        // Simulation state
        this.goalPos = null;
        this.currentPath = [];
        this.isAnimating = false;
        this.animationFrame = null;
        this.pathProgress = 0;
        
        // Obstacle drawing state
        this.obstacleMode = null;  // null = goal mode, or 'circle'/'rectangle'/'parallelogram'
        this.isDrawingObstacle = false;
        this.drawStartPos = null;  // World coords where drag started
        this.drawCurrentPos = null;  // Current mouse position during drag
        
        // Animation settings
        this.animationSpeed = 30;  // cm per second (0.3 m/s default - slower for observation)
        this.maxOmega = 0.5;  // rad/s - max turn speed (slow for observation)
        this.lastTimestamp = 0;
        this.smoothPath = [];  // Bezier curve path
        
        // Initialize
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
        // Canvas mouse events for drag-to-draw obstacles
        this.canvas.addEventListener('mousedown', (e) => this.handleMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        this.canvas.addEventListener('mouseup', (e) => this.handleMouseUp(e));
        this.canvas.addEventListener('mouseleave', (e) => this.handleMouseLeave(e));
        this.canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            this.handleRightClick(e);
        });
        
        // Obstacle tool buttons - click to enter draw mode
        document.querySelectorAll('.tool-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const tool = e.target.dataset.tool;
                
                // Select obstacle tool
                document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
                e.target.classList.add('active');
                this.obstacleMode = tool;
                this.canvas.style.cursor = 'crosshair';
                this.log(`Draw ${tool}: Click and drag on map`, 'info');
            });
        });
        
        // Action buttons
        document.getElementById('stopSim').addEventListener('click', () => this.stopAnimation());
        document.getElementById('resetSim').addEventListener('click', () => this.reset());
        document.getElementById('clearObstacles').addEventListener('click', () => {
            this.obstacleManager.clear();
            this.render();
            this.log('Obstacles cleared', 'info');
        });
        
        // Configuration inputs
        document.getElementById('outerRadius').addEventListener('change', (e) => {
            this.robot.setConfig({ outerRadius: parseFloat(e.target.value) });
            this.render();
        });
        document.getElementById('innerRadius').addEventListener('change', (e) => {
            this.robot.setConfig({ innerRadius: parseFloat(e.target.value) });
            this.render();
        });
        document.getElementById('maxVelocity').addEventListener('change', (e) => {
            this.animationSpeed = parseFloat(e.target.value) * 100;  // m/s to cm/s
        });
        document.getElementById('maxOmega').addEventListener('change', (e) => {
            this.maxOmega = parseFloat(e.target.value);  // rad/s
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
        
        // Kinematics parameters
        document.getElementById('wheelDistance').addEventListener('change', (e) => {
            this.kinematics.setConfig({ wheelDistance: parseFloat(e.target.value) });
        });
        document.getElementById('wheelRadius').addEventListener('change', (e) => {
            this.kinematics.setConfig({ wheelRadius: parseFloat(e.target.value) });
        });
    }
    
    // Clear obstacle mode and return to goal mode
    clearObstacleMode() {
        this.obstacleMode = null;
        this.isDrawingObstacle = false;
        this.drawStartPos = null;
        this.drawCurrentPos = null;
        document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
        this.canvas.style.cursor = 'crosshair';
    }
    
    handleMouseDown(event) {
        if (event.button !== 0) return;  // Only left click
        
        const rect = this.canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const world = this.grid.canvasToWorld(px, py);
        
        if (this.obstacleMode) {
            // Start drawing obstacle
            this.isDrawingObstacle = true;
            this.drawStartPos = { x: world.x, y: world.y };
            this.drawCurrentPos = { x: world.x, y: world.y };
        }
    }
    
    handleMouseUp(event) {
        if (event.button !== 0) return;  // Only left click
        
        const rect = this.canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const world = this.grid.canvasToWorld(px, py);
        
        if (this.isDrawingObstacle && this.drawStartPos) {
            // Finish drawing obstacle
            const dx = Math.abs(world.x - this.drawStartPos.x);
            const dy = Math.abs(world.y - this.drawStartPos.y);
            const size = Math.max(dx, dy, 10);  // Minimum size 10cm
            
            // Center of obstacle
            const cx = (this.drawStartPos.x + world.x) / 2;
            const cy = (this.drawStartPos.y + world.y) / 2;
            
            // Add obstacle with calculated size
            this.addObstacleWithSize(cx, cy, this.obstacleMode, dx, dy);
            
            this.log(`Created ${this.obstacleMode} at (${cx.toFixed(0)}, ${cy.toFixed(0)})`, 'success');
            
            // Return to goal mode after drawing
            this.clearObstacleMode();
            this.log('Back to move mode. Click to move robot.', 'info');
        } else if (!this.obstacleMode) {
            // No obstacle mode = set goal and move robot
            this.moveRobotTo(world.x, world.y);
        }
        
        this.isDrawingObstacle = false;
        this.drawStartPos = null;
        this.drawCurrentPos = null;
        this.render();
    }
    
    handleMouseLeave(event) {
        // Cancel drawing if mouse leaves canvas
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
                const radius = Math.max(width, height) / 2;
                obstacle = new CircleObstacle(cx, cy, radius);
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

    handleCanvasClick(event) {
        // This is now handled by mouseup
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
        
        document.getElementById('mousePos').textContent = 
            `Mouse: (${world.x.toFixed(0)}, ${world.y.toFixed(0)}) cm`;
        
        // Update preview while drawing
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
            case 'circle':
                const radius = Math.max(width, height) / 2;
                ctx.beginPath();
                ctx.arc(cx, cy, radius, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();
                break;
            case 'rectangle':
                ctx.beginPath();
                ctx.rect(cx - width/2, cy - height/2, width, height);
                ctx.fill();
                ctx.stroke();
                break;
            case 'parallelogram':
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
        
        ctx.restore();
        
        // Show size info
        const worldWidth = Math.abs(this.drawCurrentPos.x - this.drawStartPos.x);
        const worldHeight = Math.abs(this.drawCurrentPos.y - this.drawStartPos.y);
        ctx.fillStyle = '#ffff00';
        ctx.font = '12px Arial';
        ctx.fillText(`${worldWidth.toFixed(0)} x ${worldHeight.toFixed(0)} cm`, current.x + 10, current.y - 10);
    }
    
    moveRobotTo(targetX, targetY) {
        // Stop any current animation
        this.stopAnimation();
        
        this.goalPos = { x: targetX, y: targetY };
        
        // Calculate and display kinematics
        this.updateKinematics(targetX, targetY);
        
        // Find path
        const startPos = { x: this.robot.x, y: this.robot.y };
        const path = this.pathFinder.findPath(
            startPos,
            this.goalPos,
            this.robot.getBoundingRadius()
        );
        
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
        // Calculate kinematics from current position to target (with rotation)
        const result = this.kinematics.calculateFromTarget(
            this.robot.x,
            this.robot.y,
            this.robot.theta,
            targetX,
            targetY,
            this.animationSpeed,  // speed in cm/s
            true  // face movement direction
        );
        
        // Update UI
        document.getElementById('kinAngle').textContent = result.angleDegrees.toFixed(1) + '°';
        document.getElementById('kinDistance').textContent = result.distance.toFixed(1) + ' cm';
        document.getElementById('kinVx').textContent = result.Vx_robot.toFixed(2) + ' cm/s';
        document.getElementById('kinVy').textContent = result.Vy_robot.toFixed(2) + ' cm/s';
        
        const wa = result.wheelAngularVelocities;
        document.getElementById('kinOmegaLeft').textContent = wa.omega1_left.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaBack').textContent = wa.omega2_back.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaRight').textContent = wa.omega3_right.toFixed(3) + ' rad/s';
        
        // Store for display
        this.currentKinematics = result;
        
        return result;
    }
    
    updateKinematicsDisplay() {
        // Update kinematics during animation based on current segment
        if (!this.isAnimating || this.currentPath.length < 2) return;
        
        // Find next waypoint
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
        
        // Calculate kinematics to next waypoint (with rotation at turns)
        const result = this.kinematics.calculateFromTarget(
            this.robot.x,
            this.robot.y,
            this.robot.theta,
            nextPoint.x,
            nextPoint.y,
            this.animationSpeed,
            true  // face movement direction
        );
        
        // Update UI
        document.getElementById('kinAngle').textContent = result.angleDegrees.toFixed(1) + '°';
        document.getElementById('kinDistance').textContent = (totalDist - this.pathProgress).toFixed(1) + ' cm';
        document.getElementById('kinVx').textContent = result.Vx_robot.toFixed(2) + ' cm/s';
        document.getElementById('kinVy').textContent = result.Vy_robot.toFixed(2) + ' cm/s';
        
        const wa = result.wheelAngularVelocities;
        document.getElementById('kinOmegaLeft').textContent = wa.omega1_left.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaBack').textContent = wa.omega2_back.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaRight').textContent = wa.omega3_right.toFixed(3) + ' rad/s';
        
        // Store current omega for robot rotation
        this.currentOmega = result.omega;
        
        // Update robot wheel visualization
        this.robot.setWheelOmega(wa.omega1_left, wa.omega2_back, wa.omega3_right);
    }
    
    /**
     * Calculate realtime kinematics based on current position, heading, and movement
     * This is called every animation frame
     */
    updateKinematicsRealtimeDisplay() {
        if (!this.isAnimating || this.currentPath.length < 2) return;
        
        // Find next waypoint
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
        let currentSegment = 0;
        for (let i = 0; i < segmentLengths.length; i++) {
            if (accumulated + segmentLengths[i] >= this.pathProgress) {
                nextPoint = this.currentPath[i + 1];
                currentSegment = i;
                break;
            }
            accumulated += segmentLengths[i];
        }
        
        // Current robot state
        const robotX = this.robot.x;
        const robotY = this.robot.y;
        const robotTheta = this.robot.theta;  // Current heading in radians
        
        // Calculate velocity in world frame
        const dx = nextPoint.x - robotX;
        const dy = nextPoint.y - robotY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        const remainingDist = totalDist - this.pathProgress;
        
        // Direction to next waypoint (world frame)
        const targetAngle = Math.atan2(dy, dx);
        
        // Set kinematics robot heading for coordinate transform
        this.kinematics.setRobotHeading(robotTheta);
        
        // World frame velocities (moving toward next waypoint)
        const Vx_world = (distance > 0.1) ? (dx / distance) * this.animationSpeed : 0;
        const Vy_world = (distance > 0.1) ? (dy / distance) * this.animationSpeed : 0;
        
        // Calculate angular velocity for rotation (smooth turn to face direction)
        let omega = 0;
        if (distance > 5) {  // Only rotate if far enough from waypoint
            const angleDiff = this.normalizeAngle(targetAngle - robotTheta);
            const kp = 0.8;  // Proportional gain for rotation (reduced for slower turning)
            omega = Math.max(-this.maxOmega, Math.min(this.maxOmega, kp * angleDiff));
        }
        
        // Convert world velocity to robot frame
        const robotVel = this.kinematics.worldToMobile(Vx_world, Vy_world);
        
        // Calculate inverse kinematics (wheel linear velocities)
        const wheelVel = this.kinematics.inverseMobile(robotVel.Vxm, robotVel.Vym, omega);
        
        // Calculate wheel angular velocities (pass the wheel velocities object)
        const wheelOmega = this.kinematics.getWheelAngularVelocities(wheelVel);
        
        // Update UI with realtime values
        document.getElementById('kinAngle').textContent = (targetAngle * 180 / Math.PI).toFixed(1) + '°';
        document.getElementById('kinDistance').textContent = remainingDist.toFixed(1) + ' cm';
        document.getElementById('kinVx').textContent = robotVel.Vxm.toFixed(2) + ' cm/s';
        document.getElementById('kinVy').textContent = robotVel.Vym.toFixed(2) + ' cm/s';
        document.getElementById('kinOmegaRobot').textContent = omega.toFixed(3) + ' rad/s';
        
        document.getElementById('kinOmegaLeft').textContent = wheelOmega.omega1_left.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaBack').textContent = wheelOmega.omega2_back.toFixed(3) + ' rad/s';
        document.getElementById('kinOmegaRight').textContent = wheelOmega.omega3_right.toFixed(3) + ' rad/s';
        
        // Update robot wheel omega for visualization (show rotation direction)
        this.robot.setWheelOmega(wheelOmega.omega1_left, wheelOmega.omega2_back, wheelOmega.omega3_right);
        
        // Store angular velocity
        this.currentOmega = omega;
        this.robot.omega = omega;
    }
    
    /**
     * Normalize angle to [-π, π]
     */
    normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    startAnimation() {
        if (this.currentPath.length < 2) return;
        if (this.isAnimating) return;
        
        this.isAnimating = true;
        this.pathProgress = 0;
        this.lastTimestamp = performance.now();
        
        // Generate smooth Bezier curve from path waypoints
        this.smoothPath = this.generateSmoothPath(this.currentPath);
        this.smoothPathLength = this.calculateSmoothPathLength();
        
        this.animate();
    }
    
    /**
     * Generate smooth Bezier curve path from waypoints
     * This creates a continuous curve that passes through all waypoints
     */
    generateSmoothPath(waypoints) {
        if (waypoints.length < 2) return waypoints;
        
        const smoothPoints = [];
        const tension = 0.3;  // Control curve tightness (0 = sharp, 1 = very smooth)
        const segments = 20;  // Points per segment for smooth curve
        
        for (let i = 0; i < waypoints.length - 1; i++) {
            const p0 = waypoints[Math.max(0, i - 1)];
            const p1 = waypoints[i];
            const p2 = waypoints[i + 1];
            const p3 = waypoints[Math.min(waypoints.length - 1, i + 2)];
            
            // Generate Catmull-Rom spline points (converted to Bezier)
            for (let j = 0; j < segments; j++) {
                const t = j / segments;
                const point = this.catmullRom(p0, p1, p2, p3, t, tension);
                smoothPoints.push(point);
            }
        }
        
        // Add final point
        smoothPoints.push(waypoints[waypoints.length - 1]);
        
        return smoothPoints;
    }
    
    /**
     * Catmull-Rom spline interpolation
     */
    catmullRom(p0, p1, p2, p3, t, tension = 0.5) {
        const t2 = t * t;
        const t3 = t2 * t;
        
        const alpha = tension;
        
        const x = alpha * (
            (-p0.x + 3*p1.x - 3*p2.x + p3.x) * t3 +
            (2*p0.x - 5*p1.x + 4*p2.x - p3.x) * t2 +
            (-p0.x + p2.x) * t +
            2*p1.x
        ) / 2;
        
        const y = alpha * (
            (-p0.y + 3*p1.y - 3*p2.y + p3.y) * t3 +
            (2*p0.y - 5*p1.y + 4*p2.y - p3.y) * t2 +
            (-p0.y + p2.y) * t +
            2*p1.y
        ) / 2;
        
        // Simplified Catmull-Rom
        const x2 = 0.5 * (
            (2 * p1.x) +
            (-p0.x + p2.x) * t +
            (2*p0.x - 5*p1.x + 4*p2.x - p3.x) * t2 +
            (-p0.x + 3*p1.x - 3*p2.x + p3.x) * t3
        );
        
        const y2 = 0.5 * (
            (2 * p1.y) +
            (-p0.y + p2.y) * t +
            (2*p0.y - 5*p1.y + 4*p2.y - p3.y) * t2 +
            (-p0.y + 3*p1.y - 3*p2.y + p3.y) * t3
        );
        
        return { x: x2, y: y2 };
    }
    
    /**
     * Calculate total length of smooth path
     */
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
        
        // Use smooth path for animation
        const path = this.smoothPath || this.currentPath;
        
        // Calculate total path length
        let totalLength = 0;
        const segmentLengths = [];
        
        for (let i = 1; i < path.length; i++) {
            const dx = path[i].x - path[i - 1].x;
            const dy = path[i].y - path[i - 1].y;
            const len = Math.sqrt(dx * dx + dy * dy);
            segmentLengths.push(len);
            totalLength += len;
        }
        
        // Update progress
        this.pathProgress += this.animationSpeed * dt;
        
        if (this.pathProgress >= totalLength) {
            // Reached goal
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
        
        // Find current segment and position on smooth path
        let accumulated = 0;
        for (let i = 0; i < segmentLengths.length; i++) {
            if (accumulated + segmentLengths[i] >= this.pathProgress) {
                const t = (this.pathProgress - accumulated) / segmentLengths[i];
                const p1 = path[i];
                const p2 = path[i + 1];
                
                const x = p1.x + (p2.x - p1.x) * t;
                const y = p1.y + (p2.y - p1.y) * t;
                
                // Calculate tangent direction for smooth heading
                const tangentAngle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
                
                // Smoothly interpolate robot heading toward tangent direction
                const targetTheta = tangentAngle;
                const currentTheta = this.robot.theta;
                let angleDiff = this.normalizeAngle(targetTheta - currentTheta);
                
                // Smooth rotation with limited angular velocity
                const maxRotation = this.maxOmega * dt;
                if (Math.abs(angleDiff) > maxRotation) {
                    angleDiff = Math.sign(angleDiff) * maxRotation;
                }
                const newTheta = currentTheta + angleDiff;
                
                this.robot.setPosition(x, y, newTheta);
                
                // Calculate actual angular velocity
                this.robot.omega = angleDiff / dt;
                
                // Set velocity for vector display
                const speed = this.animationSpeed;
                this.robot.vx = (p2.x - p1.x) / segmentLengths[i] * speed;
                this.robot.vy = (p2.y - p1.y) / segmentLengths[i] * speed;
                break;
            }
            accumulated += segmentLengths[i];
        }
        
        // Update kinematics display during animation (realtime calculation)
        this.updateKinematicsRealtimeDisplay();
        
        // Draw smooth path
        this.renderSmoothPath();
        
        this.render();
        this.animationFrame = requestAnimationFrame((t) => this.animate(t));
    }
    
    /**
     * Render the smooth Bezier path
     */
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
    
    /**
     * Update display when robot stops
     */
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
        this.robot.setPosition(0, 0, Math.PI / 2);  // Reset to origin, facing up
        this.robot.vx = 0;
        this.robot.vy = 0;
        this.robot.omega = 0;
        this.robot.setWheelOmega(0, 0, 0);
        
        // Reset kinematics display
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
        // Draw grid
        this.grid.draw();
        
        // Draw obstacles
        this.obstacleManager.draw();
        
        // Draw original path (waypoints)
        this.pathRenderer.draw();
        
        // Draw smooth Bezier path on top
        if (this.smoothPath && this.smoothPath.length > 1) {
            this.renderSmoothPath();
        }
        
        // Draw goal marker
        if (this.goalPos) {
            this.drawGoalMarker();
        }
        
        // Draw robot
        this.robot.draw(true);
        
        // Draw velocity vector
        if (this.isAnimating) {
            this.robot.drawVelocityVector();
        }
        
        // Update robot position display
        document.getElementById('robotPos').textContent = 
            `Robot: (${this.robot.x.toFixed(0)}, ${this.robot.y.toFixed(0)}) θ=${(this.robot.theta * 180 / Math.PI).toFixed(0)}°`;
    }
    
    drawGoalMarker() {
        const pos = this.grid.worldToCanvas(this.goalPos.x, this.goalPos.y);
        const ctx = this.ctx;
        
        // Target circles
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
        
        // Keep only last 8 messages
        while (logDiv.children.length > 8) {
            logDiv.removeChild(logDiv.lastChild);
        }
    }
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.simulator = new Simulator();
});
