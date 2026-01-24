/**
 * Robot Module - Handles robot rendering and state
 * 
 * Robot shape based on diagram:
 *        Front (+Y) - Vym
 *           ↑
 *      V1 ╱   ╲ V3
 *  (150°)     (30°)
 *         ╲   ╱
 *          ─●─ → Vxm
 *           │
 *          V2 (270°)
 *         Back
 * 
 * - Irregular hexagon (truncated equilateral triangle)
 * - Outer radius: 19cm (distance from center to farthest point)
 * - Inner radius: 14.4cm (distance from center to flat edge)
 * - Wheel diameter: 82mm = 8.2cm, radius = 4.1cm
 * - L (wheel distance from center): 14.4cm
 */
class Robot {
    constructor(grid, config = {}) {
        this.grid = grid;
        this.ctx = grid.ctx;
        
        // Robot dimensions (in cm)
        this.outerRadius = config.outerRadius || 19;    // To vertices
        this.innerRadius = config.innerRadius || 14.4;  // To flat edges / wheel distance L
        this.safetyMargin = config.safetyMargin || 2;
        
        // Wheel parameters
        this.wheelRadius = config.wheelRadius || 4.1;   // 82mm diameter
        this.wheelWidth = config.wheelWidth || 2.5;     // Visual width
        
        // Robot state (in world coordinates - cm)
        this.x = config.x || 0;
        this.y = config.y || 0;
        this.theta = config.theta || 0;  // Heading in radians (0 = facing +X, π/2 = facing +Y)
        
        // Velocity state
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;  // Angular velocity of robot
        
        // Wheel angular velocities (rad/s)
        this.wheelOmega = {
            omega1: 0,  // V1 - Left (150°)
            omega2: 0,  // V2 - Back (270°)
            omega3: 0   // V3 - Right (30°)
        };
        
        // Limits
        this.maxVelocity = config.maxVelocity || 100;  // cm/s
        this.maxOmega = config.maxOmega || 2.0;        // rad/s
        
        // Colors
        this.bodyColor = '#3498db';
        this.outlineColor = '#2980b9';
        this.wheelColor = '#2c3e50';
        this.wheelRollerColor = '#1a1a2a';
        this.directionColor = '#e74c3c';
        this.safetyColor = 'rgba(255, 255, 0, 0.15)';
        
        // Wheel positions (angles from robot +X axis)
        this.wheelAngles = {
            v1: 150 * Math.PI / 180,  // Left wheel
            v2: 270 * Math.PI / 180,  // Back wheel  
            v3: 30 * Math.PI / 180    // Right wheel
        };
    }
    
    /**
     * Set robot position
     */
    setPosition(x, y, theta = null) {
        this.x = x;
        this.y = y;
        if (theta !== null) {
            this.theta = theta;
        }
    }
    
    /**
     * Set wheel angular velocities for display
     */
    setWheelOmega(omega1, omega2, omega3) {
        this.wheelOmega.omega1 = omega1;
        this.wheelOmega.omega2 = omega2;
        this.wheelOmega.omega3 = omega3;
    }
    
    /**
     * Set robot configuration
     */
    setConfig(config) {
        if (config.outerRadius !== undefined) this.outerRadius = config.outerRadius;
        if (config.innerRadius !== undefined) this.innerRadius = config.innerRadius;
        if (config.safetyMargin !== undefined) this.safetyMargin = config.safetyMargin;
        if (config.maxVelocity !== undefined) this.maxVelocity = config.maxVelocity;
        if (config.maxOmega !== undefined) this.maxOmega = config.maxOmega;
        if (config.wheelRadius !== undefined) this.wheelRadius = config.wheelRadius;
    }
    
    /**
     * Draw the robot on the canvas
     */
    draw(showSafety = true) {
        const ctx = this.ctx;
        const pos = this.grid.worldToCanvas(this.x, this.y);
        const scale = this.grid.scale;
        
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-this.theta + Math.PI/2);  // Adjust so +Y is forward when theta=0
        
        // Draw safety margin circle
        if (showSafety) {
            ctx.beginPath();
            ctx.arc(0, 0, (this.outerRadius + this.safetyMargin) * scale, 0, Math.PI * 2);
            ctx.fillStyle = this.safetyColor;
            ctx.fill();
        }
        
        // Draw robot body (hexagon shape)
        this.drawHexagonBody(scale);
        
        // Draw wheels at correct positions
        this.drawWheel(this.wheelAngles.v1, scale, 'V1', this.wheelOmega.omega1);
        this.drawWheel(this.wheelAngles.v2, scale, 'V2', this.wheelOmega.omega2);
        this.drawWheel(this.wheelAngles.v3, scale, 'V3', this.wheelOmega.omega3);
        
        // Draw center point
        ctx.beginPath();
        ctx.arc(0, 0, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#fff';
        ctx.fill();
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 1;
        ctx.stroke();
        
        // Draw coordinate axes on robot
        this.drawRobotAxes(scale);
        
        // Draw direction indicator (front arrow)
        this.drawDirectionArrow(scale);
        
        ctx.restore();
    }
    
    /**
     * Draw hexagon body based on the diagram
     */
    drawHexagonBody(scale) {
        const ctx = this.ctx;
        const r = this.outerRadius * scale;
        const rInner = this.innerRadius * scale;
        
        // Create hexagon vertices (truncated triangle pointing up)
        // Triangle vertices at 90°, 210°, 330° (pointing +Y)
        // Cut corners create 6 vertices
        const vertices = [];
        const cornerCut = 0.35;  // How much to cut the corners
        
        for (let i = 0; i < 3; i++) {
            const mainAngle = (Math.PI / 2) + (i * 2 * Math.PI / 3);
            const nextAngle = mainAngle + 2 * Math.PI / 3;
            
            // Two vertices per triangle corner (cut corner)
            const cutAngle = Math.PI / 6;  // 30 degrees for cut
            vertices.push({
                x: rInner * Math.cos(mainAngle - cutAngle),
                y: rInner * Math.sin(mainAngle - cutAngle)
            });
            vertices.push({
                x: rInner * Math.cos(mainAngle + cutAngle),
                y: rInner * Math.sin(mainAngle + cutAngle)
            });
        }
        
        // Draw body
        ctx.beginPath();
        ctx.moveTo(vertices[0].x, -vertices[0].y);  // Flip Y for canvas
        for (let i = 1; i < vertices.length; i++) {
            ctx.lineTo(vertices[i].x, -vertices[i].y);
        }
        ctx.closePath();
        
        // Fill and stroke
        ctx.fillStyle = this.bodyColor;
        ctx.fill();
        ctx.strokeStyle = this.outlineColor;
        ctx.lineWidth = 3;
        ctx.stroke();
    }
    
    /**
     * Draw a single omni wheel at given angle
     */
    drawWheel(angle, scale, label, omega) {
        const ctx = this.ctx;
        const L = this.innerRadius * scale;  // Distance from center
        const wheelR = this.wheelRadius * scale;
        const wheelW = this.wheelWidth * scale;
        
        // Wheel center position
        const wx = L * Math.cos(angle);
        const wy = -L * Math.sin(angle);  // Flip Y for canvas
        
        ctx.save();
        ctx.translate(wx, wy);
        
        // Wheel is perpendicular to radius (tangent to circle)
        // Rotate so wheel axle points toward center
        ctx.rotate(-angle + Math.PI/2);
        
        // Draw wheel body (rectangle representing omni wheel)
        ctx.fillStyle = this.wheelColor;
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 1;
        
        // Main wheel rectangle
        const wheelLength = wheelR * 2;
        ctx.fillRect(-wheelLength/2, -wheelW/2, wheelLength, wheelW);
        ctx.strokeRect(-wheelLength/2, -wheelW/2, wheelLength, wheelW);
        
        // Draw rollers on wheel
        ctx.fillStyle = this.wheelRollerColor;
        const rollerCount = 6;
        const rollerWidth = wheelLength / rollerCount;
        for (let i = 0; i < rollerCount; i++) {
            const rx = -wheelLength/2 + i * rollerWidth + rollerWidth/2;
            ctx.beginPath();
            ctx.ellipse(rx, 0, rollerWidth/3, wheelW/2 + 1, 0, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
        }
        
        // Draw omega indicator (rotation speed)
        const omegaColor = omega >= 0 ? '#00ff00' : '#ff0000';
        const omegaLength = Math.min(Math.abs(omega) * 3, 20) * scale / 2;
        
        if (Math.abs(omega) > 0.01) {
            ctx.strokeStyle = omegaColor;
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, -wheelW/2 - 2);
            ctx.lineTo(omega > 0 ? omegaLength : -omegaLength, -wheelW/2 - 2);
            ctx.stroke();
            
            // Arrow head
            const dir = omega > 0 ? 1 : -1;
            ctx.beginPath();
            ctx.moveTo(dir * omegaLength, -wheelW/2 - 2);
            ctx.lineTo(dir * omegaLength - dir * 4, -wheelW/2 - 6);
            ctx.lineTo(dir * omegaLength - dir * 4, -wheelW/2 + 2);
            ctx.closePath();
            ctx.fillStyle = omegaColor;
            ctx.fill();
        }
        
        ctx.restore();
        
        // Draw label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 10px Arial';
        ctx.textAlign = 'center';
        const labelDist = L + wheelR + 8;
        const lx = labelDist * Math.cos(angle);
        const ly = -labelDist * Math.sin(angle);
        ctx.fillText(label, lx, ly + 3);
    }
    
    /**
     * Draw robot coordinate axes
     */
    drawRobotAxes(scale) {
        const ctx = this.ctx;
        const axisLen = this.innerRadius * 0.5 * scale;
        
        // X axis (red) - right
        ctx.strokeStyle = '#ff6666';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(axisLen, 0);
        ctx.stroke();
        ctx.fillStyle = '#ff6666';
        ctx.font = '10px Arial';
        ctx.fillText('Xm', axisLen + 5, 3);
        
        // Y axis (green) - forward (up in robot frame)
        ctx.strokeStyle = '#66ff66';
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(0, -axisLen);  // Negative because canvas Y is flipped
        ctx.stroke();
        ctx.fillStyle = '#66ff66';
        ctx.fillText('Ym', 3, -axisLen - 3);
    }
    
    /**
     * Draw direction arrow (front indicator)
     */
    drawDirectionArrow(scale) {
        const ctx = this.ctx;
        const arrowLength = this.innerRadius * 0.7 * scale;
        const arrowWidth = 8;
        
        // Arrow pointing up (forward direction)
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(0, -arrowLength);
        ctx.strokeStyle = this.directionColor;
        ctx.lineWidth = 4;
        ctx.stroke();
        
        // Arrow head
        ctx.beginPath();
        ctx.moveTo(0, -arrowLength - 5);
        ctx.lineTo(-arrowWidth, -arrowLength + 5);
        ctx.lineTo(arrowWidth, -arrowLength + 5);
        ctx.closePath();
        ctx.fillStyle = this.directionColor;
        ctx.fill();
        
        // "FRONT" label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 9px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('FRONT', 0, -arrowLength - 12);
    }
    
    /**
     * Draw velocity vector
     */
    drawVelocityVector() {
        if (Math.abs(this.vx) < 0.01 && Math.abs(this.vy) < 0.01) return;
        
        const ctx = this.ctx;
        const pos = this.grid.worldToCanvas(this.x, this.y);
        const scale = this.grid.scale;
        
        // Scale velocity for visualization
        const velScale = 0.3;
        const vxPx = this.vx * velScale * scale;
        const vyPx = -this.vy * velScale * scale;  // Flip Y
        
        ctx.save();
        ctx.translate(pos.x, pos.y);
        
        // Draw velocity arrow
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(vxPx, vyPx);
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 3;
        ctx.stroke();
        
        // Arrow head
        const len = Math.sqrt(vxPx * vxPx + vyPx * vyPx);
        if (len > 10) {
            const angle = Math.atan2(vyPx, vxPx);
            ctx.save();
            ctx.translate(vxPx, vyPx);
            ctx.rotate(angle);
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(-10, -5);
            ctx.lineTo(-10, 5);
            ctx.closePath();
            ctx.fillStyle = '#00ff00';
            ctx.fill();
            ctx.restore();
        }
        
        ctx.restore();
    }
    
    /**
     * Update robot state based on velocity (for simulation)
     */
    update(dt) {
        this.x += this.vx * dt;
        this.y += this.vy * dt;
        this.theta += this.omega * dt;
        
        // Normalize theta to [-π, π]
        while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
        while (this.theta < -Math.PI) this.theta += 2 * Math.PI;
    }
    
    /**
     * Get bounding circle radius for collision detection
     */
    getBoundingRadius() {
        return this.outerRadius + this.safetyMargin;
    }
}

// Export
window.Robot = Robot;
