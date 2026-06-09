/**
 * Robot Module - Handles robot rendering and state
 */
class Robot {
    constructor(grid, config = {}) {
        this.grid = grid;
        this.ctx = grid.ctx;
        
        this.outerRadius = config.outerRadius || 19;
        this.innerRadius = config.innerRadius || 14.4;
        this.safetyMargin = config.safetyMargin || 2;
        this.wheelRadius = config.wheelRadius || 4.1;
        this.wheelWidth = config.wheelWidth || 2.5;
        this.x = config.x || 0;
        this.y = config.y || 0;
        this.theta = config.theta || 0;
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;
        this.wheelOmega = {
            omega1: 0,
            omega2: 0,
            omega3: 0
        };
        this.maxVelocity = config.maxVelocity || 100;
        this.maxOmega = config.maxOmega || 2.0;
        this.bodyColor = '#3498db';
        this.outlineColor = '#2980b9';
        this.wheelColor = '#2c3e50';
        this.wheelRollerColor = '#1a1a2a';
        this.directionColor = '#e74c3c';
        this.safetyColor = 'rgba(255, 255, 0, 0.15)';
        this.wheelAngles = {
            v1: 150 * Math.PI / 180,
            v2: 270 * Math.PI / 180,
            v3: 30 * Math.PI / 180
        };
    }
    
    setPosition(x, y, theta = null) {
        this.x = x;
        this.y = y;
        if (theta !== null) {
            this.theta = theta;
        }
    }
    
    setWheelOmega(omega1, omega2, omega3) {
        this.wheelOmega.omega1 = omega1;
        this.wheelOmega.omega2 = omega2;
        this.wheelOmega.omega3 = omega3;
    }
    
    setConfig(config) {
        if (config.outerRadius !== undefined) this.outerRadius = config.outerRadius;
        if (config.innerRadius !== undefined) this.innerRadius = config.innerRadius;
        if (config.safetyMargin !== undefined) this.safetyMargin = config.safetyMargin;
        if (config.maxVelocity !== undefined) this.maxVelocity = config.maxVelocity;
        if (config.maxOmega !== undefined) this.maxOmega = config.maxOmega;
        if (config.wheelRadius !== undefined) this.wheelRadius = config.wheelRadius;
    }
    
    draw(showSafety = true) {
        const ctx = this.ctx;
        const pos = this.grid.worldToCanvas(this.x, this.y);
        const scale = this.grid.scale;
        
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-this.theta + Math.PI/2);
        
        if (showSafety) {
            ctx.beginPath();
            ctx.arc(0, 0, (this.outerRadius + this.safetyMargin) * scale, 0, Math.PI * 2);
            ctx.fillStyle = this.safetyColor;
            ctx.fill();
        }
        
        this.drawHexagonBody(scale);
        this.drawWheel(this.wheelAngles.v1, scale, 'V1', this.wheelOmega.omega1);
        this.drawWheel(this.wheelAngles.v2, scale, 'V2', this.wheelOmega.omega2);
        this.drawWheel(this.wheelAngles.v3, scale, 'V3', this.wheelOmega.omega3);
        
        ctx.beginPath();
        ctx.arc(0, 0, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#fff';
        ctx.fill();
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 1;
        ctx.stroke();
        
        this.drawRobotAxes(scale);
        this.drawDirectionArrow(scale);
        
        ctx.restore();
    }
    
    drawHexagonBody(scale) {
        const ctx = this.ctx;
        const rInner = this.innerRadius * scale;
        const vertices = [];
        
        for (let i = 0; i < 3; i++) {
            const mainAngle = (Math.PI / 2) + (i * 2 * Math.PI / 3);
            const cutAngle = Math.PI / 6;
            vertices.push({
                x: rInner * Math.cos(mainAngle - cutAngle),
                y: rInner * Math.sin(mainAngle - cutAngle)
            });
            vertices.push({
                x: rInner * Math.cos(mainAngle + cutAngle),
                y: rInner * Math.sin(mainAngle + cutAngle)
            });
        }
        
        ctx.beginPath();
        ctx.moveTo(vertices[0].x, -vertices[0].y);
        for (let i = 1; i < vertices.length; i++) {
            ctx.lineTo(vertices[i].x, -vertices[i].y);
        }
        ctx.closePath();
        ctx.fillStyle = this.bodyColor;
        ctx.fill();
        ctx.strokeStyle = this.outlineColor;
        ctx.lineWidth = 3;
        ctx.stroke();
    }
    
    drawWheel(angle, scale, label, omega) {
        const ctx = this.ctx;
        const L = this.innerRadius * scale;
        const wheelR = this.wheelRadius * scale;
        const wheelW = this.wheelWidth * scale;
        const wx = L * Math.cos(angle);
        const wy = -L * Math.sin(angle);
        
        ctx.save();
        ctx.translate(wx, wy);
        ctx.rotate(-angle + Math.PI/2);
        
        ctx.fillStyle = this.wheelColor;
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 1;
        const wheelLength = wheelR * 2;
        ctx.fillRect(-wheelLength/2, -wheelW/2, wheelLength, wheelW);
        ctx.strokeRect(-wheelLength/2, -wheelW/2, wheelLength, wheelW);
        
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
        
        const omegaColor = omega >= 0 ? '#00ff00' : '#ff0000';
        const omegaLength = Math.min(Math.abs(omega) * 3, 20) * scale / 2;
        if (Math.abs(omega) > 0.01) {
            ctx.strokeStyle = omegaColor;
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, -wheelW/2 - 2);
            ctx.lineTo(omega > 0 ? omegaLength : -omegaLength, -wheelW/2 - 2);
            ctx.stroke();
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
        
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 10px Arial';
        ctx.textAlign = 'center';
        const labelDist = L + wheelR + 8;
        const lx = labelDist * Math.cos(angle);
        const ly = -labelDist * Math.sin(angle);
        ctx.fillText(label, lx, ly + 3);
    }
    
    drawRobotAxes(scale) {
        const ctx = this.ctx;
        const axisLen = this.innerRadius * 0.5 * scale;
        ctx.strokeStyle = '#ff6666';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(axisLen, 0);
        ctx.stroke();
        ctx.fillStyle = '#ff6666';
        ctx.font = '10px Arial';
        ctx.fillText('Xm', axisLen + 5, 3);
        
        ctx.strokeStyle = '#66ff66';
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(0, -axisLen);
        ctx.stroke();
        ctx.fillStyle = '#66ff66';
        ctx.fillText('Ym', 3, -axisLen - 3);
    }
    
    drawDirectionArrow(scale) {
        const ctx = this.ctx;
        const arrowLength = this.innerRadius * 0.7 * scale;
        const arrowWidth = 8;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(0, -arrowLength);
        ctx.strokeStyle = this.directionColor;
        ctx.lineWidth = 4;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(0, -arrowLength - 5);
        ctx.lineTo(-arrowWidth, -arrowLength + 5);
        ctx.lineTo(arrowWidth, -arrowLength + 5);
        ctx.closePath();
        ctx.fillStyle = this.directionColor;
        ctx.fill();
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 9px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('FRONT', 0, -arrowLength - 12);
    }
    
    drawVelocityVector() {
        if (Math.abs(this.vx) < 0.01 && Math.abs(this.vy) < 0.01) return;
        const ctx = this.ctx;
        const pos = this.grid.worldToCanvas(this.x, this.y);
        const scale = this.grid.scale;
        const velScale = 0.3;
        const vxPx = this.vx * velScale * scale;
        const vyPx = -this.vy * velScale * scale;
        
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(vxPx, vyPx);
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 3;
        ctx.stroke();
        
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
    
    update(dt) {
        this.x += this.vx * dt;
        this.y += this.vy * dt;
        this.theta += this.omega * dt;
        while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
        while (this.theta < -Math.PI) this.theta += 2 * Math.PI;
    }
    
    getBoundingRadius() {
        return this.outerRadius + this.safetyMargin;
    }
}

window.Robot = Robot;
