/**
 * Kinematics Module - Inverse kinematics for 3-wheel omni robot
 */

class Kinematics {
    constructor(config = {}) {
        this.L = config.wheelDistance || 14.4;
        this.wheelRadius = config.wheelRadius || 4.1;
        this.sqrt3 = Math.sqrt(3);
        this.sqrt3_2 = this.sqrt3 / 2;
        this.wheelAngles = {
            left: 150 * Math.PI / 180,
            back: 270 * Math.PI / 180,
            right: 30 * Math.PI / 180
        };
        this.robotHeading = 0;
    }
    
    setConfig(config) {
        if (config.wheelDistance !== undefined) this.L = config.wheelDistance;
        if (config.wheelRadius !== undefined) this.wheelRadius = config.wheelRadius;
    }
    
    setRobotHeading(theta) {
        this.robotHeading = theta;
    }
    
    worldToMobile(Vxw, Vyw) {
        const cos_t = Math.cos(this.robotHeading);
        const sin_t = Math.sin(this.robotHeading);
        return {
            Vxm:  cos_t * Vxw + sin_t * Vyw,
            Vym: -sin_t * Vxw + cos_t * Vyw
        };
    }
    
    mobileToWorld(Vxm, Vym) {
        const cos_t = Math.cos(this.robotHeading);
        const sin_t = Math.sin(this.robotHeading);
        return {
            Vxw: cos_t * Vxm - sin_t * Vym,
            Vyw: sin_t * Vxm + cos_t * Vym
        };
    }
    
    inverseMobile(Vx, Vy, omega) {
        const L_omega = this.L * omega;
        return {
            v1_left:  -0.5 * Vx - this.sqrt3_2 * Vy + L_omega,
            v2_back:   1.0 * Vx + 0 * Vy             + L_omega,
            v3_right: -0.5 * Vx + this.sqrt3_2 * Vy + L_omega
        };
    }
    
    inverseWorld(Vxw, Vyw, omega) {
        const mobile = this.worldToMobile(Vxw, Vyw);
        return this.inverseMobile(mobile.Vxm, mobile.Vym, omega);
    }
    
    getWheelAngularVelocities(arg1, arg2, arg3) {
        let wheelVelocities;
        
        if (arg2 !== undefined && arg3 !== undefined) {
            wheelVelocities = this.inverseMobile(arg1, arg2, arg3);
        } else if (arg1 && typeof arg1 === 'object') {
            wheelVelocities = arg1;
        } else {
            return {
                omega1_left: 0,
                omega2_back: 0,
                omega3_right: 0
            };
        }
        
        const safeDiv = (v) => {
            if (isNaN(v) || !isFinite(v)) return 0;
            return v / this.wheelRadius;
        };
        
        return {
            omega1_left:  safeDiv(wheelVelocities.v1_left),
            omega2_back:  safeDiv(wheelVelocities.v2_back),
            omega3_right: safeDiv(wheelVelocities.v3_right)
        };
    }
    
    forwardMobile(v1, v2, v3) {
        return {
            Vx:    (2 * v2 - v1 - v3) / 3,
            Vy:    (this.sqrt3 * v3 - this.sqrt3 * v1) / 3,
            omega: (v1 + v2 + v3) / (3 * this.L)
        };
    }
    
    calculateTurnOmega(currentHeading, targetHeading, maxOmega = 0.8, dt = 0.1) {
        let deltaTheta = targetHeading - currentHeading;
        while (deltaTheta > Math.PI) deltaTheta -= 2 * Math.PI;
        while (deltaTheta < -Math.PI) deltaTheta += 2 * Math.PI;
        const Kp = 1.0;
        let omega = Kp * deltaTheta;
        omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
        return omega;
    }
    
    calculateFromTarget(currentX, currentY, currentTheta, targetX, targetY, speed, faceMovementDirection = true) {
        const dx = targetX - currentX;
        const dy = targetY - currentY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        if (distance < 0.1) {
            return {
                angle: currentTheta,
                angleDegrees: currentTheta * 180 / Math.PI,
                distance: 0,
                Vx: 0,
                Vy: 0,
                omega: 0,
                wheelLinearVelocities: { v1_left: 0, v2_back: 0, v3_right: 0 },
                wheelAngularVelocities: { omega1_left: 0, omega2_back: 0, omega3_right: 0 }
            };
        }
        
        const angleToTarget = Math.atan2(dy, dx);
        let omega = 0;
        if (faceMovementDirection) {
            const targetHeading = angleToTarget - Math.PI / 2;
            omega = this.calculateTurnOmega(currentTheta, targetHeading);
        }
        
        const Vxw = speed * Math.cos(angleToTarget);
        const Vyw = speed * Math.sin(angleToTarget);
        this.setRobotHeading(currentTheta);
        const mobile = this.worldToMobile(Vxw, Vyw);
        const wheelLinear = this.inverseMobile(mobile.Vxm, mobile.Vym, omega);
        const wheelAngular = this.getWheelAngularVelocities(wheelLinear);
        
        return {
            angle: angleToTarget,
            angleDegrees: angleToTarget * 180 / Math.PI,
            distance: distance,
            Vx_world: Vxw,
            Vy_world: Vyw,
            Vx_robot: mobile.Vxm,
            Vy_robot: mobile.Vym,
            omega: omega,
            omegaDegrees: omega * 180 / Math.PI,
            wheelLinearVelocities: wheelLinear,
            wheelAngularVelocities: wheelAngular
        };
    }
    
    calculateForPathSegment(currentX, currentY, currentTheta, nextX, nextY, speed) {
        return this.calculateFromTarget(currentX, currentY, currentTheta, nextX, nextY, speed, true);
    }
}

window.Kinematics = Kinematics;
