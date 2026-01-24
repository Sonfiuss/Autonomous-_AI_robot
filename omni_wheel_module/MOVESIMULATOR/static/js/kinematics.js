/**
 * Kinematics Module - Inverse kinematics for 3-wheel omni robot
 * 
 * Robot Configuration (based on diagram):
 *        Front (+Y)
 *           ↑
 *      V1 ╱   ╲ V3
 *   (150°)     (30°)
 *         ╲   ╱
 *          ─●─
 *           │
 *          V2 (270°/-90°)
 *         Back
 * 
 * Wheel positions from +X axis:
 *   - V1 (Left):  α1 = 150° = 5π/6
 *   - V2 (Back):  α2 = 270° = -π/2 = 3π/2  
 *   - V3 (Right): α3 = 30°  = π/6
 * 
 * Each wheel's velocity direction is tangent to the circle (perpendicular to radius)
 * 
 * Inverse Kinematics Formula:
 *   v_i = -sin(αi) * Vx + cos(αi) * Vy + L * ω
 * 
 * For our configuration:
 *   v1 = -sin(150°)*Vx + cos(150°)*Vy + L*ω = -0.5*Vx - (√3/2)*Vy + L*ω
 *   v2 = -sin(270°)*Vx + cos(270°)*Vy + L*ω = Vx + 0*Vy + L*ω = Vx + L*ω
 *   v3 = -sin(30°)*Vx  + cos(30°)*Vy  + L*ω = -0.5*Vx + (√3/2)*Vy + L*ω
 * 
 * Wheel Angular Velocity: ω_wheel = v_wheel / r_wheel (rad/s)
 */

class Kinematics {
    constructor(config = {}) {
        // Robot parameters
        this.L = config.wheelDistance || 14.4;  // cm - distance from center to wheel
        this.wheelRadius = config.wheelRadius || 4.1;  // cm - wheel radius (82mm diameter)
        
        // Pre-calculated constants
        this.sqrt3 = Math.sqrt(3);
        this.sqrt3_2 = this.sqrt3 / 2;
        
        // Wheel positions (angles from +X axis in radians)
        this.wheelAngles = {
            left: 150 * Math.PI / 180,   // V1: 150°
            back: 270 * Math.PI / 180,   // V2: 270° (-90°)
            right: 30 * Math.PI / 180    // V3: 30°
        };
        
        // Current robot heading (for world frame conversion)
        this.robotHeading = 0;  // radians
    }
    
    /**
     * Set robot parameters
     */
    setConfig(config) {
        if (config.wheelDistance !== undefined) this.L = config.wheelDistance;
        if (config.wheelRadius !== undefined) this.wheelRadius = config.wheelRadius;
    }
    
    /**
     * Set current robot heading for world-to-mobile conversion
     */
    setRobotHeading(theta) {
        this.robotHeading = theta;
    }
    
    /**
     * Convert world frame velocity to mobile (robot) frame
     */
    worldToMobile(Vxw, Vyw) {
        const cos_t = Math.cos(this.robotHeading);
        const sin_t = Math.sin(this.robotHeading);
        return {
            Vxm:  cos_t * Vxw + sin_t * Vyw,
            Vym: -sin_t * Vxw + cos_t * Vyw
        };
    }
    
    /**
     * Convert mobile (robot) frame velocity to world frame
     */
    mobileToWorld(Vxm, Vym) {
        const cos_t = Math.cos(this.robotHeading);
        const sin_t = Math.sin(this.robotHeading);
        return {
            Vxw: cos_t * Vxm - sin_t * Vym,
            Vyw: sin_t * Vxm + cos_t * Vym
        };
    }
    
    /**
     * Inverse Kinematics - Mobile Frame
     * Calculate wheel LINEAR velocities from robot velocity
     * 
     * @param {number} Vx - X velocity in robot frame (cm/s), positive = right
     * @param {number} Vy - Y velocity in robot frame (cm/s), positive = forward
     * @param {number} omega - Angular velocity (rad/s), positive = CCW
     * @returns {Object} Wheel linear velocities (cm/s)
     */
    inverseMobile(Vx, Vy, omega) {
        const L_omega = this.L * omega;
        
        // v_i = -sin(αi) * Vx + cos(αi) * Vy + L * ω
        return {
            v1_left:  -0.5 * Vx - this.sqrt3_2 * Vy + L_omega,  // V1 at 150°
            v2_back:   1.0 * Vx + 0 * Vy             + L_omega,  // V2 at 270°
            v3_right: -0.5 * Vx + this.sqrt3_2 * Vy + L_omega   // V3 at 30°
        };
    }
    
    /**
     * Inverse Kinematics - World Frame
     */
    inverseWorld(Vxw, Vyw, omega) {
        const mobile = this.worldToMobile(Vxw, Vyw);
        return this.inverseMobile(mobile.Vxm, mobile.Vym, omega);
    }
    
    /**
     * Get wheel ANGULAR velocities from linear velocities
     * Can accept either an object with v1_left, v2_back, v3_right
     * OR three separate parameters (Vxm, Vym, omega) to calculate directly
     * @returns {Object} Angular velocities in rad/s
     */
    getWheelAngularVelocities(arg1, arg2, arg3) {
        let wheelVelocities;
        
        // Check if called with 3 params (Vxm, Vym, omega) or 1 object param
        if (arg2 !== undefined && arg3 !== undefined) {
            // Called with (Vxm, Vym, omega) - calculate wheel linear velocities first
            wheelVelocities = this.inverseMobile(arg1, arg2, arg3);
        } else if (arg1 && typeof arg1 === 'object') {
            // Called with wheelVelocities object
            wheelVelocities = arg1;
        } else {
            // Invalid call - return zeros
            return {
                omega1_left: 0,
                omega2_back: 0,
                omega3_right: 0
            };
        }
        
        // Prevent NaN by checking for valid values
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
    
    /**
     * Forward Kinematics - Mobile Frame
     * Calculate robot velocity from wheel velocities
     */
    forwardMobile(v1, v2, v3) {
        // Derived from inverse kinematics matrix
        return {
            Vx:    (2 * v2 - v1 - v3) / 3,
            Vy:    (this.sqrt3 * v3 - this.sqrt3 * v1) / 3,
            omega: (v1 + v2 + v3) / (3 * this.L)
        };
    }
    
    /**
     * Calculate angular velocity needed to turn robot from current heading to target heading
     * 
     * @param {number} currentHeading - Current robot heading (rad)
     * @param {number} targetHeading - Target heading (rad)
     * @param {number} maxOmega - Maximum angular velocity (rad/s)
     * @param {number} dt - Time step (s)
     * @returns {number} Angular velocity (rad/s)
     */
    calculateTurnOmega(currentHeading, targetHeading, maxOmega = 0.8, dt = 0.1) {
        // Calculate shortest angle difference
        let deltaTheta = targetHeading - currentHeading;
        
        // Normalize to [-π, π]
        while (deltaTheta > Math.PI) deltaTheta -= 2 * Math.PI;
        while (deltaTheta < -Math.PI) deltaTheta += 2 * Math.PI;
        
        // P controller for smooth turning (reduced gain for slower rotation)
        const Kp = 1.0;  // Proportional gain - reduced for observation
        let omega = Kp * deltaTheta;
        
        // Clamp to max omega (reduced for slower turning)
        omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
        
        return omega;
    }
    
    /**
     * Calculate inverse kinematics from target position with rotation
     * Robot will rotate to face the direction of movement
     * 
     * @param {number} currentX - Current robot X position (cm)
     * @param {number} currentY - Current robot Y position (cm)
     * @param {number} currentTheta - Current robot heading (rad)
     * @param {number} targetX - Target X position (cm)
     * @param {number} targetY - Target Y position (cm)
     * @param {number} speed - Desired linear speed (cm/s)
     * @param {boolean} faceMovementDirection - If true, robot rotates to face movement direction
     * @returns {Object} Complete kinematics result
     */
    calculateFromTarget(currentX, currentY, currentTheta, targetX, targetY, speed, faceMovementDirection = true) {
        // Calculate direction to target
        const dx = targetX - currentX;
        const dy = targetY - currentY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        if (distance < 0.1) {
            // Already at target
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
        
        // Angle to target in world frame
        const angleToTarget = Math.atan2(dy, dx);
        
        // Calculate omega if robot should face movement direction
        let omega = 0;
        if (faceMovementDirection) {
            // Target heading: robot's +Y (front) should point toward target
            // +Y is at 90° from +X, so target heading = angleToTarget - 90° = angleToTarget - π/2
            const targetHeading = angleToTarget - Math.PI / 2;
            omega = this.calculateTurnOmega(currentTheta, targetHeading);
        }
        
        // Velocity components in world frame
        const Vxw = speed * Math.cos(angleToTarget);
        const Vyw = speed * Math.sin(angleToTarget);
        
        // Set current heading for conversion
        this.setRobotHeading(currentTheta);
        
        // Convert to mobile frame
        const mobile = this.worldToMobile(Vxw, Vyw);
        
        // Calculate wheel velocities (including rotation)
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
    
    /**
     * Calculate kinematics for a path segment (with turning at corners)
     */
    calculateForPathSegment(currentX, currentY, currentTheta, nextX, nextY, speed) {
        return this.calculateFromTarget(currentX, currentY, currentTheta, nextX, nextY, speed, true);
    }
}

// Export
window.Kinematics = Kinematics;
