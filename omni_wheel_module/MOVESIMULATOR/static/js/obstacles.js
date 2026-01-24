/**
 * Obstacles Module - Handles obstacle creation and rendering
 * Supports: Circle, Rectangle, Parallelogram
 */

// Obstacle types enum
const ObstacleType = {
    CIRCLE: 'circle',
    RECTANGLE: 'rectangle',
    PARALLELOGRAM: 'parallelogram'
};

/**
 * Base Obstacle class
 */
class Obstacle {
    constructor(x, y, type) {
        this.x = x;  // Center position in world coords (cm)
        this.y = y;
        this.type = type;
        this.color = '#e74c3c';
        this.outlineColor = '#c0392b';
    }
    
    /**
     * Check if point is inside obstacle
     * @returns {boolean}
     */
    containsPoint(px, py) {
        return false;  // Override in subclass
    }
    
    /**
     * Get bounding box [minX, minY, maxX, maxY]
     */
    getBoundingBox() {
        return [this.x, this.y, this.x, this.y];  // Override in subclass
    }
    
    /**
     * Check collision with a circle (robot bounding)
     */
    collidesWithCircle(cx, cy, radius) {
        return false;  // Override in subclass
    }
    
    /**
     * Draw the obstacle
     */
    draw(ctx, grid) {
        // Override in subclass
    }
}

/**
 * Circle Obstacle
 */
class CircleObstacle extends Obstacle {
    constructor(x, y, radius) {
        super(x, y, ObstacleType.CIRCLE);
        this.radius = radius;  // cm
    }
    
    containsPoint(px, py) {
        const dx = px - this.x;
        const dy = py - this.y;
        return Math.sqrt(dx * dx + dy * dy) <= this.radius;
    }
    
    getBoundingBox() {
        return [
            this.x - this.radius,
            this.y - this.radius,
            this.x + this.radius,
            this.y + this.radius
        ];
    }
    
    collidesWithCircle(cx, cy, radius) {
        const dx = cx - this.x;
        const dy = cy - this.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        return dist <= (this.radius + radius);
    }
    
    draw(ctx, grid) {
        const pos = grid.worldToCanvas(this.x, this.y);
        const scale = grid.scale;
        
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, this.radius * scale, 0, Math.PI * 2);
        ctx.fillStyle = this.color;
        ctx.fill();
        ctx.strokeStyle = this.outlineColor;
        ctx.lineWidth = 2;
        ctx.stroke();
    }
}

/**
 * Rectangle Obstacle
 */
class RectangleObstacle extends Obstacle {
    constructor(x, y, width, height, rotation = 0) {
        super(x, y, ObstacleType.RECTANGLE);
        this.width = width;    // cm
        this.height = height;  // cm
        this.rotation = rotation;  // radians
    }
    
    /**
     * Get corners in world coordinates
     */
    getCorners() {
        const hw = this.width / 2;
        const hh = this.height / 2;
        const cos = Math.cos(this.rotation);
        const sin = Math.sin(this.rotation);
        
        // Local corners
        const local = [
            [-hw, -hh],
            [hw, -hh],
            [hw, hh],
            [-hw, hh]
        ];
        
        // Transform to world
        return local.map(([lx, ly]) => ({
            x: this.x + lx * cos - ly * sin,
            y: this.y + lx * sin + ly * cos
        }));
    }
    
    containsPoint(px, py) {
        // Transform point to local coordinates
        const dx = px - this.x;
        const dy = py - this.y;
        const cos = Math.cos(-this.rotation);
        const sin = Math.sin(-this.rotation);
        
        const localX = dx * cos - dy * sin;
        const localY = dx * sin + dy * cos;
        
        return Math.abs(localX) <= this.width / 2 && Math.abs(localY) <= this.height / 2;
    }
    
    getBoundingBox() {
        const corners = this.getCorners();
        const xs = corners.map(c => c.x);
        const ys = corners.map(c => c.y);
        return [
            Math.min(...xs),
            Math.min(...ys),
            Math.max(...xs),
            Math.max(...ys)
        ];
    }
    
    collidesWithCircle(cx, cy, radius) {
        // Transform circle center to local coordinates
        const dx = cx - this.x;
        const dy = cy - this.y;
        const cos = Math.cos(-this.rotation);
        const sin = Math.sin(-this.rotation);
        
        const localX = dx * cos - dy * sin;
        const localY = dx * sin + dy * cos;
        
        // Find closest point on rectangle
        const closestX = Math.max(-this.width / 2, Math.min(this.width / 2, localX));
        const closestY = Math.max(-this.height / 2, Math.min(this.height / 2, localY));
        
        // Check distance
        const distX = localX - closestX;
        const distY = localY - closestY;
        return Math.sqrt(distX * distX + distY * distY) <= radius;
    }
    
    draw(ctx, grid) {
        const pos = grid.worldToCanvas(this.x, this.y);
        const scale = grid.scale;
        
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-this.rotation);  // Negative for canvas coords
        
        const w = this.width * scale;
        const h = this.height * scale;
        
        ctx.fillStyle = this.color;
        ctx.fillRect(-w / 2, -h / 2, w, h);
        ctx.strokeStyle = this.outlineColor;
        ctx.lineWidth = 2;
        ctx.strokeRect(-w / 2, -h / 2, w, h);
        
        ctx.restore();
    }
}

/**
 * Parallelogram Obstacle
 */
class ParallelogramObstacle extends Obstacle {
    constructor(x, y, width, height, skew = 0.3) {
        super(x, y, ObstacleType.PARALLELOGRAM);
        this.width = width;    // cm
        this.height = height;  // cm
        this.skew = skew;      // Skew factor (0 = rectangle)
        this.rotation = 0;
    }
    
    /**
     * Get corners in world coordinates
     */
    getCorners() {
        const hw = this.width / 2;
        const hh = this.height / 2;
        const skewOffset = this.skew * this.width;
        
        // Local corners (parallelogram shape)
        const local = [
            [-hw + skewOffset, -hh],
            [hw + skewOffset, -hh],
            [hw - skewOffset, hh],
            [-hw - skewOffset, hh]
        ];
        
        const cos = Math.cos(this.rotation);
        const sin = Math.sin(this.rotation);
        
        return local.map(([lx, ly]) => ({
            x: this.x + lx * cos - ly * sin,
            y: this.y + lx * sin + ly * cos
        }));
    }
    
    containsPoint(px, py) {
        // Use cross product method for polygon containment
        const corners = this.getCorners();
        let inside = true;
        
        for (let i = 0; i < corners.length; i++) {
            const j = (i + 1) % corners.length;
            const edge = {
                x: corners[j].x - corners[i].x,
                y: corners[j].y - corners[i].y
            };
            const toPoint = {
                x: px - corners[i].x,
                y: py - corners[i].y
            };
            const cross = edge.x * toPoint.y - edge.y * toPoint.x;
            if (cross < 0) {
                inside = false;
                break;
            }
        }
        return inside;
    }
    
    getBoundingBox() {
        const corners = this.getCorners();
        const xs = corners.map(c => c.x);
        const ys = corners.map(c => c.y);
        return [
            Math.min(...xs),
            Math.min(...ys),
            Math.max(...xs),
            Math.max(...ys)
        ];
    }
    
    collidesWithCircle(cx, cy, radius) {
        // Check if circle center is inside
        if (this.containsPoint(cx, cy)) return true;
        
        // Check distance to each edge
        const corners = this.getCorners();
        for (let i = 0; i < corners.length; i++) {
            const j = (i + 1) % corners.length;
            const dist = this.pointToSegmentDistance(
                cx, cy,
                corners[i].x, corners[i].y,
                corners[j].x, corners[j].y
            );
            if (dist <= radius) return true;
        }
        return false;
    }
    
    pointToSegmentDistance(px, py, x1, y1, x2, y2) {
        const dx = x2 - x1;
        const dy = y2 - y1;
        const len2 = dx * dx + dy * dy;
        
        if (len2 === 0) {
            return Math.sqrt((px - x1) ** 2 + (py - y1) ** 2);
        }
        
        let t = ((px - x1) * dx + (py - y1) * dy) / len2;
        t = Math.max(0, Math.min(1, t));
        
        const nearestX = x1 + t * dx;
        const nearestY = y1 + t * dy;
        
        return Math.sqrt((px - nearestX) ** 2 + (py - nearestY) ** 2);
    }
    
    draw(ctx, grid) {
        const corners = this.getCorners();
        const scale = grid.scale;
        
        ctx.beginPath();
        const first = grid.worldToCanvas(corners[0].x, corners[0].y);
        ctx.moveTo(first.x, first.y);
        
        for (let i = 1; i < corners.length; i++) {
            const p = grid.worldToCanvas(corners[i].x, corners[i].y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        
        ctx.fillStyle = this.color;
        ctx.fill();
        ctx.strokeStyle = this.outlineColor;
        ctx.lineWidth = 2;
        ctx.stroke();
    }
}

/**
 * Obstacle Manager - handles collection of obstacles
 */
class ObstacleManager {
    constructor(grid) {
        this.grid = grid;
        this.obstacles = [];
        this.selectedTool = ObstacleType.CIRCLE;
        this.defaultSize = 30;  // cm
    }
    
    /**
     * Add obstacle at world position
     */
    addObstacle(worldX, worldY, type = null, size = null) {
        type = type || this.selectedTool;
        size = size || this.defaultSize;
        
        let obstacle;
        switch (type) {
            case ObstacleType.CIRCLE:
                obstacle = new CircleObstacle(worldX, worldY, size / 2);
                break;
            case ObstacleType.RECTANGLE:
                obstacle = new RectangleObstacle(worldX, worldY, size, size * 0.6);
                break;
            case ObstacleType.PARALLELOGRAM:
                obstacle = new ParallelogramObstacle(worldX, worldY, size, size * 0.5, 0.3);
                break;
            default:
                return null;
        }
        
        this.obstacles.push(obstacle);
        return obstacle;
    }
    
    /**
     * Remove obstacle at position
     */
    removeObstacleAt(worldX, worldY) {
        for (let i = this.obstacles.length - 1; i >= 0; i--) {
            if (this.obstacles[i].containsPoint(worldX, worldY)) {
                this.obstacles.splice(i, 1);
                return true;
            }
        }
        return false;
    }
    
    /**
     * Clear all obstacles
     */
    clear() {
        this.obstacles = [];
    }
    
    /**
     * Check if position collides with any obstacle
     */
    checkCollision(worldX, worldY, radius) {
        for (const obs of this.obstacles) {
            if (obs.collidesWithCircle(worldX, worldY, radius)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Draw all obstacles
     */
    draw() {
        for (const obs of this.obstacles) {
            obs.draw(this.grid.ctx, this.grid);
        }
    }
    
    /**
     * Get obstacles data for pathfinding
     */
    getObstaclesData() {
        return this.obstacles.map(obs => ({
            type: obs.type,
            x: obs.x,
            y: obs.y,
            boundingBox: obs.getBoundingBox()
        }));
    }
}

// Export for use in other modules
window.ObstacleType = ObstacleType;
window.CircleObstacle = CircleObstacle;
window.RectangleObstacle = RectangleObstacle;
window.ParallelogramObstacle = ParallelogramObstacle;
window.ObstacleManager = ObstacleManager;
