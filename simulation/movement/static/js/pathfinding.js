/**
 * Pathfinding Module - A* algorithm for grid-based path planning
 */

class PathFinder {
    constructor(grid, obstacleManager) {
        this.grid = grid;
        this.obstacleManager = obstacleManager;
        this.resolution = 5;
        this.smoothIterations = 3;
    }
    
    findPath(start, goal, robotRadius) {
        const startCell = this.worldToCell(start.x, start.y);
        const goalCell = this.worldToCell(goal.x, goal.y);
        
        if (this.isBlocked(startCell.row, startCell.col, robotRadius)) {
            console.error('Start position is blocked');
            return null;
        }
        if (this.isBlocked(goalCell.row, goalCell.col, robotRadius)) {
            console.error('Goal position is blocked');
            return null;
        }
        
        const openSet = new MinHeap();
        const closedSet = new Set();
        const cameFrom = new Map();
        const gScore = new Map();
        const fScore = new Map();
        
        const startKey = this.cellKey(startCell.row, startCell.col);
        gScore.set(startKey, 0);
        fScore.set(startKey, this.heuristic(startCell, goalCell));
        openSet.push({ cell: startCell, f: fScore.get(startKey) });
        
        const directions = [
            { dr: -1, dc: 0, cost: 1 },
            { dr: 1, dc: 0, cost: 1 },
            { dr: 0, dc: -1, cost: 1 },
            { dr: 0, dc: 1, cost: 1 },
            { dr: -1, dc: -1, cost: Math.SQRT2 },
            { dr: -1, dc: 1, cost: Math.SQRT2 },
            { dr: 1, dc: -1, cost: Math.SQRT2 },
            { dr: 1, dc: 1, cost: Math.SQRT2 }
        ];
        
        while (!openSet.isEmpty()) {
            const current = openSet.pop().cell;
            const currentKey = this.cellKey(current.row, current.col);
            
            if (current.row === goalCell.row && current.col === goalCell.col) {
                return this.reconstructPath(cameFrom, current, start, goal);
            }
            
            closedSet.add(currentKey);
            
            for (const dir of directions) {
                const neighbor = {
                    row: current.row + dir.dr,
                    col: current.col + dir.dc
                };
                const neighborKey = this.cellKey(neighbor.row, neighbor.col);
                
                if (closedSet.has(neighborKey)) continue;
                if (this.isBlocked(neighbor.row, neighbor.col, robotRadius)) continue;
                
                if (dir.dr !== 0 && dir.dc !== 0) {
                    if (this.isBlocked(current.row + dir.dr, current.col, robotRadius) ||
                        this.isBlocked(current.row, current.col + dir.dc, robotRadius)) {
                        continue;
                    }
                }
                
                const tentativeG = gScore.get(currentKey) + dir.cost * this.resolution;
                
                if (!gScore.has(neighborKey) || tentativeG < gScore.get(neighborKey)) {
                    cameFrom.set(neighborKey, current);
                    gScore.set(neighborKey, tentativeG);
                    fScore.set(neighborKey, tentativeG + this.heuristic(neighbor, goalCell));
                    openSet.push({ cell: neighbor, f: fScore.get(neighborKey) });
                }
            }
        }
        
        return null;
    }
    
    worldToCell(x, y) {
        return {
            row: Math.floor(y / this.resolution),
            col: Math.floor(x / this.resolution)
        };
    }
    
    cellToWorld(row, col) {
        return {
            x: (col + 0.5) * this.resolution,
            y: (row + 0.5) * this.resolution
        };
    }
    
    cellKey(row, col) {
        return `${row},${col}`;
    }
    
    isBlocked(row, col, robotRadius) {
        const world = this.cellToWorld(row, col);
        return this.obstacleManager.checkCollision(world.x, world.y, robotRadius);
    }
    
    heuristic(cell, goal) {
        const dx = (cell.col - goal.col) * this.resolution;
        const dy = (cell.row - goal.row) * this.resolution;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    reconstructPath(cameFrom, current, start, goal) {
        const path = [goal];
        let key = this.cellKey(current.row, current.col);
        while (cameFrom.has(key)) {
            const cell = cameFrom.get(key);
            const world = this.cellToWorld(cell.row, cell.col);
            path.unshift(world);
            key = this.cellKey(cell.row, cell.col);
        }
        path[0] = start;
        return this.smoothPath(path);
    }
    
    smoothPath(path) {
        if (path.length <= 2) return path;
        const smoothed = [path[0]];
        let current = 0;
        while (current < path.length - 1) {
            let furthest = current + 1;
            for (let i = path.length - 1; i > current + 1; i--) {
                if (this.hasLineOfSight(path[current], path[i])) {
                    furthest = i;
                    break;
                }
            }
            smoothed.push(path[furthest]);
            current = furthest;
        }
        return smoothed;
    }
    
    hasLineOfSight(p1, p2, robotRadius = 20) {
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const steps = Math.ceil(dist / (this.resolution / 2));
        for (let i = 1; i < steps; i++) {
            const t = i / steps;
            const x = p1.x + dx * t;
            const y = p1.y + dy * t;
            if (this.obstacleManager.checkCollision(x, y, robotRadius)) {
                return false;
            }
        }
        return true;
    }
}

class MinHeap {
    constructor() {
        this.heap = [];
    }
    
    push(item) {
        this.heap.push(item);
        this.bubbleUp(this.heap.length - 1);
    }
    
    pop() {
        if (this.heap.length === 0) return null;
        if (this.heap.length === 1) return this.heap.pop();
        const min = this.heap[0];
        this.heap[0] = this.heap.pop();
        this.bubbleDown(0);
        return min;
    }
    
    isEmpty() {
        return this.heap.length === 0;
    }
    
    bubbleUp(index) {
        while (index > 0) {
            const parent = Math.floor((index - 1) / 2);
            if (this.heap[parent].f <= this.heap[index].f) break;
            [this.heap[parent], this.heap[index]] = [this.heap[index], this.heap[parent]];
            index = parent;
        }
    }
    
    bubbleDown(index) {
        const length = this.heap.length;
        while (true) {
            const left = 2 * index + 1;
            const right = 2 * index + 2;
            let smallest = index;
            if (left < length && this.heap[left].f < this.heap[smallest].f) {
                smallest = left;
            }
            if (right < length && this.heap[right].f < this.heap[smallest].f) {
                smallest = right;
            }
            if (smallest === index) break;
            [this.heap[index], this.heap[smallest]] = [this.heap[smallest], this.heap[index]];
            index = smallest;
        }
    }
}

class PathRenderer {
    constructor(grid) {
        this.grid = grid;
        this.ctx = grid.ctx;
        this.path = [];
        this.pathColor = '#00ff00';
        this.waypointColor = '#ffff00';
    }
    
    setPath(path) {
        this.path = path || [];
    }
    
    draw() {
        if (this.path.length < 2) return;
        const ctx = this.ctx;
        ctx.beginPath();
        const first = this.grid.worldToCanvas(this.path[0].x, this.path[0].y);
        ctx.moveTo(first.x, first.y);
        for (let i = 1; i < this.path.length; i++) {
            const p = this.grid.worldToCanvas(this.path[i].x, this.path[i].y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.strokeStyle = this.pathColor;
        ctx.lineWidth = 3;
        ctx.setLineDash([10, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
        
        for (let i = 0; i < this.path.length; i++) {
            const p = this.grid.worldToCanvas(this.path[i].x, this.path[i].y);
            ctx.beginPath();
            ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
            ctx.fillStyle = i === 0 ? '#00ff00' : i === this.path.length - 1 ? '#ff0000' : this.waypointColor;
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
        this.drawDirectionArrows();
    }
    
    drawDirectionArrows() {
        if (this.path.length < 2) return;
        const ctx = this.ctx;
        ctx.fillStyle = this.pathColor;
        for (let i = 0; i < this.path.length - 1; i++) {
            const p1 = this.path[i];
            const p2 = this.path[i + 1];
            const midX = (p1.x + p2.x) / 2;
            const midY = (p1.y + p2.y) / 2;
            const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
            const pos = this.grid.worldToCanvas(midX, midY);
            ctx.save();
            ctx.translate(pos.x, pos.y);
            ctx.rotate(-angle);
            ctx.beginPath();
            ctx.moveTo(8, 0);
            ctx.lineTo(-4, -5);
            ctx.lineTo(-4, 5);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }
    }
    
    getPathLength() {
        let length = 0;
        for (let i = 1; i < this.path.length; i++) {
            const dx = this.path[i].x - this.path[i - 1].x;
            const dy = this.path[i].y - this.path[i - 1].y;
            length += Math.sqrt(dx * dx + dy * dy);
        }
        return length;
    }
}

window.PathFinder = PathFinder;
window.PathRenderer = PathRenderer;
