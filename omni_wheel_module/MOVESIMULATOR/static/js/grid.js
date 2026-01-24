/**
 * Grid Module - Handles 2D grid rendering
 */
class Grid {
    constructor(canvas, config = {}) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        
        // Grid configuration
        this.cellSize = config.cellSize || 20;      // pixels per cell
        this.scale = config.scale || 2;             // pixels per cm
        this.width = config.width || 800;
        this.height = config.height || 600;
        
        // Colors
        this.backgroundColor = '#111';
        this.gridColor = '#2a2a4a';
        this.majorGridColor = '#3a3a5a';
        this.axisColor = '#00d4ff';
        
        // Grid origin (center of canvas by default)
        this.originX = this.width / 2;
        this.originY = this.height / 2;
        
        this.init();
    }
    
    init() {
        this.canvas.width = this.width;
        this.canvas.height = this.height;
    }
    
    /**
     * Convert world coordinates (cm) to canvas pixels
     */
    worldToCanvas(x, y) {
        return {
            x: this.originX + x * this.scale,
            y: this.originY - y * this.scale  // Y is flipped in canvas
        };
    }
    
    /**
     * Convert canvas pixels to world coordinates (cm)
     */
    canvasToWorld(px, py) {
        return {
            x: (px - this.originX) / this.scale,
            y: (this.originY - py) / this.scale
        };
    }
    
    /**
     * Draw the grid
     */
    draw() {
        const ctx = this.ctx;
        
        // Clear and fill background
        ctx.fillStyle = this.backgroundColor;
        ctx.fillRect(0, 0, this.width, this.height);
        
        // Draw minor grid lines
        ctx.strokeStyle = this.gridColor;
        ctx.lineWidth = 0.5;
        
        // Vertical lines
        for (let x = this.originX % this.cellSize; x < this.width; x += this.cellSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, this.height);
            ctx.stroke();
        }
        
        // Horizontal lines
        for (let y = this.originY % this.cellSize; y < this.height; y += this.cellSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(this.width, y);
            ctx.stroke();
        }
        
        // Draw major grid lines (every 5 cells = 50cm if scale=2, cellSize=20)
        const majorInterval = this.cellSize * 5;
        ctx.strokeStyle = this.majorGridColor;
        ctx.lineWidth = 1;
        
        for (let x = this.originX % majorInterval; x < this.width; x += majorInterval) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, this.height);
            ctx.stroke();
        }
        
        for (let y = this.originY % majorInterval; y < this.height; y += majorInterval) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(this.width, y);
            ctx.stroke();
        }
        
        // Draw axes
        ctx.strokeStyle = this.axisColor;
        ctx.lineWidth = 2;
        
        // X axis
        ctx.beginPath();
        ctx.moveTo(0, this.originY);
        ctx.lineTo(this.width, this.originY);
        ctx.stroke();
        
        // Y axis
        ctx.beginPath();
        ctx.moveTo(this.originX, 0);
        ctx.lineTo(this.originX, this.height);
        ctx.stroke();
        
        // Draw axis labels
        ctx.fillStyle = this.axisColor;
        ctx.font = '12px Consolas';
        ctx.fillText('X (cm)', this.width - 50, this.originY - 10);
        ctx.fillText('Y (cm)', this.originX + 10, 20);
        
        // Draw scale markers
        this.drawScaleMarkers();
    }
    
    drawScaleMarkers() {
        const ctx = this.ctx;
        const interval = 50 * this.scale; // Every 50cm
        
        ctx.fillStyle = '#666';
        ctx.font = '10px Consolas';
        ctx.textAlign = 'center';
        
        // X axis markers
        for (let x = this.originX + interval; x < this.width; x += interval) {
            const worldX = Math.round((x - this.originX) / this.scale);
            ctx.fillText(worldX.toString(), x, this.originY + 15);
        }
        for (let x = this.originX - interval; x > 0; x -= interval) {
            const worldX = Math.round((x - this.originX) / this.scale);
            ctx.fillText(worldX.toString(), x, this.originY + 15);
        }
        
        // Y axis markers
        ctx.textAlign = 'right';
        for (let y = this.originY - interval; y > 0; y -= interval) {
            const worldY = Math.round((this.originY - y) / this.scale);
            ctx.fillText(worldY.toString(), this.originX - 5, y + 4);
        }
        for (let y = this.originY + interval; y < this.height; y += interval) {
            const worldY = Math.round((this.originY - y) / this.scale);
            ctx.fillText(worldY.toString(), this.originX - 5, y + 4);
        }
    }
    
    /**
     * Update grid configuration
     */
    setConfig(config) {
        if (config.cellSize) this.cellSize = config.cellSize;
        if (config.scale) this.scale = config.scale;
        if (config.width) {
            this.width = config.width;
            this.canvas.width = config.width;
            this.originX = config.width / 2;
        }
        if (config.height) {
            this.height = config.height;
            this.canvas.height = config.height;
            this.originY = config.height / 2;
        }
    }
    
    /**
     * Set origin position
     */
    setOrigin(x, y) {
        this.originX = x;
        this.originY = y;
    }
}

// Export for use in other modules
window.Grid = Grid;
