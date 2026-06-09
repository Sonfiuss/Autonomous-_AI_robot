/**
 * Grid Module - Handles 2D grid rendering
 */
class Grid {
    constructor(canvas, config = {}) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        
        this.cellSize = config.cellSize || 20;
        this.scale = config.scale || 2;
        this.width = config.width || 800;
        this.height = config.height || 600;
        
        this.backgroundColor = '#111';
        this.gridColor = '#2a2a4a';
        this.majorGridColor = '#3a3a5a';
        this.axisColor = '#00d4ff';
        
        this.originX = this.width / 2;
        this.originY = this.height / 2;
        
        this.init();
    }
    
    init() {
        this.canvas.width = this.width;
        this.canvas.height = this.height;
    }
    
    worldToCanvas(x, y) {
        return {
            x: this.originX + x * this.scale,
            y: this.originY - y * this.scale
        };
    }
    
    canvasToWorld(px, py) {
        return {
            x: (px - this.originX) / this.scale,
            y: (this.originY - py) / this.scale
        };
    }
    
    draw() {
        const ctx = this.ctx;
        
        ctx.fillStyle = this.backgroundColor;
        ctx.fillRect(0, 0, this.width, this.height);
        
        ctx.strokeStyle = this.gridColor;
        ctx.lineWidth = 0.5;
        
        for (let x = this.originX % this.cellSize; x < this.width; x += this.cellSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, this.height);
            ctx.stroke();
        }
        
        for (let y = this.originY % this.cellSize; y < this.height; y += this.cellSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(this.width, y);
            ctx.stroke();
        }
        
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
        
        ctx.strokeStyle = this.axisColor;
        ctx.lineWidth = 2;
        
        ctx.beginPath();
        ctx.moveTo(0, this.originY);
        ctx.lineTo(this.width, this.originY);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(this.originX, 0);
        ctx.lineTo(this.originX, this.height);
        ctx.stroke();
        
        ctx.fillStyle = this.axisColor;
        ctx.font = '12px Consolas';
        ctx.fillText('X (cm)', this.width - 50, this.originY - 10);
        ctx.fillText('Y (cm)', this.originX + 10, 20);
        
        this.drawScaleMarkers();
    }
    
    drawScaleMarkers() {
        const ctx = this.ctx;
        const interval = 50 * this.scale;
        
        ctx.fillStyle = '#666';
        ctx.font = '10px Consolas';
        ctx.textAlign = 'center';
        
        for (let x = this.originX + interval; x < this.width; x += interval) {
            const worldX = Math.round((x - this.originX) / this.scale);
            ctx.fillText(worldX.toString(), x, this.originY + 15);
        }
        for (let x = this.originX - interval; x > 0; x -= interval) {
            const worldX = Math.round((x - this.originX) / this.scale);
            ctx.fillText(worldX.toString(), x, this.originY + 15);
        }
        
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
    
    setOrigin(x, y) {
        this.originX = x;
        this.originY = y;
    }
}

window.Grid = Grid;
