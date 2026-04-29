/**
 * Main Application Logic
 * 
 * Khởi tạo kết nối WebSocket và quản lý state ứng dụng.
 */

// const socket = io();  // Uncomment khi dùng Socket.IO

const App = {
    isConnected: false,

    init() {
        console.log("Robot Control Panel initialized");
        this.setupEventListeners();
        this.checkStatus();
    },

    setupEventListeners() {
        document.getElementById("btn-stop").addEventListener("click", () => {
            this.sendCommand("stop", {});
        });

        document.getElementById("btn-home").addEventListener("click", () => {
            this.sendCommand("home", {});
        });
    },

    async checkStatus() {
        try {
            const response = await fetch("/api/status");
            const data = await response.json();
            this.updateConnectionStatus(data.status === "online");
        } catch (error) {
            this.updateConnectionStatus(false);
        }
    },

    updateConnectionStatus(connected) {
        this.isConnected = connected;
        const indicator = document.getElementById("status-indicator");
        indicator.textContent = connected ? "Connected" : "Disconnected";
        indicator.className = `status ${connected ? "connected" : "disconnected"}`;
    },

    sendCommand(action, params) {
        console.log(`Command: ${action}`, params);
        // TODO: Gửi qua WebSocket hoặc REST API
        // socket.emit("control", { action, params });
    },
};

document.addEventListener("DOMContentLoaded", () => App.init());
