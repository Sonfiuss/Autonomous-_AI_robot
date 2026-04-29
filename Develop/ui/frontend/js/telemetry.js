/**
 * Telemetry - Sensor Data Display
 * 
 * Hiển thị và cập nhật dữ liệu cảm biến real-time.
 */

const Telemetry = {
    updateInterval: 200, // ms
    timer: null,

    init() {
        // TODO: Kết nối WebSocket để nhận data real-time
        // Tạm thời poll qua REST API
        // this.startPolling();
        console.log("Telemetry module initialized");
    },

    startPolling() {
        this.timer = setInterval(() => this.fetchSensorData(), this.updateInterval);
    },

    stopPolling() {
        if (this.timer) clearInterval(this.timer);
    },

    async fetchSensorData() {
        try {
            const response = await fetch("/api/sensors");
            const data = await response.json();
            this.updateDisplay(data.sensors);
        } catch (error) {
            // Silently handle errors
        }
    },

    updateDisplay(sensors) {
        if (sensors.ultrasonic !== undefined) {
            document.getElementById("us-data").textContent = sensors.ultrasonic;
        }
        if (sensors.heading !== undefined) {
            document.getElementById("imu-data").textContent = sensors.heading.toFixed(1);
        }
        if (sensors.speed !== undefined) {
            document.getElementById("speed-data").textContent = sensors.speed.toFixed(0);
        }
    },
};

document.addEventListener("DOMContentLoaded", () => Telemetry.init());
