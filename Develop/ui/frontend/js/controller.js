/**
 * Controller - Joystick & Gamepad
 * 
 * Xử lý input từ joystick ảo và gamepad vật lý.
 */

const Controller = {
    gamepadIndex: null,
    deadzone: 0.15,

    init() {
        this.setupGamepad();
        // TODO: Setup virtual joystick (nipplejs hoặc custom)
    },

    setupGamepad() {
        window.addEventListener("gamepadconnected", (e) => {
            console.log(`Gamepad connected: ${e.gamepad.id}`);
            this.gamepadIndex = e.gamepad.index;
            this.pollGamepad();
        });

        window.addEventListener("gamepaddisconnected", () => {
            console.log("Gamepad disconnected");
            this.gamepadIndex = null;
        });
    },

    pollGamepad() {
        if (this.gamepadIndex === null) return;

        const gamepad = navigator.getGamepads()[this.gamepadIndex];
        if (!gamepad) return;

        // Left stick = movement (vx, vy)
        const vx = this.applyDeadzone(gamepad.axes[1]) * -1; // Forward/backward
        const vy = this.applyDeadzone(gamepad.axes[0]);       // Left/right

        // Right stick X = rotation (omega)
        const omega = this.applyDeadzone(gamepad.axes[2]);

        if (Math.abs(vx) > 0 || Math.abs(vy) > 0 || Math.abs(omega) > 0) {
            App.sendCommand("move", { vx, vy, omega });
        }

        requestAnimationFrame(() => this.pollGamepad());
    },

    applyDeadzone(value) {
        return Math.abs(value) > this.deadzone ? value : 0;
    },
};

document.addEventListener("DOMContentLoaded", () => Controller.init());
