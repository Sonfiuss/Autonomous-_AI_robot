/**
 * Control Panel Logic for 6DOF Robotic Arm
 * Handles user input and communication with backend
 */

class ArmController {
    constructor() {
        this.apiBase = '';
        this.joints = [0, 0, 0, 0, 0, 0];
        this.dhParams = {
            d1: 100,
            a2: 200,
            a3: 200,
            d4: 50,
            d5: 50,
            d6: 50
        };
        
        this.init();
    }
    
    async init() {
        // Load initial state from server
        await this.loadState();
        
        // Setup event listeners
        this.setupJointSliders();
        this.setupDHSliders();
        this.setupDirectInput();
        this.setupResetButton();
        this.setupObjectControls();
    }
    
    async loadState() {
        try {
            const response = await fetch(`${this.apiBase}/api/state`);
            const data = await response.json();
            
            this.joints = data.joints;
            this.dhParams = data.dh_params;
            
            // Update UI
            this.updateJointUI();
            this.updateDHUI();
            
            // Update 3D visualization
            if (typeof arm3D !== 'undefined') {
                arm3D.updateArm(data.positions);
            }
        } catch (error) {
            console.error('Error loading state:', error);
        }
    }
    
    setupJointSliders() {
        for (let i = 1; i <= 6; i++) {
            const slider = document.getElementById(`joint-${i}`);
            const valueDisplay = document.getElementById(`j${i}-value`);
            const directInput = document.getElementById(`input-j${i}`);
            
            if (slider) {
                slider.addEventListener('input', (e) => {
                    const value = parseFloat(e.target.value);
                    this.joints[i - 1] = value;
                    valueDisplay.textContent = `${value}°`;
                    directInput.value = value;
                    this.debouncedUpdateJoints();
                });
            }
        }
    }
    
    setupDHSliders() {
        const dhParams = ['d1', 'a2', 'a3', 'd4', 'd5', 'd6'];
        
        dhParams.forEach(param => {
            const slider = document.getElementById(`dh-${param}`);
            const valueDisplay = document.getElementById(`${param}-value`);
            
            if (slider) {
                slider.addEventListener('input', (e) => {
                    const value = parseFloat(e.target.value);
                    this.dhParams[param] = value;
                    valueDisplay.textContent = `${value} mm`;
                    this.debouncedUpdateDH();
                });
            }
        });
    }
    
    setupDirectInput() {
        const applyButton = document.getElementById('btn-apply-joints');
        
        if (applyButton) {
            applyButton.addEventListener('click', () => {
                for (let i = 1; i <= 6; i++) {
                    const input = document.getElementById(`input-j${i}`);
                    if (input) {
                        this.joints[i - 1] = parseFloat(input.value) || 0;
                    }
                }
                this.updateJointUI();
                this.updateJoints();
            });
        }
        
        // Allow Enter key to apply
        for (let i = 1; i <= 6; i++) {
            const input = document.getElementById(`input-j${i}`);
            if (input) {
                input.addEventListener('keypress', (e) => {
                    if (e.key === 'Enter') {
                        applyButton.click();
                    }
                });
            }
        }
    }
    
    setupResetButton() {
        const resetButton = document.getElementById('btn-reset-arm');
        
        if (resetButton) {
            resetButton.addEventListener('click', async () => {
                try {
                    const response = await fetch(`${this.apiBase}/api/reset`, {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' }
                    });
                    const data = await response.json();
                    
                    this.joints = data.joints;
                    this.dhParams = data.dh_params;
                    
                    this.updateJointUI();
                    this.updateDHUI();
                    
                    if (typeof arm3D !== 'undefined') {
                        arm3D.updateArm(data.positions);
                    }
                } catch (error) {
                    console.error('Error resetting arm:', error);
                }
            });
        }
    }
    
    updateJointUI() {
        for (let i = 1; i <= 6; i++) {
            const slider = document.getElementById(`joint-${i}`);
            const valueDisplay = document.getElementById(`j${i}-value`);
            const directInput = document.getElementById(`input-j${i}`);
            
            const value = this.joints[i - 1];
            
            if (slider) slider.value = value;
            if (valueDisplay) valueDisplay.textContent = `${value}°`;
            if (directInput) directInput.value = value;
        }
    }
    
    updateDHUI() {
        const dhParams = ['d1', 'a2', 'a3', 'd4', 'd5', 'd6'];
        
        dhParams.forEach(param => {
            const slider = document.getElementById(`dh-${param}`);
            const valueDisplay = document.getElementById(`${param}-value`);
            
            const value = this.dhParams[param];
            
            if (slider) slider.value = value;
            if (valueDisplay) valueDisplay.textContent = `${value} mm`;
        });
    }
    
    // Debounced update functions to avoid too many API calls
    debounce(func, wait) {
        let timeout;
        return (...args) => {
            clearTimeout(timeout);
            timeout = setTimeout(() => func.apply(this, args), wait);
        };
    }
    
    debouncedUpdateJoints = this.debounce(this.updateJoints, 50);
    debouncedUpdateDH = this.debounce(this.updateDH, 50);
    
    async updateJoints() {
        try {
            const response = await fetch(`${this.apiBase}/api/joints`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ joints: this.joints })
            });
            const data = await response.json();
            
            // Update 3D visualization
            if (typeof arm3D !== 'undefined') {
                arm3D.updateArm(data.positions);
            }
        } catch (error) {
            console.error('Error updating joints:', error);
        }
    }
    
    async updateDH() {
        try {
            const response = await fetch(`${this.apiBase}/api/dh_params`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(this.dhParams)
            });
            const data = await response.json();
            
            // Update 3D visualization
            if (typeof arm3D !== 'undefined') {
                arm3D.updateArm(data.positions);
                arm3D.updateMaxReach(this.dhParams);
            }
        } catch (error) {
            console.error('Error updating DH params:', error);
        }
    }
    
    // ===== Object Controls & Grip Functionality =====
    
    setupObjectControls() {
        const gripButton = document.getElementById('btn-grip');
        const removeButton = document.getElementById('btn-remove-object');
        const clearButton = document.getElementById('btn-clear-objects');
        
        if (gripButton) {
            gripButton.addEventListener('click', () => this.gripObject());
        }
        
        if (removeButton) {
            removeButton.addEventListener('click', () => {
                if (typeof arm3D !== 'undefined') {
                    arm3D.removeSelectedObject();
                }
            });
        }
        
        if (clearButton) {
            clearButton.addEventListener('click', () => {
                if (typeof arm3D !== 'undefined') {
                    arm3D.clearAllObjects();
                }
            });
        }
    }
    
    async gripObject() {
        if (typeof arm3D === 'undefined') return;
        
        const objectPos = arm3D.getSelectedObjectPosition();
        
        if (!objectPos) {
            this.showMessage('⚠️ Chưa chọn vật thể nào! Double-click để đặt vật.', 'warning');
            return;
        }
        
        // Get object data
        const objData = arm3D.placedObjects.find(o => o.mesh === arm3D.selectedObject);
        if (!objData) {
            this.showMessage('⚠️ Không tìm thấy dữ liệu vật thể!', 'warning');
            return;
        }
        
        // Calculate target position: gripper fingertips should touch top of object
        // Object bottom is at Y=0, so top is at Y = height
        const targetX = objectPos.x;
        const targetY = objData.dimensions.height;  // Top of the object
        const targetZ = objectPos.z;
        
        console.log(`Target position: X=${targetX}, Y=${targetY}, Z=${targetZ}`);
        
        try {
            const response = await fetch(`${this.apiBase}/api/inverse_kinematics`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    target_x: targetX,
                    target_y: targetY,
                    target_z: targetZ,
                    dh_params: this.dhParams
                })
            });
            
            const data = await response.json();
            
            // Show target marker for debugging
            arm3D.showTargetMarker(targetX, targetY, targetZ);
            
            if (data.success) {
                this.showMessage('✅ Đang di chuyển cánh tay để gắp vật...', 'success');
                console.log('IK joints:', data.joints);
                
                // Animate joints to target position
                await this.animateToJoints(data.joints);
                
                // Debug: check gripper tip position
                const tipPos = arm3D.getGripperTipPosition();
                if (tipPos) {
                    console.log(`Gripper tip at: X=${tipPos.x.toFixed(1)}, Y=${tipPos.y.toFixed(1)}, Z=${tipPos.z.toFixed(1)}`);
                    console.log(`Target was: X=${targetX.toFixed(1)}, Y=${targetY.toFixed(1)}, Z=${targetZ.toFixed(1)}`);
                }
                
                // Show grab animation
                arm3D.showGrabAnimation(true);
                
                this.showMessage('🎉 Đã gắp vật thể thành công!', 'success');
            } else {
                this.showMessage(`❌ ${data.message || 'Không thể tính toán vị trí gắp!'}`, 'error');
            }
        } catch (error) {
            console.error('Error calculating IK:', error);
            this.showMessage('❌ Lỗi khi tính toán vị trí gắp!', 'error');
        }
    }
    
    async animateToJoints(targetJoints) {
        const startJoints = [...this.joints];
        const duration = 1500;
        const startTime = Date.now();
        
        return new Promise((resolve) => {
            const animateStep = async () => {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1);
                const eased = this.easeOutCubic(progress);
                
                // Interpolate joints
                for (let i = 0; i < 6; i++) {
                    this.joints[i] = startJoints[i] + (targetJoints[i] - startJoints[i]) * eased;
                }
                
                // Update UI
                this.updateJointUI();
                
                // Update arm
                await this.updateJoints();
                
                if (progress < 1) {
                    requestAnimationFrame(animateStep);
                } else {
                    resolve();
                }
            };
            
            animateStep();
        });
    }
    
    easeOutCubic(t) {
        return 1 - Math.pow(1 - t, 3);
    }
    
    showMessage(message, type = 'info') {
        const messageEl = document.getElementById('message-display');
        if (!messageEl) return;
        
        messageEl.textContent = message;
        messageEl.className = `message-display message-${type}`;
        messageEl.classList.remove('hidden');
        
        // Auto hide after delay
        setTimeout(() => {
            messageEl.classList.add('hidden');
        }, type === 'error' ? 5000 : 3000);
    }
}

// Initialize controller when DOM is ready
let armController;
document.addEventListener('DOMContentLoaded', () => {
    // Small delay to ensure arm3D is initialized first
    setTimeout(() => {
        armController = new ArmController();
    }, 100);
});
