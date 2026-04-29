"""
6DOF Robotic Arm Simulation Server
Flask backend for the 3D arm visualization and control
"""

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import numpy as np
import math

app = Flask(__name__)
CORS(app)

# Default DH parameters for 6DOF arm (in mm and degrees)
DEFAULT_DH_PARAMS = {
    'd1': 100,   # Link 1 offset (base height)
    'a2': 200,   # Link 2 length (shoulder to elbow)
    'a3': 200,   # Link 3 length (elbow to wrist)
    'd4': 50,    # Link 4 offset
    'd5': 50,    # Link 5 offset
    'd6': 50,    # Link 6 offset (end effector)
}

# Joint limits (degrees)
JOINT_LIMITS = {
    'j1': {'min': -180, 'max': 180},  # Base rotation
    'j2': {'min': -90, 'max': 90},    # Shoulder
    'j3': {'min': -135, 'max': 135},  # Elbow
    'j4': {'min': -180, 'max': 180},  # Wrist rotation
    'j5': {'min': -90, 'max': 90},    # Wrist bend
    'j6': {'min': -180, 'max': 180},  # End effector rotation
}

# Current arm state
arm_state = {
    'joints': [0, 0, 0, 0, 0, 0],  # Joint angles in degrees
    'dh_params': DEFAULT_DH_PARAMS.copy(),
}


def dh_matrix(theta, d, a, alpha):
    """
    Compute Denavit-Hartenberg transformation matrix
    theta: joint angle (radians)
    d: link offset
    a: link length
    alpha: link twist (radians)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])


def forward_kinematics(joints, dh_params):
    """
    Calculate forward kinematics for 6DOF arm
    Returns positions of all joints and end effector
    """
    # Convert degrees to radians
    theta = [math.radians(j) for j in joints]
    
    # DH parameters: [theta, d, a, alpha]
    dh = [
        [theta[0], dh_params['d1'], 0, math.pi/2],
        [theta[1], 0, dh_params['a2'], 0],
        [theta[2], 0, dh_params['a3'], 0],
        [theta[3], dh_params['d4'], 0, math.pi/2],
        [theta[4], dh_params['d5'], 0, -math.pi/2],
        [theta[5], dh_params['d6'], 0, 0],
    ]
    
    # Calculate transformation matrices
    positions = [[0, 0, 0]]  # Base position
    T = np.eye(4)
    
    for params in dh:
        T = T @ dh_matrix(*params)
        positions.append(T[:3, 3].tolist())
    
    return positions


@app.route('/')
def index():
    """Render main page"""
    return render_template('index.html')


@app.route('/api/state', methods=['GET'])
def get_state():
    """Get current arm state"""
    positions = forward_kinematics(arm_state['joints'], arm_state['dh_params'])
    return jsonify({
        'joints': arm_state['joints'],
        'dh_params': arm_state['dh_params'],
        'joint_limits': JOINT_LIMITS,
        'positions': positions
    })


@app.route('/api/joints', methods=['POST'])
def set_joints():
    """Set joint angles"""
    data = request.json
    if 'joints' in data:
        joints = data['joints']
        # Validate and clamp joint values
        for i, (key, limits) in enumerate(JOINT_LIMITS.items()):
            if i < len(joints):
                joints[i] = max(limits['min'], min(limits['max'], joints[i]))
        arm_state['joints'] = joints[:6]
    
    positions = forward_kinematics(arm_state['joints'], arm_state['dh_params'])
    return jsonify({
        'joints': arm_state['joints'],
        'positions': positions
    })


@app.route('/api/dh_params', methods=['POST'])
def set_dh_params():
    """Set DH parameters"""
    data = request.json
    for key in DEFAULT_DH_PARAMS:
        if key in data:
            arm_state['dh_params'][key] = float(data[key])
    
    positions = forward_kinematics(arm_state['joints'], arm_state['dh_params'])
    return jsonify({
        'dh_params': arm_state['dh_params'],
        'positions': positions
    })


@app.route('/api/reset', methods=['POST'])
def reset_arm():
    """Reset arm to default state"""
    arm_state['joints'] = [0, 0, 0, 0, 0, 0]
    arm_state['dh_params'] = DEFAULT_DH_PARAMS.copy()
    
    positions = forward_kinematics(arm_state['joints'], arm_state['dh_params'])
    return jsonify({
        'joints': arm_state['joints'],
        'dh_params': arm_state['dh_params'],
        'positions': positions
    })


def inverse_kinematics(target_x, target_y, target_z, dh_params, gripper_offset=120):
    """
    Calculate inverse kinematics for 6DOF arm
    Target position is where the gripper fingertips should reach
    
    Coordinate system (based on DH FK):
    - When all joints = 0, arm extends along +X axis
    - Y: up (positive)
    - theta1 rotates base around Y axis
    
    gripper_offset: distance from end effector frame to fingertips (~120mm for new gripper)
    """
    # Extract DH parameters
    d1 = dh_params['d1']      # Base height
    a2 = dh_params['a2']      # Upper arm length
    a3 = dh_params['a3']      # Forearm length
    d4 = dh_params['d4']
    d5 = dh_params['d5']
    d6 = dh_params['d6']
    
    # Wrist to gripper tip length (vertical when pointing down)
    wrist_to_tip = d4 + d5 + d6 + gripper_offset
    
    # === Joint 1: Base rotation ===
    # Arm extends along X when theta1=0, so rotate from X-axis
    theta1 = math.atan2(target_z, target_x)
    
    # === Calculate wrist center position ===
    # Gripper points down, so wrist is directly above target
    wrist_x = target_x
    wrist_y = target_y + wrist_to_tip
    wrist_z = target_z
    
    # Distance from base center to wrist in horizontal plane (XZ)
    r_wrist = math.sqrt(wrist_x**2 + wrist_z**2)
    
    # Height from shoulder (top of d1) to wrist
    h_wrist = wrist_y - d1
    
    # Distance from shoulder to wrist (in the arm's plane)
    L = math.sqrt(r_wrist**2 + h_wrist**2)
    
    # Check reachability
    max_reach = a2 + a3
    min_reach = abs(a2 - a3) + 5
    
    if L > max_reach:
        return None, f"OVERRANGE: Khoảng cách {L:.0f}mm vượt quá tầm với {max_reach:.0f}mm"
    if L < min_reach:
        return None, f"OVERRANGE: Quá gần, khoảng cách {L:.0f}mm < {min_reach:.0f}mm"
    
    try:
        # === Joint 3: Elbow angle ===
        # Law of cosines: L² = a2² + a3² - 2*a2*a3*cos(π - θ3)
        # cos(π - θ3) = (a2² + a3² - L²) / (2*a2*a3)
        cos_elbow = (a2**2 + a3**2 - L**2) / (2 * a2 * a3)
        cos_elbow = max(-1, min(1, cos_elbow))
        
        # Elbow angle (interior angle at elbow)
        elbow_interior = math.acos(cos_elbow)
        theta3 = elbow_interior - math.pi  # Convert to joint angle (negative = elbow down)
        
        # === Joint 2: Shoulder angle ===
        # Angle from horizontal to the wrist
        phi = math.atan2(h_wrist, r_wrist)
        
        # Angle at shoulder in the triangle (shoulder-elbow-wrist)
        cos_shoulder = (a2**2 + L**2 - a3**2) / (2 * a2 * L)
        cos_shoulder = max(-1, min(1, cos_shoulder))
        psi = math.acos(cos_shoulder)
        
        # theta2: angle from horizontal
        # Positive theta2 = arm goes up
        theta2 = phi + psi
        
        # === Joints 4, 5, 6: Wrist orientation ===
        # We want gripper pointing straight down (along -Y)
        # The cumulative arm angle is theta2 + theta3
        # theta5 compensates to make end effector vertical
        arm_pitch = theta2 + theta3
        theta5 = -arm_pitch - math.pi/2  # Point gripper down
        
        theta4 = 0  # No wrist roll
        theta6 = 0  # No end effector rotation
        
        # Convert to degrees
        joints = [
            math.degrees(theta1),
            math.degrees(theta2),
            math.degrees(theta3),
            math.degrees(theta4),
            math.degrees(theta5),
            math.degrees(theta6)
        ]
        
        # Validate joint limits
        for i, (key, limits) in enumerate(JOINT_LIMITS.items()):
            if i < len(joints):
                if joints[i] < limits['min'] or joints[i] > limits['max']:
                    return None, f"OVERRANGE: Góc khớp {i+1} = {joints[i]:.1f}° vượt giới hạn [{limits['min']}°, {limits['max']}°]"
        
        return joints, None
        
    except Exception as e:
        return None, f"Lỗi tính toán IK: {str(e)}"


@app.route('/api/inverse_kinematics', methods=['POST'])
def calculate_ik():
    """Calculate inverse kinematics for a target position"""
    data = request.json
    
    target_x = float(data.get('target_x', 0))
    target_y = float(data.get('target_y', 200))
    target_z = float(data.get('target_z', 0))
    
    # Use provided DH params or current state
    dh_params = data.get('dh_params', arm_state['dh_params'])
    
    print(f"IK Target: x={target_x}, y={target_y}, z={target_z}")
    
    joints, error_msg = inverse_kinematics(target_x, target_y, target_z, dh_params)
    
    if joints is None:
        print(f"IK Failed: {error_msg}")
        return jsonify({
            'success': False,
            'message': error_msg
        })
    
    # Update arm state
    arm_state['joints'] = joints
    positions = forward_kinematics(joints, arm_state['dh_params'])
    
    # Debug: print computed joints and end effector position
    print(f"IK Joints: {[f'{j:.1f}' for j in joints]}")
    end_pos = positions[-1]
    print(f"FK End Effector: x={end_pos[0]:.1f}, y={end_pos[1]:.1f}, z={end_pos[2]:.1f}")
    # Fingertip position (99mm below end effector when pointing down)
    print(f"Expected Fingertip: x={end_pos[0]:.1f}, y={end_pos[1]-99:.1f}, z={end_pos[2]:.1f}")
    
    return jsonify({
        'success': True,
        'joints': joints,
        'positions': positions,
        'target': {'x': target_x, 'y': target_y, 'z': target_z}
    })


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5001)
