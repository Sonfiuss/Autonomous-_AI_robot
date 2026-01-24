"""
Flask server for Robot Path Planning Simulator
"""
from flask import Flask, render_template, jsonify, request
import json

app = Flask(__name__)

# Default robot configuration
DEFAULT_CONFIG = {
    "robot": {
        "outer_radius": 19.0,  # cm - khoảng cách tâm đến cạnh xa nhất
        "inner_radius": 14.4,  # cm - khoảng cách tâm đến cạnh gần nhất
        "max_velocity": 1.0,   # m/s
        "max_omega": 1.0,      # rad/s
        "safety_margin": 2.0   # cm - bán kính an toàn thêm
    },
    "grid": {
        "width": 800,          # pixels
        "height": 600,         # pixels
        "cell_size": 20,       # pixels per cell
        "scale": 2.0           # pixels per cm (1cm = 2px)
    },
    "simulation": {
        "update_rate": 60,     # FPS
        "path_color": "#00ff00",
        "robot_color": "#3498db",
        "obstacle_color": "#e74c3c"
    }
}


@app.route('/')
def index():
    """Render main simulator page"""
    return render_template('index.html', config=DEFAULT_CONFIG)


@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration"""
    return jsonify(DEFAULT_CONFIG)


@app.route('/api/config', methods=['POST'])
def update_config():
    """Update configuration"""
    data = request.get_json()
    # Merge with default config
    for key in data:
        if key in DEFAULT_CONFIG:
            DEFAULT_CONFIG[key].update(data[key])
    return jsonify({"status": "ok", "config": DEFAULT_CONFIG})


@app.route('/api/pathfind', methods=['POST'])
def calculate_path():
    """Calculate path from start to goal avoiding obstacles"""
    data = request.get_json()
    start = data.get('start')
    goal = data.get('goal')
    obstacles = data.get('obstacles', [])
    
    # TODO: Implement A* pathfinding on server side
    # For now, return direct path
    return jsonify({
        "status": "ok",
        "path": [start, goal],
        "message": "Direct path (pathfinding not yet implemented)"
    })


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
