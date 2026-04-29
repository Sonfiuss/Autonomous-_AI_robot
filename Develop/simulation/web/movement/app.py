"""
Flask server for Robot Path Planning Simulator
"""
from flask import Flask, render_template, jsonify, request

app = Flask(__name__)

DEFAULT_CONFIG = {
    "robot": {
        "outer_radius": 19.0,
        "inner_radius": 14.4,
        "max_velocity": 1.0,
        "max_omega": 1.0,
        "safety_margin": 2.0
    },
    "grid": {
        "width": 800,
        "height": 600,
        "cell_size": 20,
        "scale": 2.0
    },
    "simulation": {
        "update_rate": 60,
        "path_color": "#00ff00",
        "robot_color": "#3498db",
        "obstacle_color": "#e74c3c"
    }
}


@app.route('/')
def index():
    return render_template('index.html', config=DEFAULT_CONFIG)


@app.route('/api/config', methods=['GET'])
def get_config():
    return jsonify(DEFAULT_CONFIG)


@app.route('/api/config', methods=['POST'])
def update_config():
    data = request.get_json()
    for key in data:
        if key in DEFAULT_CONFIG:
            DEFAULT_CONFIG[key].update(data[key])
    return jsonify({"status": "ok", "config": DEFAULT_CONFIG})


@app.route('/api/pathfind', methods=['POST'])
def calculate_path():
    data = request.get_json()
    start = data.get('start')
    goal = data.get('goal')
    return jsonify({
        "status": "ok",
        "path": [start, goal],
        "message": "Direct path (pathfinding not yet implemented)"
    })


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
