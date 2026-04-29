"""
REST API Routes

Endpoints cho điều khiển robot qua HTTP.
"""

# from flask import Blueprint, request, jsonify

# api_bp = Blueprint("api", __name__, url_prefix="/api")

# @api_bp.route("/drive", methods=["POST"])
# def drive():
#     """Gửi lệnh di chuyển."""
#     data = request.json
#     vx = data.get("vx", 0)
#     vy = data.get("vy", 0)
#     omega = data.get("omega", 0)
#     # TODO: Gửi đến DriveController
#     return jsonify({"status": "ok"})

# @api_bp.route("/arm", methods=["POST"])
# def arm():
#     """Gửi lệnh cánh tay."""
#     data = request.json
#     # TODO: Gửi đến ArmController
#     return jsonify({"status": "ok"})

# @api_bp.route("/sensors", methods=["GET"])
# def sensors():
#     """Đọc dữ liệu cảm biến."""
#     # TODO: Đọc từ SensorReader
#     return jsonify({"sensors": {}})
