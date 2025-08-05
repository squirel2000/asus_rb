#!/usr/bin/env python3
from flask import Flask, request, jsonify
import threading
import time
import uuid
import random
import logging

class PostOnlyFilter(logging.Filter):
    def filter(self, record):
        # Check if the log record has 'GET' in its message
        if 'GET' in record.getMessage():
            return False
        return True

ROBOT_API_PORT = 1448
app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.addFilter(PostOnlyFilter())

# In-memory storage for actions
actions = {}
current_action_id = None

# --- Mock Data ---
current_pose = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
}
target_pose = None
pose_lock = threading.Lock()

def update_pose_thread():
    """A thread that simulates robot movement by updating its pose."""
    global current_pose, target_pose
    log_counter = 0
    while True:
        with pose_lock:
            if target_pose is None or (
                abs(current_pose['x'] - target_pose['x']) < 0.1 and
                abs(current_pose['y'] - target_pose['y']) < 0.1
            ):
                target_pose = {
                    "x": round(random.uniform(-10.0, 10.0), 2),
                    "y": round(random.uniform(-10.0, 10.0), 2)
                }
                app.logger.info(f"New target set: {target_pose}")

            # Move towards the target
            current_pose['x'] += 0.05 * (target_pose['x'] - current_pose['x'])
            current_pose['y'] += 0.05 * (target_pose['y'] - current_pose['y'])

            if log_counter % 50 == 0:
                app.logger.info(f"Current Pose: x={current_pose['x']:.2f}, y={current_pose['y']:.2f}")
            log_counter += 1

        time.sleep(0.04)  # 25 Hz

# --- Action Simulation Thread ---
def simulate_action(action_id):
    """Simulates the lifecycle of a navigation action."""
    actions[action_id]['status'] = 'running'
    app.logger.info(f"Action {action_id} started.")
    
    # Simulate work for 5 seconds
    time.sleep(5)
    
    # Check if the action was not canceled
    if action_id in actions and actions[action_id]['status'] != 'canceled':
        actions[action_id]['status'] = 'succeeded'
        app.logger.info(f"Action {action_id} succeeded.")
    else:
        app.logger.info(f"Action {action_id} was canceled or removed.")


# --- API Endpoints ---

@app.route('/api/core/slam/v1/localization/pose', methods=['GET'])
def get_pose():
    """Endpoint to get the current robot pose."""
    with pose_lock:
        return jsonify(current_pose)

@app.route('/api/core/motion/v1/actions', methods=['POST'])
def create_action():
    """Endpoint to create a new navigation action."""
    global current_action_id
    action_id = str(uuid.uuid4())
    current_action_id = action_id
    
    actions[action_id] = {
        "id": action_id,
        "status": "pending",
        "request_data": request.json
    }
    
    # Start a background thread to simulate the action
    thread = threading.Thread(target=simulate_action, args=(action_id,))
    thread.daemon = True
    thread.start()
    
    app.logger.info(f"Created action with ID: {action_id}")
    return jsonify({"id": action_id}), 200

@app.route('/api/core/motion/v1/actions/<string:action_id>', methods=['GET'])
def get_action_status(action_id):
    """Endpoint to get the status of an action."""
    if action_id not in actions:
        return jsonify({"error": "Action not found"}), 404
    
    status = actions[action_id].get('status', 'unknown')
    app.logger.info(f"Request for status of action {action_id}. Current status: {status}")
    return jsonify({"state": status})

@app.route('/api/core/motion/v1/actions/:current', methods=['DELETE'])
def cancel_action():
    """Endpoint to cancel the current action."""
    global current_action_id
    if current_action_id and current_action_id in actions:
        actions[current_action_id]['status'] = 'canceled'
        app.logger.info(f"Canceled action {current_action_id}")
        current_action_id = None
        return jsonify({"message": "Action canceled"}), 200
    
    app.logger.warning("Request to cancel an action, but no current action is active.")
    return jsonify({"error": "No active action to cancel"}), 404


@app.route('/api/core/system/v1/laserscan', methods=['GET'])
def get_laserscan():
    """Endpoint to get the current laser scan data."""
    with pose_lock:
        # Generate mock laser scan data
        laser_points = []
        for i in range(360):  # 360 points for a full circle
            laser_points.append({
                "distance": random.uniform(0.1, 10.0),
                "angle": (i - 180) * 3.14159 / 180.0,
                "valid": True
            })

        laserscan_data = {
            "pose": {
                "x": current_pose["x"],
                "y": current_pose["y"],
                "z": 0.0,
                "yaw": 0.0,
                "pitch": 0.0,
                "roll": 0.0
            },
            "laser_points": laser_points
        }
        return jsonify(laserscan_data)


if __name__ == '__main__':
    # Start the pose update thread
    pose_thread = threading.Thread(target=update_pose_thread)
    pose_thread.daemon = True
    pose_thread.start()
    app.run(host='0.0.0.0', port=ROBOT_API_PORT, debug=True)
