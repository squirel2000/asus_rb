#!/usr/bin/env python3
from flask import Flask, request, jsonify
import threading
import time
import uuid


ROBOT_API_PORT = 1448
app = Flask(__name__)

# In-memory storage for actions
actions = {}
current_action_id = None

# --- Mock Data ---
def get_mock_pose():
    """Returns a mock pose."""
    return {
        "x": 1.0, "y": 2.0, "z": 0.0,
        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
    }

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
    app.logger.info("Received request for /api/core/slam/v1/localization/pose")
    return jsonify(get_mock_pose())

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


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=ROBOT_API_PORT, debug=True)
