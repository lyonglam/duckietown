from flask import Flask, request, jsonify
from flask_cors import CORS
import time

app = Flask(__name__)

# CRITICAL: Enable CORS to allow the Single Page Interface to send cross-origin requests
CORS(app, resources={r"/*": {"origins": "*"}})

def execute_bot_command(command):
    """
    Interfaces with the Duckiebot low-level control loops.
    """
    print(f"[EXEC] Executing direction: {command}")
    
    # Map the discrete command outputs exported from your A* routing engine
    if command == "straight":
        # Code to engage low-level PID lane followers forward
        pass
    elif command == "left":
        # Code to execute a physical 90-degree left turn sequence
        pass
    elif command == "right":
        # Code to execute a physical 90-degree right turn sequence
        pass
    elif command == "stop":
        # Code to execute an emergency stop or terminal halt sequence
        pass
    else:
        print(f"[WARN] Unknown command received: {command}")

@app.route('/navigate', methods=['POST'])
def receive_directions():
    """
    Endpoint that handles direct serialization payloads from the web interface.
    """
    data = request.get_json()
    
    if not data or 'commands' not in data:
        return jsonify({"status": "rejected", "error": "Missing 'commands' array"}), 400
    
    path_list = data['commands']
    print(f"[RX] Successfully received route payload: {path_list}")
    
    # Process the turn instructions array sequentially
    for direction in path_list:
        execute_bot_command(direction)
        
        # Add buffer/polling logic depending on how long your robot takes to complete a tile grid transit
        time.sleep(1.5) 
        
    return jsonify({"status": "executed", "commands_processed": len(path_list)}), 200

if __name__ == '__main__':
    # Bind to 0.0.0.0 to listen on the local network interface, matching port 8083
    app.run(host='0.0.0.0', port=8083, debug=False)