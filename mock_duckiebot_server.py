#!/usr/bin/env python3
# mock_duckiebot_server.py
# Run this on your computer to pretend to be the Duckiebot.
# It listens on port 8083 and prints the turn list it receives from the App.

from http.server import BaseHTTPRequestHandler, HTTPServer
import json

class MockDuckiebotHandler(BaseHTTPRequestHandler):
    # This handles the CORS policy so the web/mobile app is allowed to send data
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    # This catches the JSON turn list from Person 2's app
    def do_POST(self):
        if self.path == '/turn_list':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                # Decode the JSON payload sent from the app
                data = json.loads(post_data.decode('utf-8'))
                
                print("\n" + "="*50)
                print("🦆 BEEP BOOP! RECEIVED NEW MISSION FROM APP 🦆")
                print("="*50)
                
                # Print out the turns nicely so you can verify them
                turn_list = data.get('turn_list', [])
                for i, turn in enumerate(turn_list):
                    print(f"Step {i+1}: Intersection {turn['intersection']} -> Action: {turn['action'].upper()} (In: {turn['h_in']}, Out: {turn['h_out']})")
                
                print("="*50 + "\n")

                # Tell the app "I got it successfully!"
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success", "message": "Turn list received by Mock Duckiebot"}).encode('utf-8'))
                
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                print(f"Error parsing data: {e}")
        else:
            self.send_response(404)
            self.end_headers()

def run(port=8083):
    server_address = ('', port)
    httpd = HTTPServer(server_address, MockDuckiebotHandler)
    print(f"🤖 Mock Duckiebot Server is listening on port {port}...")
    print("Open Person 2's app, calculate a path, and hit 'Send to Duck4'!")
    print("Press Ctrl+C to stop the server.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print("Server stopped.")

if __name__ == '__main__':
    run()