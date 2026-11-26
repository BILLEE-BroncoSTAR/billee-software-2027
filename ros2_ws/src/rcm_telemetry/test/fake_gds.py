"""A stand-in for the RCM's F´ GDS HTTP API, with INA780B readings for the four packs.

    python3 test/fake_gds.py 5099      # serve it, to run battery_node against without the RCM

/channels returns the readings each time it's called with a session (the real GDS returns what
arrived since the last call; the node polls about once a second, so it sees them as fresh).
"""

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PREFIX = "rp2350Deployment."
CHANNELS = {
    268574720 + i: f"inaManager.DRIVE{i + 1}_POWER" for i in range(6)
} | {268574726: "inaManager.ARM_POWER", 268574727: "inaManager.SCIENCE_POWER",
     268574728: "inaManager.LOGIC_POWER", 268582929: "fpManager.ARM_POWER_BOUNDS",
     268509188: "systemResources.CPU"}


def reading(voltage, current=1.0, valid=True):
    return {"timestamp": 1, "valid": valid, "power": voltage * current, "current": current,
            "sourceId": "X", "voltage": voltage}


# DRIVE: 4S at 15.2 V (50 %) on two channels, the others switched off (0 V) or not answering.
# LOGIC 22.8 V (50 % of 20.4-25.2), SCIENCE missing (sensor not answering), ARM 25.2 V (full).
DEFAULT_STATE = {
    "inaManager.DRIVE1_POWER": reading(15.2, 2.0), "inaManager.DRIVE2_POWER": reading(15.1, 3.0),
    "inaManager.DRIVE3_POWER": reading(0.0, 0.0), "inaManager.DRIVE4_POWER": reading(0.0, 0.0),
    "inaManager.DRIVE5_POWER": reading(0.0, 0.0, valid=False),
    "inaManager.DRIVE6_POWER": reading(0.0, 0.0, valid=False),
    "inaManager.LOGIC_POWER": reading(22.8, 1.5),
    "inaManager.SCIENCE_POWER": reading(0.0, 0.0, valid=False),
    "inaManager.ARM_POWER": reading(25.2, 0.5),
    "systemResources.CPU": 3.0,
}


def make_server(port=0, state=None):
    state = dict(DEFAULT_STATE if state is None else state)
    ids = {name: cid for cid, name in CHANNELS.items()}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, body):
            data = json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/session":
                return self._json({"session": "fake-session"})
            if path == "/dictionary/channels":
                return self._json({"dictionary": {str(cid): {"id": cid, "full_name": PREFIX + n}
                                                  for cid, n in CHANNELS.items()}})
            if path == "/channels":
                if "session=" not in self.path:
                    return self._json({"message": {"session": "Session key for fetching data."}})
                return self._json({"history": [{"id": ids[n], "val": v}
                                               for n, v in server.state.items()]})
            self.send_error(404)

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.state = state
    return server


def serve_in_thread(port=0, state=None):
    server = make_server(port, state)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


if __name__ == "__main__":
    srv = make_server(int(sys.argv[1]) if len(sys.argv) > 1 else 5099)
    print(f"fake RCM GDS on http://127.0.0.1:{srv.server_address[1]}", flush=True)
    srv.serve_forever()
