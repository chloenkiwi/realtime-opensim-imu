"""WebSocket client for receiving SageMotion IMU data."""

import json
import logging
import time

import websocket

logger = logging.getLogger(__name__)


class DataStreamClient:
    """Receive streamed SageMotion sensor data and place it on a queue."""

    def __init__(self, ip_address, queue, port=5678, requested_data=None):
        self.ip_address = ip_address
        self.port = port
        self.queue = queue
        self.requested_data = requested_data
        self.ws_url = f"ws://{self.ip_address}:{self.port}/"

    def _on_message(self, ws, message):
        data = json.loads(message)
        if all(sensor for sensor in data["raw_data"]):
            self.queue.put((0, data))
        else:
            logger.warning("Received a packet with empty sensor data.")

    def _on_error(self, ws, error):
        logger.error("WebSocket error: %s", error)

    def _on_close(self, ws, close_status_code, close_msg):
        logger.info(
            "WebSocket closed (status=%s, message=%s)",
            close_status_code,
            close_msg,
        )

    def _send_request(self, ws):
        ws.send(json.dumps(self.requested_data))
        logger.info("Sent the requested-data specification to the server.")
        time.sleep(2)

    def _on_open(self, ws):
        logger.info("WebSocket connection opened.")
        self._send_request(ws)

    def _on_reconnect(self, ws):
        logger.info("WebSocket connection restored.")
        self._send_request(ws)

    def run_forever(self):
        """Connect to the configured server and reconnect after interruptions."""
        ws = websocket.WebSocketApp(
            self.ws_url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
            on_reconnect=self._on_reconnect,
        )
        ws.run_forever(reconnect=1)
