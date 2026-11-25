#!/usr/bin/env python3
"""Compatibility redirect for clients that still open the legacy server."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8081
SETTINGS_URL = "http://127.0.0.1:8080/#konfigurasi"


class LegacyRedirectHandler(BaseHTTPRequestHandler):
    def _redirect(self):
        self.send_response(302)
        self.send_header("Location", SETTINGS_URL)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def do_GET(self):
        self._redirect()

    def do_HEAD(self):
        self._redirect()

    def do_POST(self):
        self.send_error(410, "Use the unified application on port 8080.")

    def log_message(self, format, *args):
        pass


def run_server():
    with ThreadingHTTPServer(("127.0.0.1", PORT), LegacyRedirectHandler) as server:
        print(f"Legacy server redirects to {SETTINGS_URL}")
        server.serve_forever()


if __name__ == "__main__":
    run_server()
