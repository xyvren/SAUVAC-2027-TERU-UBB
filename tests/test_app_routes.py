import importlib
import sys
import unittest
from unittest.mock import patch


with patch.dict(sys.modules, {"serial": None}):
    app = importlib.import_module("simulasi_auv.app")
app.bridge.shutdown()
app.bridge.worker_thread.join(timeout=1.0)
legacy_server = importlib.import_module("simulasi_auv.server")


class ConfigurationRouteTests(unittest.TestCase):
    def test_config_export_returns_versioned_backup(self):
        response = _JsonCaptureStub()
        response.path = "/api/config/export"
        response.headers = {"Host": "127.0.0.1:8080"}
        app.UnifiedHttpHandler.do_GET(response)
        self.assertTrue(response.data["success"])
        backup = response.data["backup"]
        self.assertEqual(backup["format"], "auv-config-backup")
        self.assertIn("config", backup)

    def test_config_import_rejects_wrong_format_and_stale_version(self):
        response = _JsonCaptureStub()
        with self.assertRaises(ValueError):
            app.UnifiedHttpHandler._handle_post(
                response,
                "/api/config/import",
                {"format": "wrong", "config": {}},
            )
        with self.assertRaises(ValueError):
            app.UnifiedHttpHandler._handle_post(
                response,
                "/api/config/import",
                {
                    "format": "auv-config-backup",
                    "config_version": 1,
                    "config": {},
                },
            )
    def test_legacy_config_paths_redirect_to_settings_tab(self):
        for path in ("/config", "/config.html"):
            with self.subTest(path=path):
                response = _ResponseStub(path)
                app.UnifiedHttpHandler.do_GET(response)
                self.assertEqual(response.status, 302)
                self.assertEqual(response.response_headers["Location"], "/#konfigurasi")
                self.assertEqual(response.response_headers["Cache-Control"], "no-store")

    def test_legacy_server_redirects_without_opening_a_serial_bridge(self):
        response = _ResponseStub("/")
        legacy_server.LegacyRedirectHandler._redirect(response)
        self.assertFalse(hasattr(legacy_server, "bridge"))
        self.assertEqual(response.status, 302)
        self.assertEqual(
            response.response_headers["Location"],
            "http://127.0.0.1:8080/#konfigurasi",
        )

    def test_simulation_physics_accepts_signed_buoyancy_and_model_bounds(self):
        response = _JsonCaptureStub()
        with app.bridge.state_lock:
            original = (app.bridge.sim_mass, app.bridge.sim_net_buoyancy_g, app.bridge.sim_drag_scale)
        try:
            with patch.object(app, "rl_agent", None):
                app.UnifiedHttpHandler._handle_post(
                    response,
                    "/api/physics_config",
                    {"mass": 50, "net_buoyancy_g": -1000, "drag_scale": 5},
                )
                self.assertEqual(response.data["success"], True)
                self.assertEqual(response.data["mass_kg"], 50)
                self.assertEqual(response.data["net_buoyancy_g"], -1000)
                self.assertEqual(response.data["drag_scale"], 5)

                with self.assertRaises(ValueError):
                    app.UnifiedHttpHandler._handle_post(
                        response,
                        "/api/physics_config",
                        {"net_buoyancy_g": -1000.1},
                    )
        finally:
            with app.bridge.state_lock:
                app.bridge.sim_mass, app.bridge.sim_net_buoyancy_g, app.bridge.sim_drag_scale = original


class _ResponseStub:
    def __init__(self, path):
        self.path = path
        self.headers = {"Host": "127.0.0.1:8080"}
        self.status = None
        self.response_headers = {}

    def _local_request_allowed(self):
        return app.UnifiedHttpHandler._local_request_allowed(self)

    def send_response(self, status):
        self.status = status

    def send_header(self, name, value):
        self.response_headers[name] = value

    def end_headers(self):
        pass


class _JsonCaptureStub:
    def _local_request_allowed(self):
        return app.UnifiedHttpHandler._local_request_allowed(self)

    def send_json(self, data, code=200):
        self.data = data
        self.status = code


if __name__ == "__main__":
    unittest.main()
