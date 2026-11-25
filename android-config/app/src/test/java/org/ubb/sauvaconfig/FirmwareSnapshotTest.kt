package org.ubb.sauvaconfig

import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Test

class FirmwareSnapshotTest {
    @Test
    fun parsesLiveSensorValuesAndThrusterOutputs() {
        val thrusters = JSONArray()
        repeat(6) { index ->
            thrusters.put(
                JSONObject()
                    .put("id", index + 1)
                    .put("pin", index + 2)
                    .put("reverse", false)
                    .put("trim", 1.0)
                    .put("pulse_min", 1000)
                    .put("pulse_neutral", 1500)
                    .put("pulse_max", 2000),
            )
        }
        val settings = JSONObject()
            .put("power_limit", 25)
            .put("esc_deadband_us", 25)
            .put("balance_horiz", 0.0)
            .put("balance_vert", 0.0)
            .put(
                "pid",
                JSONObject()
                    .put("depth", JSONObject().put("kp", 0.6).put("ki", 0.0).put("kd", 0.015))
                    .put("roll", JSONObject().put("kp", 0.8).put("ki", 0.0).put("kd", 0.008)),
            )
            .put(
                "autorun",
                JSONObject().put("enabled", false).put("delay_sec", 10)
                    .put("tare_sec", 3).put("level_sec", 2),
            )
            .put(
                "startup_calibration",
                JSONObject().put("depth_tare", true).put("imu_level", false),
            )
            .put("thrusters", thrusters)
        val status = JSONObject()
            .put("armed", false)
            .put("fsm_state", 0)
            .put("imu_ready", true)
            .put("pressure_ready", true)
            .put("depth_calibrated", true)
            .put("sensor_age_ms", 100)
            .put("depth_cm", 12.5)
            .put("pressure_mbar", 1014.2)
            .put("pitch", -1.5)
            .put("roll", 2.25)
            .put("temperature_c", 23.4)
            .put("pulses", JSONArray().put(1500).put(1500).put(1500).put(1500).put(1500).put(1500))
        val response = JSONObject()
            .put("settings", settings)
            .put("status", status)
            .put("dirty", false)

        val snapshot = parseFirmwareSnapshot(response)

        assertEquals(12.5, snapshot.status.depthCm, 0.0)
        assertEquals(1014.2, snapshot.status.pressureMbar, 0.0)
        assertEquals(-1.5, snapshot.status.pitchDeg, 0.0)
        assertEquals(2.25, snapshot.status.rollDeg, 0.0)
        assertEquals(23.4, snapshot.status.temperatureC, 0.0)
        assertEquals(List(6) { 1500 }, snapshot.status.thrusterPulsesUs)
        assertEquals(true, snapshot.settings.autoDepthTareOnBoot)
        assertEquals(false, snapshot.settings.autoImuLevelOnBoot)
    }
}
