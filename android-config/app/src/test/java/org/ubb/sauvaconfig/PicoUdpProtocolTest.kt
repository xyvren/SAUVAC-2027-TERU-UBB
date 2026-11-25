package org.ubb.sauvaconfig

import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Test
import java.nio.charset.StandardCharsets

class PicoUdpProtocolTest {
    @Test
    fun encodesRequestIdAndOperationWithinFirmwareLimit() {
        val request = JSONObject().put("axis", "depth")
        val bytes = PicoUdpProtocol.encodeRequest(7, "get", request)
        val json = JSONObject(String(bytes, StandardCharsets.UTF_8))

        assertEquals(7, json.getInt("req_id"))
        assertEquals("get", json.getString("op"))
        assertEquals("depth", json.getString("axis"))
        assert(bytes.size <= PicoUdpProtocol.MAX_REQUEST_BYTES)
    }

    @Test
    fun rejectsOversizedRequest() {
        val fields = JSONObject().put("value", "x".repeat(400))
        assertThrows(IllegalArgumentException::class.java) {
            PicoUdpProtocol.encodeRequest(1, "set", fields)
        }
    }

    @Test
    fun rejectsMismatchedRequestIdsAndUnknownFirmwareProtocol() {
        val wrongId = JSONObject().put("req_id", 2).put("ok", true)
        assertThrows(IllegalArgumentException::class.java) {
            PicoUdpProtocol.decodeResponse(
                wrongId.toString().toByteArray(),
                requestId = 1,
            )
        }

        val wrongVersion = JSONObject()
            .put("req_id", 1)
            .put("op", "config")
            .put("protocol_version", 2)
            .put("ok", true)
        assertThrows(IllegalArgumentException::class.java) {
            PicoUdpProtocol.decodeResponse(
                wrongVersion.toString().toByteArray(),
                requestId = 1,
                isGet = true,
            )
        }
    }

    @Test
    fun stagesOnlyChangedPidAndDoesNotPersistIt() {
        val current = defaultConfig()
        val changed = current.copy(
            depthPid = PidDraft(kp = "0.7", ki = "0.0", kd = "0.015"),
        )
        val operations = buildStageOperations(current, changed)

        assertEquals(1, operations.size)
        assertEquals("set_pid", operations.single().operation)
        assertEquals("depth", operations.single().fields.getString("axis"))
        assertFalse(operations.any { it.operation == "save" })
    }

    @Test
    fun refusesAutorunTimingChangesWhileAutorunIsEnabled() {
        val current = defaultConfig().copy(autorunEnabled = true)
        val changed = current.copy(autorunDelay = "15")

        assertThrows(IllegalArgumentException::class.java) {
            buildStageOperations(current, changed)
        }
    }

    @Test
    fun stagesDepthStartupCalibrationWithoutEnablingImuLeveling() {
        val operations = buildStageOperations(
            defaultConfig(),
            defaultConfig().copy(autoDepthTareOnBoot = true),
        )

        assertEquals(1, operations.size)
        assertEquals("set_boot_calibration", operations.single().operation)
        assertEquals(true, operations.single().fields.getBoolean("depth_tare"))
        assertEquals(false, operations.single().fields.getBoolean("imu_level"))
    }

    @Test
    fun stagesImuStartupLevelingWithoutEnablingDepthTare() {
        val operations = buildStageOperations(
            defaultConfig(),
            defaultConfig().copy(autoImuLevelOnBoot = true),
        )

        assertEquals(1, operations.size)
        assertEquals("set_boot_calibration", operations.single().operation)
        assertEquals(false, operations.single().fields.getBoolean("depth_tare"))
        assertEquals(true, operations.single().fields.getBoolean("imu_level"))
    }

    private fun defaultConfig() = AuvConfiguration(
        powerLimit = "25",
        deadbandUs = "25",
        balanceHoriz = "0.0",
        balanceVert = "0.0",
        depthPid = PidDraft("0.6", "0.0", "0.015"),
        rollPid = PidDraft("0.8", "0.0", "0.008"),
        autorunEnabled = false,
        autorunDelay = "10",
        autorunTare = "3",
        autorunLevel = "2",
        autoDepthTareOnBoot = false,
        autoImuLevelOnBoot = false,
        thrusters = (1..6).map { id ->
            ThrusterDraft(
                id = id,
                pin = id + 1,
                reverse = false,
                trim = "1.0",
                pulseMin = "1000",
                pulseNeutral = "1500",
                pulseMax = "2000",
            )
        },
    )
}
