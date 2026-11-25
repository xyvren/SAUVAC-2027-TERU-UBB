package org.ubb.sauvaconfig

import org.json.JSONObject

internal data class PidDraft(
    val kp: String,
    val ki: String,
    val kd: String,
)

internal data class ThrusterDraft(
    val id: Int,
    val pin: Int,
    val reverse: Boolean,
    val trim: String,
    val pulseMin: String,
    val pulseNeutral: String,
    val pulseMax: String,
)

internal data class AuvConfiguration(
    val powerLimit: String,
    val deadbandUs: String,
    val balanceHoriz: String,
    val balanceVert: String,
    val depthPid: PidDraft,
    val rollPid: PidDraft,
    val autorunEnabled: Boolean,
    val autorunDelay: String,
    val autorunTare: String,
    val autorunLevel: String,
    val autoDepthTareOnBoot: Boolean,
    val autoImuLevelOnBoot: Boolean,
    val thrusters: List<ThrusterDraft>,
)

internal data class DeviceStatus(
    val armed: Boolean,
    val fsmState: Int,
    val imuReady: Boolean,
    val pressureReady: Boolean,
    val depthCalibrated: Boolean,
    val sensorAgeMs: Long,
    val depthCm: Double,
    val pressureMbar: Double,
    val pitchDeg: Double,
    val rollDeg: Double,
    val temperatureC: Double,
    val thrusterPulsesUs: List<Int>,
) {
    val idleAndDisarmed: Boolean get() = !armed && fsmState == 0
    val sensorsFresh: Boolean get() = sensorAgeMs in 0..500
}

internal data class FirmwareSnapshot(
    val settings: AuvConfiguration,
    val status: DeviceStatus,
    val dirty: Boolean,
)

internal data class ConfigOperation(val operation: String, val fields: JSONObject)

internal fun parseFirmwareSnapshot(response: JSONObject): FirmwareSnapshot {
    val settings = response.getJSONObject("settings")
    val pid = settings.getJSONObject("pid")
    val depth = pid.getJSONObject("depth")
    val roll = pid.getJSONObject("roll")
    val autorun = settings.getJSONObject("autorun")
    val startupCalibration = settings.optJSONObject("startup_calibration")
    val legacyBootCalibration = startupCalibration?.optBoolean("enabled", false) ?: false
    val status = response.getJSONObject("status")
    val thrusters = settings.getJSONArray("thrusters")
    require(thrusters.length() == 6) { "Firmware returned an unexpected thruster count." }

    val config = AuvConfiguration(
        powerLimit = settings.getInt("power_limit").toString(),
        deadbandUs = settings.getInt("esc_deadband_us").toString(),
        balanceHoriz = settings.getDouble("balance_horiz").toString(),
        balanceVert = settings.getDouble("balance_vert").toString(),
        depthPid = parsePid(depth),
        rollPid = parsePid(roll),
        autorunEnabled = autorun.getBoolean("enabled"),
        autorunDelay = autorun.getInt("delay_sec").toString(),
        autorunTare = autorun.getInt("tare_sec").toString(),
        autorunLevel = autorun.getInt("level_sec").toString(),
        autoDepthTareOnBoot = startupCalibration?.optBoolean("depth_tare", legacyBootCalibration) ?: false,
        autoImuLevelOnBoot = startupCalibration?.optBoolean("imu_level", legacyBootCalibration) ?: false,
        thrusters = List(thrusters.length()) { index ->
            val item = thrusters.getJSONObject(index)
            ThrusterDraft(
                id = item.getInt("id"),
                pin = item.getInt("pin"),
                reverse = item.getBoolean("reverse"),
                trim = item.getDouble("trim").toString(),
                pulseMin = item.getInt("pulse_min").toString(),
                pulseNeutral = item.getInt("pulse_neutral").toString(),
                pulseMax = item.getInt("pulse_max").toString(),
            )
        },
    )
    val deviceStatus = DeviceStatus(
        armed = status.getBoolean("armed"),
        fsmState = status.getInt("fsm_state"),
        imuReady = status.getBoolean("imu_ready"),
        pressureReady = status.getBoolean("pressure_ready"),
        depthCalibrated = status.getBoolean("depth_calibrated"),
        sensorAgeMs = status.getLong("sensor_age_ms"),
        depthCm = status.getDouble("depth_cm"),
        pressureMbar = status.getDouble("pressure_mbar"),
        pitchDeg = status.getDouble("pitch"),
        rollDeg = status.getDouble("roll"),
        temperatureC = status.getDouble("temperature_c"),
        thrusterPulsesUs = status.getJSONArray("pulses").let { pulses ->
            require(pulses.length() == 6) { "Firmware returned an unexpected thruster pulse count." }
            List(pulses.length()) { pulses.getInt(it) }
        },
    )
    return FirmwareSnapshot(config, deviceStatus, response.getBoolean("dirty"))
}

internal fun validateConfiguration(config: AuvConfiguration): String? {
    if (config.thrusters.size != 6) return "Konfigurasi harus memiliki enam thruster."
    val powerLimit = config.powerLimit.toIntOrNull()
    if (powerLimit == null || powerLimit !in 1..100) return "Batas daya harus 1–100%."
    val deadbandUs = config.deadbandUs.toIntOrNull()
    if (deadbandUs == null || deadbandUs !in 0..80) return "Deadband harus 0–80 µs."
    if (config.balanceHoriz.toFiniteDouble()?.let { it in -20.0..20.0 } != true ||
        config.balanceVert.toFiniteDouble()?.let { it in -20.0..20.0 } != true) {
        return "Balance harus berada di antara −20% dan +20%."
    }
    if (!validPid(config.depthPid) || !validPid(config.rollPid)) {
        return "Gain PID harus berupa angka finite antara 0 dan 100."
    }
    val delay = config.autorunDelay.toIntOrNull()
    if (delay == null || delay !in 3..60) return "Delay autorun harus 3–60 detik."
    val tare = config.autorunTare.toIntOrNull()
    val level = config.autorunLevel.toIntOrNull()
    if (tare == null || tare !in 1..delay || level == null || level !in 1..delay) {
        return "Tare dan level harus 1 detik sampai nilai delay."
    }
    val pins = mutableSetOf<Int>()
    for (thruster in config.thrusters) {
        val trim = thruster.trim.toFiniteDouble()
        val pulseMin = thruster.pulseMin.toIntOrNull()
        val pulseNeutral = thruster.pulseNeutral.toIntOrNull()
        val pulseMax = thruster.pulseMax.toIntOrNull()
        if (thruster.id !in 1..6 || thruster.pin !in 0..29 ||
            !pins.add(thruster.pin) || trim == null || trim !in 0.1..2.5 ||
            pulseMin == null || pulseNeutral == null || pulseMax == null ||
            pulseMin !in 800..1400 || pulseNeutral !in 1400..1600 ||
            pulseMax !in 1600..2200 || pulseMin >= pulseNeutral || pulseNeutral >= pulseMax) {
            return "Periksa pin, trim, dan urutan pulsa setiap thruster."
        }
    }
    return null
}

internal fun buildStageOperations(
    current: AuvConfiguration,
    draft: AuvConfiguration,
): List<ConfigOperation> {
    validateConfiguration(draft)?.let { throw IllegalArgumentException(it) }
    val operations = mutableListOf<ConfigOperation>()
    if (current.depthPid != draft.depthPid) {
        operations += ConfigOperation("set_pid", pidFields("depth", draft.depthPid))
    }
    if (current.rollPid != draft.rollPid) {
        operations += ConfigOperation("set_pid", pidFields("roll", draft.rollPid))
    }
    if (current.powerLimit != draft.powerLimit) {
        operations += ConfigOperation(
            "set_power_limit",
            JSONObject().put("percent", draft.powerLimit.toInt()),
        )
    }
    if (current.deadbandUs != draft.deadbandUs) {
        operations += ConfigOperation(
            "set_deadband",
            JSONObject().put("microseconds", draft.deadbandUs.toInt()),
        )
    }
    if (current.balanceHoriz != draft.balanceHoriz) {
        operations += ConfigOperation(
            "set_balance",
            JSONObject().put("axis", "horiz").put("percent", draft.balanceHoriz.toDouble()),
        )
    }
    if (current.balanceVert != draft.balanceVert) {
        operations += ConfigOperation(
            "set_balance",
            JSONObject().put("axis", "vert").put("percent", draft.balanceVert.toDouble()),
        )
    }

    current.thrusters.zip(draft.thrusters).forEach { (old, new) ->
        val fields = JSONObject().put("id", new.id)
        if (old.reverse != new.reverse) fields.put("reverse", new.reverse)
        if (old.trim != new.trim) fields.put("trim", new.trim.toDouble())
        if (old.pulseMin != new.pulseMin) fields.put("pulse_min", new.pulseMin.toInt())
        if (old.pulseNeutral != new.pulseNeutral) fields.put("pulse_neutral", new.pulseNeutral.toInt())
        if (old.pulseMax != new.pulseMax) fields.put("pulse_max", new.pulseMax.toInt())
        if (fields.length() > 1) operations += ConfigOperation("set_thruster", fields)
    }

    if (current.autorunDelay != draft.autorunDelay ||
        current.autorunTare != draft.autorunTare ||
        current.autorunLevel != draft.autorunLevel) {
        require(!draft.autorunEnabled) {
            "Matikan autorun melalui USB sebelum mengubah timing."
        }
        operations += ConfigOperation(
            "set_autorun_timing",
            JSONObject()
                .put("delay_sec", draft.autorunDelay.toInt())
                .put("tare_sec", draft.autorunTare.toInt())
                .put("level_sec", draft.autorunLevel.toInt()),
        )
    }
    if (current.autoDepthTareOnBoot != draft.autoDepthTareOnBoot ||
        current.autoImuLevelOnBoot != draft.autoImuLevelOnBoot) {
        operations += ConfigOperation(
            "set_boot_calibration",
            JSONObject()
                .put("depth_tare", draft.autoDepthTareOnBoot)
                .put("imu_level", draft.autoImuLevelOnBoot),
        )
    }
    return operations
}

private fun parsePid(json: JSONObject) = PidDraft(
    kp = json.getDouble("kp").toString(),
    ki = json.getDouble("ki").toString(),
    kd = json.getDouble("kd").toString(),
)

private fun pidFields(axis: String, gains: PidDraft) = JSONObject()
    .put("axis", axis)
    .put("kp", gains.kp.toDouble())
    .put("ki", gains.ki.toDouble())
    .put("kd", gains.kd.toDouble())

private fun validPid(gains: PidDraft): Boolean =
    listOf(gains.kp, gains.ki, gains.kd).all { it.toFiniteDouble()?.let { n -> n in 0.0..100.0 } == true }

private fun String.toFiniteDouble(): Double? =
    toDoubleOrNull()?.takeIf { it.isFinite() }
