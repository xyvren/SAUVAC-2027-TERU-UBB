package org.ubb.sauvaconfig

import org.json.JSONArray
import org.json.JSONObject
import java.nio.charset.StandardCharsets

internal enum class MissionAction(
    val wireValue: String,
    val label: String,
    val defaultMotors: Set<Int>,
) {
    DIVE("dive", "Menyelam", setOf(5, 6)),
    FORWARD("maju", "Maju", setOf(1, 2, 3, 4)),
    BACKWARD("mundur", "Mundur", setOf(1, 2, 3, 4)),
    TURN("putar", "Putar", setOf(1, 2, 3, 4)),
    SWAY_LEFT("kiri", "Geser kiri", setOf(1, 2, 3, 4)),
    SWAY_RIGHT("kanan", "Geser kanan", setOf(1, 2, 3, 4)),
    HOVER("hover", "Diam / hover", setOf(5, 6)),
    SURFACE("surface", "Naik ke permukaan", setOf(5, 6)),
}

internal data class MissionStepDraft(
    val name: String,
    val action: MissionAction,
    val power: String,
    val durationSec: String,
    val targetCm: String,
    val motors: Set<Int>,
)

internal fun newMissionStep(index: Int) = MissionStepDraft(
    name = "Langkah $index",
    action = MissionAction.FORWARD,
    power = "15",
    durationSec = "5",
    targetCm = "0",
    motors = MissionAction.FORWARD.defaultMotors,
)

internal fun validateMissionSteps(steps: List<MissionStepDraft>): String? {
    if (steps.isEmpty()) return "Tambahkan minimal satu langkah."
    if (steps.size > 32) return "Jalur maksimal berisi 32 langkah."

    steps.forEachIndexed { index, step ->
        val name = step.name.trim()
        val nameBytes = name.toByteArray(StandardCharsets.UTF_8).size
        if (nameBytes !in 1..35 || step.name.any { Character.isISOControl(it) }) {
            return "Nama langkah ${index + 1} harus 1–35 byte tanpa karakter kontrol."
        }
        val power = step.power.toFiniteNumber()
        if (power == null || power !in 0.0..100.0) {
            return "Daya langkah ${index + 1} harus 0–100%."
        }
        val duration = step.durationSec.toFiniteNumber()
        if (duration == null || duration !in 0.1..600.0) {
            return "Durasi langkah ${index + 1} harus 0,1–600 detik."
        }
        val target = step.targetCm.toFiniteNumber()
        if (target == null || target !in 0.0..140.0) {
            return "Target kedalaman langkah ${index + 1} harus 0–140 cm."
        }
        if (step.motors.isEmpty() || step.motors.any { it !in 1..6 }) {
            return "Pilih minimal satu thruster T1–T6 untuk langkah ${index + 1}."
        }
    }

    val json = encodeMissionSteps(steps)
    if (json.toByteArray(StandardCharsets.UTF_8).size > MAX_MISSION_JSON_BYTES) {
        return "Jalur terlalu besar untuk Pico W (maks. 3.500 byte)."
    }
    return null
}

internal fun buildMissionJson(steps: List<MissionStepDraft>): String {
    validateMissionSteps(steps)?.let { throw IllegalArgumentException(it) }
    return encodeMissionSteps(steps)
}

private fun encodeMissionSteps(steps: List<MissionStepDraft>): String {
    val array = JSONArray()
    steps.forEachIndexed { index, step ->
        val motors = JSONArray()
        step.motors.sorted().forEach(motors::put)
        array.put(
            JSONObject()
                .put("langkah", index + 1)
                .put("nama", step.name.trim())
                .put("aksi", step.action.wireValue)
                .put("daya", step.power.toDoubleOrNull())
                .put("durasi_detik", step.durationSec.toDoubleOrNull())
                .put("target_cm", step.targetCm.toDoubleOrNull())
                .put("motors", motors),
        )
    }
    return array.toString()
}

internal const val MAX_MISSION_JSON_BYTES = 3_500

private fun String.toFiniteNumber(): Double? =
    toDoubleOrNull()?.takeIf(Double::isFinite)
