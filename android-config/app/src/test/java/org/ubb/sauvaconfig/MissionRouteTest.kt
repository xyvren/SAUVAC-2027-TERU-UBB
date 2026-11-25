package org.ubb.sauvaconfig

import org.json.JSONArray
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Test

class MissionRouteTest {
    @Test
    fun encodesRouteInTheFirmwareMissionSchema() {
        val step = newMissionStep(1).copy(
            name = "Menyelam awal",
            action = MissionAction.DIVE,
            power = "20",
            durationSec = "4.5",
            targetCm = "50",
            motors = setOf(6, 5),
        )

        val encoded = JSONArray(buildMissionJson(listOf(step)))
        val encodedStep = encoded.getJSONObject(0)

        assertEquals(1, encodedStep.getInt("langkah"))
        assertEquals("Menyelam awal", encodedStep.getString("nama"))
        assertEquals("dive", encodedStep.getString("aksi"))
        assertEquals(20.0, encodedStep.getDouble("daya"), 0.0)
        assertEquals(4.5, encodedStep.getDouble("durasi_detik"), 0.0)
        assertEquals(50.0, encodedStep.getDouble("target_cm"), 0.0)
        assertEquals(5, encodedStep.getJSONArray("motors").getInt(0))
        assertEquals(6, encodedStep.getJSONArray("motors").getInt(1))
    }

    @Test
    fun rejectsInvalidBoundsAndEmptyThrusterSelection() {
        assertEquals(
            "Tambahkan minimal satu langkah.",
            validateMissionSteps(emptyList()),
        )
        assertEquals(
            "Pilih minimal satu thruster T1–T6 untuk langkah 1.",
            validateMissionSteps(listOf(newMissionStep(1).copy(motors = emptySet()))),
        )
        assertEquals(
            "Durasi langkah 1 harus 0,1–600 detik.",
            validateMissionSteps(listOf(newMissionStep(1).copy(durationSec = "0"))),
        )
        assertEquals(
            "Jalur maksimal berisi 32 langkah.",
            validateMissionSteps(List(33) { newMissionStep(it + 1) }),
        )
        assertEquals(
            "Nama langkah 1 harus 1–35 byte tanpa karakter kontrol.",
            validateMissionSteps(
                listOf(newMissionStep(1).copy(name = "A".repeat(34) + "é")),
            ),
        )
    }

    @Test
    fun uploadValidationEnforcesJsonSizeAndApPassword() {
        assertThrows(IllegalArgumentException::class.java) {
            validateMissionUpload("[]", "short")
        }
        assertThrows(IllegalArgumentException::class.java) {
            validateMissionUpload("x".repeat(MAX_MISSION_JSON_BYTES + 1), "123456789012")
        }
    }

    @Test
    fun acceptsValidMissionUploadAndUtf8NameWithinFirmwareLimit() {
        val steps = listOf(newMissionStep(1).copy(name = "Menyelam 🌊"))

        assertNull(validateMissionSteps(steps))
        validateMissionUpload(buildMissionJson(steps), "123456789012")
    }
}
