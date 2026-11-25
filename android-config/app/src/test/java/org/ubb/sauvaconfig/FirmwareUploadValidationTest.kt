package org.ubb.sauvaconfig

import org.junit.Assert.assertThrows
import org.junit.Test

class FirmwareUploadValidationTest {
    @Test
    fun acceptsBoundedBinWithPrivateAsciiPassword() {
        validateFirmwareUpload(
            image = byteArrayOf(1, 2, 3),
            filename = "sauvav2.bin",
            password = "private-pass-123",
        )
    }

    @Test
    fun rejectsOversizedImageAndUf2Payload() {
        assertThrows(IllegalArgumentException::class.java) {
            validateFirmwareUpload(
                image = ByteArray(MAX_FIRMWARE_IMAGE_BYTES + 1),
                filename = "firmware.bin",
                password = "private-pass-123",
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            validateFirmwareUpload(
                image = "UF2\n".toByteArray(),
                filename = "firmware.bin",
                password = "private-pass-123",
            )
        }
    }

    @Test
    fun rejectsUnsafeFilenameAndInvalidPassword() {
        assertThrows(IllegalArgumentException::class.java) {
            validateFirmwareUpload(
                image = byteArrayOf(1),
                filename = "../firmware.bin",
                password = "private-pass-123",
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            validateFirmwareUpload(
                image = byteArrayOf(1),
                filename = "firmware.bin",
                password = "short",
            )
        }
    }
}
