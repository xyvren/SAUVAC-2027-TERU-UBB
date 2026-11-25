package org.ubb.sauvaconfig

import android.net.Network
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.HttpURLConnection
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import java.net.URL
import java.nio.charset.StandardCharsets
import java.util.Base64
import java.util.concurrent.atomic.AtomicLong

internal const val MAX_FIRMWARE_IMAGE_BYTES = 800 * 1024

internal class PicoUdpClient(private val network: Network) {
    companion object {
        private const val DEVICE_IP = "192.168.4.1"
        private const val DEVICE_PORT = 4210
        private const val RESPONSE_BUFFER_BYTES = 2048
        private const val TIMEOUT_MS = 1_000
        private const val RETRIES = 1
        private const val MIN_REQUEST_INTERVAL_MS = 50
        private const val FIRMWARE_UPDATE_URL = "http://192.168.4.1/update"
        private const val MISSION_UPLOAD_URL = "http://192.168.4.1/mission"
        private const val UPDATE_USERNAME = "sauv-update"
    }

    private val requestIds = AtomicLong(1)
    private val requestMutex = Mutex()
    private var lastRequestAtNanos = 0L

    fun usesNetwork(candidate: Network): Boolean = network == candidate

    suspend fun request(operation: String, fields: JSONObject = JSONObject()): JSONObject =
        requestMutex.withLock {
            withContext(Dispatchers.IO) {
                val requestId = nextRequestId()
                val payload = PicoUdpProtocol.encodeRequest(requestId, operation, fields)
                DatagramSocket().use { socket ->
                    network.bindSocket(socket)
                    socket.soTimeout = TIMEOUT_MS
                    val destination = InetSocketAddress(InetAddress.getByName(DEVICE_IP), DEVICE_PORT)
                    val outgoing = DatagramPacket(payload, payload.size, destination)
                    val buffer = ByteArray(RESPONSE_BUFFER_BYTES)

                    for (attempt in 0..RETRIES) {
                        waitForRequestSlot()
                        socket.send(outgoing)
                        lastRequestAtNanos = System.nanoTime()
                        val deadline = System.nanoTime() + TIMEOUT_MS * 1_000_000L
                        while (System.nanoTime() < deadline) {
                            val incoming = DatagramPacket(buffer, buffer.size)
                            try {
                                socket.receive(incoming)
                            } catch (_: SocketTimeoutException) {
                                break
                            }
                            if (incoming.address.hostAddress != DEVICE_IP ||
                                incoming.port != DEVICE_PORT) {
                                continue
                            }
                            try {
                                return@withContext PicoUdpProtocol.decodeResponse(
                                    buffer.copyOf(incoming.length),
                                    requestId,
                                    isGet = operation == "get",
                                )
                            } catch (error: IllegalArgumentException) {
                                if (error.message == "Response request ID does not match.") {
                                    continue
                                }
                                throw error
                            }
                        }
                    }
                }
                throw SocketTimeoutException("Pico W did not reply after one retry.")
            }
        }

    suspend fun uploadFirmware(image: ByteArray, filename: String, password: String): String {
        validateFirmwareUpload(image, filename, password)

        val snapshot = parseFirmwareSnapshot(request("get"))
        require(snapshot.status.idleAndDisarmed && !snapshot.settings.autorunEnabled) {
            "OTA hanya diizinkan saat Pico W idle, disarmed, dan Auto-Run nonaktif."
        }

        return withContext(Dispatchers.IO) {
            val boundary = "----SAUVAC-${System.nanoTime()}"
            val prefix = (
                "--$boundary\r\n" +
                    "Content-Disposition: form-data; name=\"firmware\"; filename=\"$filename\"\r\n" +
                    "Content-Type: application/octet-stream\r\n\r\n"
                ).toByteArray(StandardCharsets.US_ASCII)
            val suffix = "\r\n--$boundary--\r\n".toByteArray(StandardCharsets.US_ASCII)
            val body = ByteArrayOutputStream(prefix.size + image.size + suffix.size).apply {
                write(prefix)
                write(image)
                write(suffix)
            }.toByteArray()
            val credentials = Base64.getEncoder().encodeToString(
                "sauv-update:$password".toByteArray(StandardCharsets.US_ASCII),
            )
            val connection = network.openConnection(URL(FIRMWARE_UPDATE_URL)) as HttpURLConnection
            var bodyTransmissionStarted = false
            try {
                connection.requestMethod = "POST"
                connection.connectTimeout = 10_000
                connection.readTimeout = 60_000
                connection.useCaches = false
                connection.doOutput = true
                connection.setFixedLengthStreamingMode(body.size)
                connection.setRequestProperty("Authorization", "Basic $credentials")
                connection.setRequestProperty(
                    "Content-Type",
                    "multipart/form-data; boundary=$boundary",
                )
                connection.outputStream.use { output ->
                    bodyTransmissionStarted = true
                    output.write(body)
                    output.flush()
                }

                val statusCode = connection.responseCode
                val responseStream = if (statusCode in 200..299) {
                    connection.inputStream
                } else {
                    connection.errorStream
                }
                val responseBody = responseStream?.bufferedReader(StandardCharsets.UTF_8)?.use {
                    it.readText().take(2_048)
                }.orEmpty()
                if (statusCode != 202) {
                    throw PicoFirmwareUpdateException(
                        responseBody.ifBlank { "Pico W menolak upload firmware (HTTP $statusCode)." },
                        outcomeUnknown = false,
                    )
                }
                responseBody.ifBlank { "Firmware diterima; Pico W sedang restart." }
            } catch (error: PicoFirmwareUpdateException) {
                throw error
            } catch (error: IOException) {
                throw PicoFirmwareUpdateException(
                    if (bodyTransmissionStarted) {
                        "Koneksi update terputus; hasil OTA belum diketahui. Baca status Pico sebelum mencoba ulang."
                    } else {
                        "Tidak dapat memulai koneksi OTA ke Pico W."
                    },
                    outcomeUnknown = bodyTransmissionStarted,
                    cause = error,
                )
            } finally {
                connection.disconnect()
            }
        }
    }

    suspend fun uploadMission(json: String, password: String): String {
        validateMissionUpload(json, password)
        val snapshot = parseFirmwareSnapshot(request("get"))
        require(snapshot.status.idleAndDisarmed && !snapshot.settings.autorunEnabled) {
            "Unggah jalur hanya diizinkan saat Pico W idle, disarmed, dan Auto-Run nonaktif."
        }

        return withContext(Dispatchers.IO) {
            val filename = "jalur.json"
            val boundary = "----SAUVAC-${System.nanoTime()}"
            val prefix = (
                "--$boundary\r\n" +
                    "Content-Disposition: form-data; name=\"mission\"; filename=\"$filename\"\r\n" +
                    "Content-Type: application/json; charset=utf-8\r\n\r\n"
                ).toByteArray(StandardCharsets.US_ASCII)
            val image = json.toByteArray(StandardCharsets.UTF_8)
            val suffix = "\r\n--$boundary--\r\n".toByteArray(StandardCharsets.US_ASCII)
            val body = ByteArrayOutputStream(prefix.size + image.size + suffix.size).apply {
                write(prefix)
                write(image)
                write(suffix)
            }.toByteArray()
            val credentials = Base64.getEncoder().encodeToString(
                "$UPDATE_USERNAME:$password".toByteArray(StandardCharsets.US_ASCII),
            )
            val connection = network.openConnection(URL(MISSION_UPLOAD_URL)) as HttpURLConnection
            var bodyTransmissionStarted = false
            try {
                connection.requestMethod = "POST"
                connection.connectTimeout = 10_000
                connection.readTimeout = 20_000
                connection.useCaches = false
                connection.doOutput = true
                connection.setFixedLengthStreamingMode(body.size)
                connection.setRequestProperty("Authorization", "Basic $credentials")
                connection.setRequestProperty(
                    "Content-Type",
                    "multipart/form-data; boundary=$boundary",
                )
                connection.outputStream.use { output ->
                    bodyTransmissionStarted = true
                    output.write(body)
                    output.flush()
                }

                val statusCode = connection.responseCode
                val responseStream = if (statusCode in 200..299) {
                    connection.inputStream
                } else {
                    connection.errorStream
                }
                val responseBody = responseStream?.bufferedReader(StandardCharsets.UTF_8)?.use {
                    it.readText().take(2_048)
                }.orEmpty()
                if (statusCode !in 200..299) {
                    throw IllegalStateException(
                        responseBody.ifBlank { "Pico W menolak upload jalur (HTTP $statusCode)." },
                    )
                }
                responseBody.ifBlank { "Jalur berhasil disimpan ke Pico W." }
            } catch (error: IllegalStateException) {
                throw error
            } catch (error: IOException) {
                throw IOException(
                    if (bodyTransmissionStarted) {
                        "Koneksi upload terputus; hasil penyimpanan jalur belum diketahui. Periksa status sebelum mencoba lagi."
                    } else {
                        "Tidak dapat memulai koneksi upload jalur ke Pico W."
                    },
                    error,
                )
            } finally {
                connection.disconnect()
            }
        }
    }

    private suspend fun waitForRequestSlot() {
        val elapsedMs = (System.nanoTime() - lastRequestAtNanos) / 1_000_000L
        val remainingMs = MIN_REQUEST_INTERVAL_MS - elapsedMs
        if (lastRequestAtNanos != 0L && remainingMs > 0) delay(remainingMs)
    }

    private fun nextRequestId(): Long {
        while (true) {
            val current = requestIds.getAndUpdate { if (it >= 2_147_483_647L) 1 else it + 1 }
            if (current in 0..2_147_483_647L) return current
        }
    }

}

internal class PicoFirmwareUpdateException(
    message: String,
    val outcomeUnknown: Boolean,
    cause: Throwable? = null,
) : IOException(message, cause)

internal fun validateFirmwareUpload(image: ByteArray, filename: String, password: String) {
    require(image.size in 1..MAX_FIRMWARE_IMAGE_BYTES) {
        "File firmware harus berukuran 1–800 KiB."
    }
    require(filename.matches(Regex("[A-Za-z0-9][A-Za-z0-9._-]{0,75}\\.bin", RegexOption.IGNORE_CASE))) {
        "Pilih file .bin dengan nama sederhana."
    }
    require(password.length in 12..63 && password.all { it.code in 32..126 }) {
        "Password AP harus 12–63 karakter ASCII yang dapat dicetak."
    }
    require(!(image.size >= 4 &&
        image[0] == 'U'.code.toByte() &&
        image[1] == 'F'.code.toByte() &&
        image[2] == '2'.code.toByte() &&
        image[3] == '\n'.code.toByte())) {
        "File UF2 harus dipasang melalui BOOTSEL, bukan OTA."
    }
}

internal fun validateMissionUpload(json: String, password: String) {
    val jsonBytes = json.toByteArray(StandardCharsets.UTF_8)
    require(jsonBytes.size in 2..MAX_MISSION_JSON_BYTES) {
        "Jalur harus berukuran 2–3.500 byte."
    }
    require(password.length in 12..63 && password.all { it.code in 32..126 }) {
        "Password AP harus 12–63 karakter ASCII yang dapat dicetak."
    }
}
