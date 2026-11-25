package org.ubb.sauvaconfig

import org.json.JSONObject
import java.nio.charset.StandardCharsets

internal object PicoUdpProtocol {
    const val MAX_REQUEST_BYTES = 384
    const val PROTOCOL_VERSION = 1

    fun encodeRequest(requestId: Long, operation: String, fields: JSONObject = JSONObject()): ByteArray {
        require(requestId in 0..2_147_483_647L) { "Request ID is outside the protocol range." }
        require(operation.isNotBlank()) { "Operation is required." }

        val request = JSONObject()
            .put("req_id", requestId)
            .put("op", operation)
        val keys = fields.keys()
        while (keys.hasNext()) {
            val key = keys.next()
            require(key != "req_id" && key != "op") { "Reserved protocol field: $key" }
            request.put(key, fields.get(key))
        }

        val bytes = request.toString().toByteArray(StandardCharsets.UTF_8)
        require(bytes.size <= MAX_REQUEST_BYTES) {
            "Request is ${bytes.size} bytes; protocol limit is $MAX_REQUEST_BYTES."
        }
        return bytes
    }

    fun decodeResponse(bytes: ByteArray, requestId: Long, isGet: Boolean = false): JSONObject {
        val response = JSONObject(String(bytes, StandardCharsets.UTF_8))
        val actualId = response.opt("req_id")
        require(actualId is Number && actualId.toLong() == requestId) {
            "Response request ID does not match."
        }
        if (isGet) {
            require(response.optInt("protocol_version", -1) == PROTOCOL_VERSION) {
                "Unsupported or missing firmware protocol version."
            }
            require(response.optString("op") == "config") {
                response.optString("error", "Firmware returned an invalid configuration response.")
            }
        }
        require(response.optBoolean("ok", false)) {
            response.optString("error", "Firmware rejected the request.")
        }
        return response
    }
}
