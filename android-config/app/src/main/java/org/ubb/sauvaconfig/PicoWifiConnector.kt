package org.ubb.sauvaconfig

import android.annotation.SuppressLint
import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.net.wifi.WifiNetworkSpecifier
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withTimeout
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException

internal class PicoWifiConnector(context: Context) {
    private val connectivity =
        context.applicationContext.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    private var callback: ConnectivityManager.NetworkCallback? = null

    @SuppressLint("MissingPermission")
    suspend fun connect(
        ssid: String,
        password: String,
        onNetworkLost: (Network) -> Unit = {},
    ): Network = withTimeout(30_000) {
        disconnect()
        val specifier = WifiNetworkSpecifier.Builder()
            .setSsid(ssid)
            .setWpa2Passphrase(password)
            .setIsHiddenSsid(true)
            .build()
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            .setNetworkSpecifier(specifier)
            .build()

        suspendCancellableCoroutine { continuation ->
            var availableNetwork: Network? = null
            val networkCallback = object : ConnectivityManager.NetworkCallback() {
                override fun onAvailable(network: Network) {
                    availableNetwork = network
                    if (continuation.isActive) continuation.resume(network)
                }

                override fun onLost(network: Network) {
                    if (network == availableNetwork) onNetworkLost(network)
                }

                override fun onUnavailable() {
                    if (continuation.isActive) {
                        continuation.resumeWithException(
                            IllegalStateException("Android could not join the Pico W access point."),
                        )
                    }
                }
            }
            callback = networkCallback
            try {
                connectivity.requestNetwork(request, networkCallback)
            } catch (error: SecurityException) {
                callback = null
                if (continuation.isActive) continuation.resumeWithException(error)
            } catch (error: IllegalArgumentException) {
                callback = null
                if (continuation.isActive) continuation.resumeWithException(error)
            }
            continuation.invokeOnCancellation { disconnect() }
        }
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        callback?.let {
            try {
                connectivity.unregisterNetworkCallback(it)
            } catch (_: IllegalArgumentException) {
                // The system can already have removed a timed-out network request.
            }
        }
        callback = null
    }
}
