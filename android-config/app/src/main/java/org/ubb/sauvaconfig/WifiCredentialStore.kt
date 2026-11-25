package org.ubb.sauvaconfig

import android.content.Context
import android.security.keystore.KeyGenParameterSpec
import android.security.keystore.KeyProperties
import android.util.Base64
import java.nio.charset.StandardCharsets
import java.security.KeyStore
import javax.crypto.Cipher
import javax.crypto.KeyGenerator
import javax.crypto.SecretKey
import javax.crypto.spec.GCMParameterSpec

internal data class SavedWifiCredentials(
    val ssid: String,
    val password: String,
)

internal class WifiCredentialStore(context: Context) {
    companion object {
        private const val PREFERENCES = "sauvac_wifi_credentials"
        private const val KEY_ALIAS = "sauvac_wifi_password"
        private const val KEY_SSID = "ssid"
        private const val KEY_IV = "password_iv"
        private const val KEY_CIPHERTEXT = "password_ciphertext"
        private const val ANDROID_KEYSTORE = "AndroidKeyStore"
        private const val TRANSFORMATION = "AES/GCM/NoPadding"
    }

    private val preferences =
        context.applicationContext.getSharedPreferences(PREFERENCES, Context.MODE_PRIVATE)

    fun load(): SavedWifiCredentials? {
        val ssid = preferences.getString(KEY_SSID, null) ?: return null
        val encodedIv = preferences.getString(KEY_IV, null)
            ?: throw IllegalStateException("Data sandi Wi-Fi tersimpan tidak lengkap.")
        val encodedCiphertext = preferences.getString(KEY_CIPHERTEXT, null)
            ?: throw IllegalStateException("Data sandi Wi-Fi tersimpan tidak lengkap.")
        val cipher = Cipher.getInstance(TRANSFORMATION)
        cipher.init(
            Cipher.DECRYPT_MODE,
            secretKey(),
            GCMParameterSpec(128, Base64.decode(encodedIv, Base64.NO_WRAP)),
        )
        cipher.updateAAD(ssid.toByteArray(StandardCharsets.UTF_8))
        val password = String(
            cipher.doFinal(Base64.decode(encodedCiphertext, Base64.NO_WRAP)),
            StandardCharsets.UTF_8,
        )
        return SavedWifiCredentials(ssid, password)
    }

    fun save(ssid: String, password: String) {
        val normalizedSsid = ssid.trim()
        require(normalizedSsid.isNotEmpty()) { "SSID Pico W tidak boleh kosong." }
        require(password.length in 12..63 && password.all { it.code in 32..126 }) {
            "Password Wi-Fi harus 12–63 karakter ASCII yang dapat dicetak."
        }

        val cipher = Cipher.getInstance(TRANSFORMATION)
        cipher.init(Cipher.ENCRYPT_MODE, secretKey())
        cipher.updateAAD(normalizedSsid.toByteArray(StandardCharsets.UTF_8))
        val ciphertext = cipher.doFinal(password.toByteArray(StandardCharsets.UTF_8))
        val saved = preferences.edit()
            .putString(KEY_SSID, normalizedSsid)
            .putString(KEY_IV, Base64.encodeToString(cipher.iv, Base64.NO_WRAP))
            .putString(KEY_CIPHERTEXT, Base64.encodeToString(ciphertext, Base64.NO_WRAP))
            .commit()
        check(saved) { "Android gagal menyimpan kredensial Wi-Fi." }
    }

    fun clear() {
        check(preferences.edit().clear().commit()) {
            "Android gagal menghapus kredensial Wi-Fi tersimpan."
        }
        val keyStore = KeyStore.getInstance(ANDROID_KEYSTORE).apply { load(null) }
        if (keyStore.containsAlias(KEY_ALIAS)) {
            keyStore.deleteEntry(KEY_ALIAS)
        }
    }

    private fun secretKey(): SecretKey {
        val keyStore = KeyStore.getInstance(ANDROID_KEYSTORE).apply { load(null) }
        if (keyStore.containsAlias(KEY_ALIAS)) {
            return keyStore.getKey(KEY_ALIAS, null) as? SecretKey
                ?: throw IllegalStateException("Kunci sandi Wi-Fi Android tidak valid.")
        }

        val generator = KeyGenerator.getInstance(KeyProperties.KEY_ALGORITHM_AES, ANDROID_KEYSTORE)
        generator.init(
            KeyGenParameterSpec.Builder(
                KEY_ALIAS,
                KeyProperties.PURPOSE_ENCRYPT or KeyProperties.PURPOSE_DECRYPT,
            )
                .setBlockModes(KeyProperties.BLOCK_MODE_GCM)
                .setEncryptionPaddings(KeyProperties.ENCRYPTION_PADDING_NONE)
                .setRandomizedEncryptionRequired(true)
                .build(),
        )
        return generator.generateKey()
    }
}
