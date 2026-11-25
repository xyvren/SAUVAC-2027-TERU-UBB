package org.ubb.sauvaconfig

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.provider.OpenableColumns
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.FilterChip
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.ScrollableTabRow
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Tab
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.lightColorScheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.text.input.VisualTransformation
import androidx.compose.ui.unit.dp
import java.io.ByteArrayOutputStream
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.util.Locale

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent { SauvConfigApp() }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun SauvConfigApp() {
    val context = LocalContext.current
    val scope = rememberCoroutineScope()
    val connector = remember { PicoWifiConnector(context) }
    val credentialStore = remember { WifiCredentialStore(context) }
    var ssid by remember { mutableStateOf("") }
    var password by remember { mutableStateOf("") }
    var rememberWifiPassword by remember { mutableStateOf(true) }
    var credentialsLoaded by remember { mutableStateOf(false) }
    var hasSavedCredentials by remember { mutableStateOf(false) }
    var credentialError by remember { mutableStateOf<String?>(null) }
    var client by remember { mutableStateOf<PicoUdpClient?>(null) }
    var consecutivePollFailures by remember { mutableStateOf(0) }
    var snapshot by remember { mutableStateOf<FirmwareSnapshot?>(null) }
    var draft by remember { mutableStateOf<AuvConfiguration?>(null) }
    var isBusy by remember { mutableStateOf(false) }
    var connectionLabel by remember { mutableStateOf("Belum terhubung") }
    var errorMessage by remember { mutableStateOf<String?>(null) }
    var infoMessage by remember { mutableStateOf<String?>(null) }
    var confirmAction by remember { mutableStateOf<String?>(null) }
    var selectedTab by remember { mutableStateOf(0) }
    var firmwareUri by remember { mutableStateOf<Uri?>(null) }
    var firmwareFilename by remember { mutableStateOf<String?>(null) }
    var firmwarePassword by remember { mutableStateOf("") }
    var firmwareStatus by remember { mutableStateOf("Pilih file .bin untuk memulai.") }
    var firmwareBusy by remember { mutableStateOf(false) }
    var firmwareRestarting by remember { mutableStateOf(false) }
    var firmwareOutcomeUnknown by remember { mutableStateOf(false) }
    var confirmFirmwareUpdate by remember { mutableStateOf(false) }
    var missionSteps by remember { mutableStateOf(emptyList<MissionStepDraft>()) }
    var missionPassword by remember { mutableStateOf("") }
    var missionStatus by remember { mutableStateOf("Buat langkah untuk menyusun jalur.") }
    var confirmMissionUpload by remember { mutableStateOf(false) }
    var confirmCalibration by remember { mutableStateOf<String?>(null) }

    fun setError(error: Throwable) {
        errorMessage = error.message ?: "Operasi gagal. Periksa koneksi dan coba lagi."
    }

    LaunchedEffect(credentialStore) {
        try {
            credentialStore.load()?.let { saved ->
                hasSavedCredentials = true
                ssid = saved.ssid
                password = saved.password
                rememberWifiPassword = true
            }
        } catch (error: Exception) {
            rememberWifiPassword = false
            credentialError = error.message
                ?: "Sandi tersimpan tidak dapat dibuka. Hapus lalu masukkan ulang."
        } finally {
            credentialsLoaded = true
        }
    }

    fun restoreSavedCredentials() {
        if (!rememberWifiPassword) return
        try {
            credentialStore.load()?.let { saved ->
                hasSavedCredentials = true
                ssid = saved.ssid
                password = saved.password
            }
        } catch (error: Exception) {
            credentialError = error.message
                ?: "Sandi tersimpan tidak dapat dibuka. Hapus lalu masukkan ulang."
            errorMessage = credentialError
        }
    }

    fun clearSavedCredentials(clearPassword: Boolean = true) {
        try {
            credentialStore.clear()
            hasSavedCredentials = false
            credentialError = null
            if (clearPassword) password = ""
            rememberWifiPassword = false
            infoMessage = "Sandi Wi-Fi tersimpan dihapus dari perangkat."
        } catch (error: Exception) {
            credentialError = error.message ?: "Sandi tersimpan gagal dihapus."
            setError(error)
        }
    }

    fun handleNetworkLost(lostNetwork: android.net.Network) {
        if (client?.usesNetwork(lostNetwork) != true) return
        client = null
        snapshot = null
        draft = null
        consecutivePollFailures = 0
        connector.disconnect()
        connectionLabel = "Wi-Fi terputus · hubungkan ulang dari aplikasi"
        errorMessage = "Koneksi ke Pico W terputus. Tekan Hubungkan ulang untuk menyambung kembali."
        restoreSavedCredentials()
    }

    fun connect() {
        if (!credentialsLoaded) {
            errorMessage = "Tunggu data sandi tersimpan selesai dimuat."
            return
        }
        if (ssid.isBlank() || password.length !in 12..63 ||
            password.any { it.code !in 32..126 }) {
            errorMessage = "Isi SSID Pico W dan password WPA2 12–63 karakter ASCII."
            return
        }
        val requestedSsid = ssid.trim()
        val requestedPassword = password
        scope.launch {
            isBusy = true
            errorMessage = null
            connectionLabel = "Menghubungkan ke access point Pico W…"
            try {
                val network = connector.connect(requestedSsid, requestedPassword) { lostNetwork ->
                    scope.launch { handleNetworkLost(lostNetwork) }
                }
                val newClient = PicoUdpClient(network)
                val firstSnapshot = parseFirmwareSnapshot(newClient.request("get"))
                client = newClient
                snapshot = firstSnapshot
                draft = firstSnapshot.settings
                consecutivePollFailures = 0
                password = ""
                connectionLabel = "Terhubung · firmware v${PicoUdpProtocol.PROTOCOL_VERSION}"
                infoMessage = "Draft konfigurasi tersinkron dari Pico W."
                if (rememberWifiPassword) {
                    try {
                        credentialStore.save(requestedSsid, requestedPassword)
                        hasSavedCredentials = true
                        credentialError = null
                        infoMessage = "Terhubung. Sandi disimpan terenkripsi di perangkat."
                    } catch (error: Exception) {
                        credentialError = error.message
                            ?: "Sandi tidak dapat disimpan di perangkat."
                        setError(error)
                    }
                } else {
                    try {
                        credentialStore.clear()
                        hasSavedCredentials = false
                        credentialError = null
                    } catch (error: Exception) {
                        credentialError = error.message ?: "Sandi tersimpan gagal dihapus."
                        setError(error)
                    }
                }
                firmwareRestarting = false
                firmwareOutcomeUnknown = false
            } catch (error: Exception) {
                connector.disconnect()
                client = null
                snapshot = null
                draft = null
                consecutivePollFailures = 0
                connectionLabel = "Gagal terhubung"
                setError(error)
            } finally {
                isBusy = false
            }
        }
    }

    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestPermission(),
    ) { granted ->
        if (granted) connect()
        else errorMessage = "Izin lokasi/perangkat sekitar diperlukan untuk meminta koneksi Wi-Fi."
    }

    fun requestConnection() {
        val requiredPermission = when {
            Build.VERSION.SDK_INT >= 33 -> Manifest.permission.NEARBY_WIFI_DEVICES
            else -> Manifest.permission.ACCESS_FINE_LOCATION
        }
        if (context.checkSelfPermission(requiredPermission) != PackageManager.PERMISSION_GRANTED) {
            permissionLauncher.launch(requiredPermission)
            return
        }
        connect()
    }

    fun disconnect() {
        client = null
        connector.disconnect()
        selectedTab = 0
        snapshot = null
        draft = null
        password = ""
        restoreSavedCredentials()
        firmwarePassword = ""
        firmwareUri = null
        firmwareFilename = null
        missionPassword = ""
        firmwareRestarting = false
        connectionLabel = "Terputus"
        infoMessage = null
        errorMessage = null
    }

    fun performAction(action: String) {
        val target = client ?: return
        val currentSnapshot = snapshot ?: return
        val currentDraft = draft ?: return
        scope.launch {
            isBusy = true
            errorMessage = null
            try {
                when (action) {
                    "stage" -> {
                        val operations = buildStageOperations(currentSnapshot.settings, currentDraft)
                        if (operations.isEmpty()) {
                            infoMessage = "Tidak ada perubahan untuk dikirim."
                            return@launch
                        }
                        for (operation in operations) {
                            target.request(operation.operation, operation.fields)
                        }
                        val staged = parseFirmwareSnapshot(target.request("get"))
                        snapshot = staged
                        draft = staged.settings
                        infoMessage = "${operations.size} perubahan berhasil di-stage pada firmware."
                    }
                    "apply" -> {
                        target.request("apply")
                        val applied = parseFirmwareSnapshot(target.request("get"))
                        snapshot = applied
                        draft = applied.settings
                        infoMessage = "Konfigurasi diterapkan ke runtime Pico W; belum tersimpan permanen."
                    }
                    "save" -> {
                        target.request("save")
                        val saved = parseFirmwareSnapshot(target.request("get"))
                        snapshot = saved
                        draft = saved.settings
                        infoMessage = "Konfigurasi tersimpan ke memori Pico W."
                    }
                    "discard" -> {
                        target.request("discard")
                        val restored = parseFirmwareSnapshot(target.request("get"))
                        snapshot = restored
                        draft = restored.settings
                        infoMessage = "Draft dibuang; nilai EEPROM dipulihkan."
                    }
                }
            } catch (error: Exception) {
                setError(error)
            } finally {
                isBusy = false
            }
        }
    }

    val firmwarePicker = rememberLauncherForActivityResult(
        ActivityResultContracts.OpenDocument(),
    ) { uri ->
        firmwareUri = uri
        firmwareFilename = uri?.let { context.displayName(it) }
        firmwareStatus = if (uri == null) {
            "Pemilihan file dibatalkan."
        } else {
            "File dipilih: ${firmwareFilename ?: "nama tidak diketahui"}."
        }
    }

    fun uploadFirmware() {
        if (firmwareOutcomeUnknown) {
            firmwareStatus = "Baca ulang status Pico dengan memutuskan dan menyambungkan kembali sebelum mempertimbangkan upload ulang."
            return
        }
        val target = client ?: return
        val uri = firmwareUri ?: run {
            firmwareStatus = "Pilih file firmware .bin terlebih dahulu."
            return
        }
        val filename = firmwareFilename.orEmpty()
        if (!filename.endsWith(".bin", ignoreCase = true)) {
            firmwareStatus = "Pilih firmware Pico W berformat .bin. File UF2 tidak didukung lewat OTA."
            return
        }
        if (firmwarePassword.length !in 12..63) {
            firmwareStatus = "Masukkan password AP privat (12–63 karakter)."
            return
        }
        scope.launch {
            firmwareBusy = true
            firmwareStatus = "Memeriksa status Pico W dan membaca file…"
            try {
                val image = withContext(Dispatchers.IO) { context.readFirmwareFile(uri) }
                firmwareStatus = "Mengirim firmware ke Pico W…"
                val response = target.uploadFirmware(image, filename, firmwarePassword)
                firmwareStatus = response
                firmwareUri = null
                firmwareFilename = null
                firmwareRestarting = true
                firmwareOutcomeUnknown = false
                connectionLabel = "Firmware diterima · menunggu Pico W restart"
                infoMessage = "Tunggu perangkat selesai boot, lalu sambungkan kembali."
            } catch (error: Exception) {
                val updateError = error as? PicoFirmwareUpdateException
                if (updateError?.outcomeUnknown == true) {
                    firmwareRestarting = true
                    firmwareOutcomeUnknown = true
                }
                firmwareStatus = error.message ?: "Upload firmware gagal."
            } finally {
                firmwarePassword = ""
                firmwareBusy = false
            }
        }
    }

    fun uploadMission() {
            val target = client ?: return
            val json = try {
                buildMissionJson(missionSteps)
            } catch (error: IllegalArgumentException) {
                missionStatus = error.message ?: "Periksa jalur sebelum mengunggah."
                return
            }
            scope.launch {
                isBusy = true
                missionStatus = "Memeriksa status dan mengunggah jalur…"
                errorMessage = null
                try {
                    missionStatus = target.uploadMission(json, missionPassword)
                    missionPassword = ""
                    val latest = parseFirmwareSnapshot(target.request("get"))
                    snapshot = latest
                    infoMessage = missionStatus
                } catch (error: Exception) {
                    missionStatus = error.message ?: "Upload jalur gagal."
                } finally {
                    isBusy = false
                }
            }
        }

    fun calibrateSensor(operation: String) {
        val target = client ?: return
        scope.launch {
            isBusy = true
            errorMessage = null
            try {
                target.request(operation)
                val latest = parseFirmwareSnapshot(target.request("get"))
                snapshot = latest
                draft = latest.settings
                infoMessage = if (operation == "calibrate_depth") {
                    "Tare kedalaman berhasil dan tersimpan di Pico W."
                } else {
                    "Kalibrasi level IMU berhasil dan tersimpan di Pico W."
                }
            } catch (error: Exception) {
                setError(error)
            } finally {
                isBusy = false
            }
        }
    }

    LaunchedEffect(client, firmwareRestarting, isBusy, firmwareBusy) {
        val target = client ?: return@LaunchedEffect
        if (firmwareRestarting) return@LaunchedEffect
        while (true) {
            delay(1_000)
            if (isBusy || firmwareBusy) continue
            try {
                val latest = parseFirmwareSnapshot(target.request("get"))
                snapshot = latest
                consecutivePollFailures = 0
                connectionLabel = "Terhubung · firmware v${PicoUdpProtocol.PROTOCOL_VERSION}"
                errorMessage = null
            } catch (error: Exception) {
                consecutivePollFailures += 1
                if (consecutivePollFailures < 3) {
                    connectionLabel = "Terhubung · mencoba ulang (${consecutivePollFailures}/3)"
                    errorMessage = null
                } else if (client === target) {
                    client = null
                    connector.disconnect()
                    snapshot = null
                    draft = null
                    connectionLabel = "Pico W tidak merespons · hubungkan ulang"
                    setError(error)
                    restoreSavedCredentials()
                    return@LaunchedEffect
                }
            }
        }
    }

    DisposableEffect(connector) {
        onDispose { connector.disconnect() }
    }

    val colorScheme = if (isSystemInDarkTheme()) {
        darkColorScheme(
            primary = Color(0xFF49B6C7),
            onPrimary = Color(0xFF06252B),
            secondary = Color(0xFF72D7B5),
            onSecondary = Color(0xFF06271B),
            background = Color(0xFF10191E),
            onBackground = Color(0xFFE7F0F2),
            surface = Color(0xFF19262D),
            onSurface = Color(0xFFE7F0F2),
            surfaceVariant = Color(0xFF24343B),
            onSurfaceVariant = Color(0xFFB7C7CD),
            outline = Color(0xFF49616B),
            error = Color(0xFFFF7B72),
            onError = Color(0xFF3B0A08),
            errorContainer = Color(0xFF5B201C),
            onErrorContainer = Color(0xFFFFDAD6),
        )
    } else {
        lightColorScheme(
            primary = Color(0xFF0B789F),
            onPrimary = Color.White,
            secondary = Color(0xFF187044),
            onSecondary = Color.White,
            background = Color(0xFFF4F6F7),
            onBackground = Color(0xFF182F3B),
            surface = Color.White,
            onSurface = Color(0xFF182F3B),
            surfaceVariant = Color(0xFFEEF3F5),
            onSurfaceVariant = Color(0xFF526772),
            outline = Color(0xFFC7D4DA),
            error = Color(0xFFB4232F),
            onError = Color.White,
            errorContainer = Color(0xFFFFE9E8),
            onErrorContainer = Color(0xFF7F1D1D),
        )
    }

    val connectionOnline = client != null && consecutivePollFailures < 3
    MaterialTheme(colorScheme = colorScheme) {
        Scaffold(
            topBar = {
                TopAppBar(
                    title = {
                        Column {
                            Text("SAUVAC")
                            Text(
                                "KONTROL PERANGKAT",
                                style = MaterialTheme.typography.labelSmall,
                                color = MaterialTheme.colorScheme.onSurfaceVariant,
                            )
                        }
                    },
                    actions = {
                        ConnectionIndicator(
                            connected = connectionOnline,
                            modifier = Modifier.padding(end = 12.dp),
                        )
                    },
                )
            },
        ) { padding ->
            Column(
                modifier = Modifier.fillMaxSize().padding(padding),
            ) {
                ScrollableTabRow(selectedTabIndex = selectedTab) {
                    listOf("Ringkasan", "Jalur", "Sensor", "Parameter", "Firmware").forEachIndexed { index, label ->
                        Tab(
                            selected = selectedTab == index,
                            onClick = { selectedTab = index },
                            text = { Text(label) },
                        )
                    }
                }
                when (selectedTab) {
                    0 -> HomeScreen(
                        modifier = Modifier.weight(1f),
                        ssid = ssid,
                        onSsidChange = { ssid = it },
                        password = password,
                        onPasswordChange = { password = it },
                        connected = connectionOnline,
                        busy = isBusy || firmwareBusy,
                        status = connectionLabel,
                        rememberPassword = rememberWifiPassword,
                        credentialsLoaded = credentialsLoaded,
                        credentialError = credentialError,
                        onRememberPasswordChange = { enabled ->
                            if (enabled) {
                                rememberWifiPassword = true
                            } else {
                                clearSavedCredentials(clearPassword = false)
                            }
                        },
                        onClearCredentials = ::clearSavedCredentials,
                        hasSavedCredentials = hasSavedCredentials,
                        snapshot = snapshot,
                        errorMessage = errorMessage,
                        infoMessage = infoMessage,
                        onConnect = ::requestConnection,
                        onDisconnect = ::disconnect,
                        onSelectTab = { selectedTab = it },
                    )
                    1 -> MissionScreen(
                        modifier = Modifier.weight(1f),
                        connected = connectionOnline,
                        steps = missionSteps,
                        onStepsChange = { missionSteps = it },
                        password = missionPassword,
                        onPasswordChange = { missionPassword = it },
                        status = missionStatus,
                        busy = isBusy,
                        canUpload = snapshot?.let {
                            remoteMutationReady(it) &&
                                !it.settings.autorunEnabled &&
                                validateMissionSteps(missionSteps) == null &&
                                missionPassword.length in 12..63 &&
                                missionPassword.all { character -> character.code in 32..126 }
                        } == true && !isBusy,
                        errorMessage = errorMessage,
                        infoMessage = infoMessage,
                        onGoHome = { selectedTab = 0 },
                        onUpload = { confirmMissionUpload = true },
                    )
                    2 -> MonitoringCalibrationScreen(
                        modifier = Modifier.weight(1f),
                        connected = connectionOnline,
                        snapshot = snapshot,
                        busy = isBusy,
                        errorMessage = errorMessage,
                        infoMessage = infoMessage,
                        onGoHome = { selectedTab = 0 },
                        onCalibrate = { confirmCalibration = it },
                    )
                    3 -> ParameterScreen(
                        modifier = Modifier.weight(1f),
                        connected = connectionOnline,
                        snapshot = snapshot,
                        draft = draft,
                        busy = isBusy,
                        errorMessage = errorMessage,
                        infoMessage = infoMessage,
                        onGoHome = { selectedTab = 0 },
                        onDraftChange = { draft = it },
                        onStage = { performAction("stage") },
                        onApply = { confirmAction = "apply" },
                        onSave = { confirmAction = "save" },
                        onDiscard = { confirmAction = "discard" },
                    )
                    else -> FirmwareScreen(
                        modifier = Modifier.weight(1f),
                        connected = connectionOnline,
                        filename = firmwareFilename,
                        password = firmwarePassword,
                        onPasswordChange = { firmwarePassword = it },
                        status = firmwareStatus,
                        busy = firmwareBusy,
                        canUpload = firmwareUri != null &&
                            firmwareFilename?.endsWith(".bin", ignoreCase = true) == true &&
                            firmwarePassword.length in 12..63 &&
                            !firmwareOutcomeUnknown &&
                            !isBusy && !firmwareBusy,
                        errorMessage = errorMessage,
                        infoMessage = infoMessage,
                        onGoHome = { selectedTab = 0 },
                        onSelectFile = { firmwarePicker.launch(arrayOf("*/*")) },
                        onUpload = { confirmFirmwareUpdate = true },
                    )
                }
            }
        }
        confirmAction?.let { action ->
            val title = when (action) {
                "apply" -> "Terapkan konfigurasi?"
                "save" -> "Simpan permanen?"
                else -> "Buang perubahan?"
            }
            val description = when (action) {
                "apply" -> "Nilai staged akan aktif di runtime Pico W. Proses ini belum menyimpannya ke EEPROM."
                "save" -> "Nilai runtime akan ditulis ke EEPROM dan bertahan setelah reboot."
                else -> "Draft yang belum disimpan akan diganti dengan nilai EEPROM terakhir."
            }
            AlertDialog(
                onDismissRequest = { confirmAction = null },
                title = { Text(title) },
                text = { Text(description) },
                confirmButton = {
                    Button(
                        onClick = {
                            val selected = action
                            confirmAction = null
                            performAction(selected)
                        },
                    ) { Text("Lanjutkan") }
                },
                dismissButton = {
                    OutlinedButton(onClick = { confirmAction = null }) { Text("Batal") }
                },
            )
        }
        if (confirmFirmwareUpdate) {
            AlertDialog(
                onDismissRequest = { if (!firmwareBusy) confirmFirmwareUpdate = false },
                title = { Text("Unggah firmware ke Pico W?") },
                text = {
                    Text(
                        "Lepas propeler dan isolasi thruster. Firmware .bin akan dikirim langsung ke Pico W; perangkat harus idle, disarmed, dan Auto-Run nonaktif. Jika koneksi terputus saat upload, hasilnya mungkin belum diketahui.",
                    )
                },
                confirmButton = {
                    Button(
                        onClick = {
                            confirmFirmwareUpdate = false
                            uploadFirmware()
                        },
                        enabled = !firmwareBusy,
                    ) { Text("Unggah") }
                },
                dismissButton = {
                    OutlinedButton(
                        onClick = { confirmFirmwareUpdate = false },
                        enabled = !firmwareBusy,
                    ) { Text("Batal") }
                },
            )
        }
        if (confirmMissionUpload) {
            AlertDialog(
                onDismissRequest = { if (!isBusy) confirmMissionUpload = false },
                title = { Text("Simpan jalur ke Pico W?") },
                text = {
                    Text(
                        "Jalur akan menggantikan rute tersimpan, tetapi tidak menjalankan misi. Pastikan Pico idle, disarmed, Auto-Run mati, dan propeler dilepas. Perangkat memeriksa interlock sebelum menyimpan.",
                    )
                },
                confirmButton = {
                    Button(
                        onClick = {
                            confirmMissionUpload = false
                            uploadMission()
                        },
                        enabled = !isBusy,
                    ) { Text("Simpan jalur") }
                },
                dismissButton = {
                    OutlinedButton(
                        onClick = { confirmMissionUpload = false },
                        enabled = !isBusy,
                    ) { Text("Batal") }
                },
            )
        }
        confirmCalibration?.let { operation ->
            val depth = operation == "calibrate_depth"
            AlertDialog(
                onDismissRequest = { if (!isBusy) confirmCalibration = null },
                title = { Text(if (depth) "Kalibrasi titik nol kedalaman?" else "Kalibrasi level IMU?") },
                text = {
                    Text(
                        if (depth) {
                            "Letakkan AUV di permukaan air yang tenang. Tare akan langsung disimpan ke EEPROM. Pastikan thruster netral dan propeler dilepas."
                        } else {
                            "Letakkan AUV diam di permukaan datar dan rata. Offset pitch/roll akan langsung disimpan ke EEPROM. Pastikan thruster netral dan propeler dilepas."
                        },
                    )
                },
                confirmButton = {
                    Button(
                        onClick = {
                            confirmCalibration = null
                            calibrateSensor(operation)
                        },
                        enabled = !isBusy,
                    ) { Text("Kalibrasi & simpan") }
                },
                dismissButton = {
                    OutlinedButton(
                        onClick = { confirmCalibration = null },
                        enabled = !isBusy,
                    ) { Text("Batal") }
                },
            )
        }
    }
}

@Composable
private fun ConnectionIndicator(
    connected: Boolean,
    modifier: Modifier = Modifier,
) {
    val dotColor = if (connected) {
        MaterialTheme.colorScheme.secondary
    } else {
        MaterialTheme.colorScheme.onSurfaceVariant
    }
    val containerColor = if (connected) {
        MaterialTheme.colorScheme.secondaryContainer
    } else {
        MaterialTheme.colorScheme.surfaceVariant
    }

    Surface(
        modifier = modifier.heightIn(min = 36.dp),
        shape = CircleShape,
        color = containerColor,
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            Box(Modifier.size(8.dp).background(dotColor, CircleShape))
            Text(
                if (connected) "ONLINE" else "OFFLINE",
                style = MaterialTheme.typography.labelSmall,
                color = MaterialTheme.colorScheme.onSurface,
            )
        }
    }
}

@Composable
private fun HomeScreen(
    modifier: Modifier,
    ssid: String,
    onSsidChange: (String) -> Unit,
    password: String,
    onPasswordChange: (String) -> Unit,
    connected: Boolean,
    busy: Boolean,
    status: String,
    rememberPassword: Boolean,
    credentialsLoaded: Boolean,
    credentialError: String?,
    onRememberPasswordChange: (Boolean) -> Unit,
    onClearCredentials: () -> Unit,
    hasSavedCredentials: Boolean,
    snapshot: FirmwareSnapshot?,
    errorMessage: String?,
    infoMessage: String?,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
    onSelectTab: (Int) -> Unit,
) {
    LazyColumn(
        modifier = modifier.fillMaxWidth(),
        contentPadding = androidx.compose.foundation.layout.PaddingValues(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        item {
            ConnectionCard(
                ssid = ssid,
                onSsidChange = onSsidChange,
                password = password,
                onPasswordChange = onPasswordChange,
                connected = connected,
                busy = busy,
                status = status,
                rememberPassword = rememberPassword,
                credentialsLoaded = credentialsLoaded,
                credentialError = credentialError,
                onRememberPasswordChange = onRememberPasswordChange,
                onClearCredentials = onClearCredentials,
                hasSavedCredentials = hasSavedCredentials,
                onConnect = onConnect,
                onDisconnect = onDisconnect,
            )
        }
        errorMessage?.let { message ->
            item { MessageCard("Koneksi bermasalah", message, isError = true) }
        }
        infoMessage?.let { message ->
            item { MessageCard("Pembaruan", message) }
        }

        if (snapshot != null) {
            item { DeviceStatusCard(snapshot.status) }
            item {
                SectionCard("Konfigurasi perangkat") {
                    Text(
                        if (snapshot.dirty) "Ada perubahan yang belum disimpan."
                        else "Parameter Pico W tersinkron dengan nilai tersimpan.",
                    )
                    Text(
                        if (snapshot.status.idleAndDisarmed) {
                            "Kondisi: idle dan disarmed"
                        } else {
                            "Perubahan parameter dikunci sampai perangkat idle dan disarmed."
                        },
                        style = MaterialTheme.typography.bodySmall,
                    )
                    Button(
                        onClick = { onSelectTab(3) },
                        modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
                    ) {
                        Text("Buka parameter")
                    }
                }
            }
        } else {
            item {
                SectionCard("Ringkasan perangkat") {
                    Text("Masukkan SSID Pico W dan hubungkan untuk membaca status dan konfigurasi.")
                    Text(
                        "Koneksi dilakukan langsung ke access point perangkat; internet tidak diperlukan.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }

        item {
            SectionCard("Aplikasi Android") {
                Text("Susun jalur, pantau sensor, kalibrasi sensor, kelola parameter, dan unggah firmware dari Android.")
                Text(
                    "Simulasi 3D, terminal USB, HIL, kalibrasi ESC, dan kontrol motor tetap menggunakan dashboard web atau USB.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(
                        onClick = { onSelectTab(3) },
                        modifier = Modifier.weight(1f).heightIn(min = 48.dp),
                    ) {
                        Text("Parameter")
                    }
                    OutlinedButton(
                        onClick = { onSelectTab(4) },
                        modifier = Modifier.weight(1f).heightIn(min = 48.dp),
                    ) {
                        Text("Firmware")
                    }
                }
            }
        }
    }
}

@Composable
private fun MissionScreen(
    modifier: Modifier,
    connected: Boolean,
    steps: List<MissionStepDraft>,
    onStepsChange: (List<MissionStepDraft>) -> Unit,
    password: String,
    onPasswordChange: (String) -> Unit,
    status: String,
    busy: Boolean,
    canUpload: Boolean,
    errorMessage: String?,
    infoMessage: String?,
    onGoHome: () -> Unit,
    onUpload: () -> Unit,
) {
    if (!connected) {
        ConnectionRequiredScreen(
            modifier = modifier,
            title = "Pico W belum tersambung",
            onGoHome = onGoHome,
        )
        return
    }

    val validationError = validateMissionSteps(steps)
    val totalDuration = steps.sumOf { it.durationSec.toDoubleOrNull() ?: 0.0 }
    LazyColumn(
        modifier = modifier.fillMaxWidth(),
        contentPadding = androidx.compose.foundation.layout.PaddingValues(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        errorMessage?.let { message ->
            item { MessageCard("Operasi gagal", message, isError = true) }
        }
        infoMessage?.let { message ->
            item { MessageCard("Perangkat", message) }
        }
        item {
            SectionCard("Editor jalur") {
                Text("${steps.size}/32 langkah · perkiraan ${formatReading(totalDuration, 1)} detik")
                Text(
                    "Jalur disimpan di Pico W tetapi tidak dijalankan. Pilih aksi, daya, durasi, kedalaman, dan thruster untuk tiap langkah.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                if (steps.isEmpty()) {
                    Text("Belum ada langkah. Tambahkan langkah pertama.")
                }
            }
        }
        itemsIndexed(steps, key = { index, _ -> index }) { index, step ->
            MissionStepCard(
                index = index,
                step = step,
                enabled = !busy,
                canMoveUp = index > 0,
                canMoveDown = index < steps.lastIndex,
                onChange = { changed ->
                    onStepsChange(steps.mapIndexed { itemIndex, item ->
                        if (itemIndex == index) changed else item
                    })
                },
                onMove = { delta ->
                    val targetIndex = index + delta
                    if (targetIndex in steps.indices) {
                        onStepsChange(steps.toMutableList().apply {
                            add(targetIndex, removeAt(index))
                        })
                    }
                },
                onRemove = {
                    onStepsChange(steps.filterIndexed { itemIndex, _ -> itemIndex != index })
                },
            )
        }
        item {
            OutlinedButton(
                onClick = { onStepsChange(steps + newMissionStep(steps.size + 1)) },
                enabled = !busy && steps.size < 32,
                modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
            ) {
                Text("Tambah langkah")
            }
        }
        item {
            SectionCard("Unggah ke Pico W") {
                Text(
                    "Unggah memakai autentikasi AP dan mengganti jalur tersimpan. Perubahan hanya diterima ketika perangkat idle, disarmed, thruster netral, dan Auto-Run mati.",
                    style = MaterialTheme.typography.bodySmall,
                )
                OutlinedTextField(
                    value = password,
                    onValueChange = onPasswordChange,
                    label = { Text("Password Wi-Fi Pico W") },
                    singleLine = true,
                    enabled = !busy,
                    visualTransformation = PasswordVisualTransformation(),
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password),
                    modifier = Modifier.fillMaxWidth(),
                )
                if (validationError != null) {
                    Text(validationError, color = MaterialTheme.colorScheme.error)
                }
                Button(
                    onClick = onUpload,
                    enabled = canUpload && validationError == null,
                    modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
                ) {
                    Text(if (busy) "Mengunggah…" else "Validasi & simpan jalur")
                }
                if (busy) LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                Text(status, style = MaterialTheme.typography.bodySmall)
            }
        }
        item {
            Text(
                "Lepas propeler saat bench test. Aplikasi tidak menyediakan tombol menjalankan misi atau menggerakkan thruster.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.error,
            )
        }
    }
}

@Composable
private fun MissionStepCard(
    index: Int,
    step: MissionStepDraft,
    enabled: Boolean,
    canMoveUp: Boolean,
    canMoveDown: Boolean,
    onChange: (MissionStepDraft) -> Unit,
    onMove: (Int) -> Unit,
    onRemove: () -> Unit,
) {
    SectionCard("Langkah ${index + 1}") {
        OutlinedTextField(
            value = step.name,
            onValueChange = { onChange(step.copy(name = it)) },
            label = { Text("Nama langkah") },
            singleLine = true,
            enabled = enabled,
            modifier = Modifier.fillMaxWidth(),
        )
        MissionActionSelector(
            action = step.action,
            enabled = enabled,
            onChange = { action ->
                onChange(
                    step.copy(
                        action = action,
                        motors = step.motors.ifEmpty { action.defaultMotors },
                    ),
                )
            },
        )
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            NumberField("Daya (%)", step.power, enabled, Modifier.weight(1f)) {
                onChange(step.copy(power = it))
            }
            NumberField("Durasi (detik)", step.durationSec, enabled, Modifier.weight(1f)) {
                onChange(step.copy(durationSec = it))
            }
        }
        NumberField("Target kedalaman (cm)", step.targetCm, enabled) {
            onChange(step.copy(targetCm = it))
        }
        Text("Thruster yang dipakai", style = MaterialTheme.typography.labelLarge)
        listOf(1..3, 4..6).forEach { group ->
            Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                group.forEach { motor ->
                    FilterChip(
                        selected = motor in step.motors,
                        onClick = {
                            val motors = if (motor in step.motors) {
                                step.motors - motor
                            } else {
                                step.motors + motor
                            }
                            onChange(step.copy(motors = motors))
                        },
                        enabled = enabled,
                        label = { Text("T$motor") },
                    )
                }
            }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(
                onClick = { onMove(-1) },
                enabled = enabled && canMoveUp,
                modifier = Modifier.weight(1f).heightIn(min = 48.dp),
            ) { Text("Naik") }
            OutlinedButton(
                onClick = { onMove(1) },
                enabled = enabled && canMoveDown,
                modifier = Modifier.weight(1f).heightIn(min = 48.dp),
            ) { Text("Turun") }
            OutlinedButton(
                onClick = onRemove,
                enabled = enabled,
                modifier = Modifier.weight(1f).heightIn(min = 48.dp),
            ) { Text("Hapus") }
        }
    }
}

@Composable
private fun MissionActionSelector(
    action: MissionAction,
    enabled: Boolean,
    onChange: (MissionAction) -> Unit,
) {
    var expanded by remember { mutableStateOf(false) }
    Box {
        OutlinedButton(
            onClick = { expanded = true },
            enabled = enabled,
            modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
        ) {
            Text("Aksi: ${action.label}")
        }
        DropdownMenu(expanded = expanded, onDismissRequest = { expanded = false }) {
            MissionAction.entries.forEach { option ->
                DropdownMenuItem(
                    text = { Text(option.label) },
                    onClick = {
                        expanded = false
                        onChange(option)
                    },
                )
            }
        }
    }
}

@Composable
private fun MonitoringCalibrationScreen(
    modifier: Modifier,
    connected: Boolean,
    snapshot: FirmwareSnapshot?,
    busy: Boolean,
    errorMessage: String?,
    infoMessage: String?,
    onGoHome: () -> Unit,
    onCalibrate: (String) -> Unit,
) {
    val device = snapshot
    if (!connected || device == null) {
        ConnectionRequiredScreen(
            modifier = modifier,
            title = "Pico W belum tersambung",
            onGoHome = onGoHome,
        )
        return
    }
    val status = device.status
    val safeForCalibration = remoteMutationReady(device) &&
        !device.dirty &&
        status.sensorsFresh
    LazyColumn(
        modifier = modifier.fillMaxWidth(),
        contentPadding = androidx.compose.foundation.layout.PaddingValues(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        errorMessage?.let { message ->
            item { MessageCard("Operasi gagal", message, isError = true) }
        }
        infoMessage?.let { message ->
            item { MessageCard("Perangkat", message) }
        }
        item { DeviceStatusCard(status) }
        item {
            SectionCard("Monitoring sensor live") {
                Text("Diperbarui otomatis sekitar sekali per detik.")
                ReadingRow("Kedalaman", "${formatReading(status.depthCm, 1)} cm")
                ReadingRow("Tekanan", "${formatReading(status.pressureMbar, 1)} mbar")
                ReadingRow("Pitch", "${formatReading(status.pitchDeg, 1)}°")
                ReadingRow("Roll", "${formatReading(status.rollDeg, 1)}°")
                ReadingRow("Suhu", "${formatReading(status.temperatureC, 1)} °C")
                ReadingRow(
                    "Umur data",
                    if (status.sensorAgeMs >= 0xFFFF_FFFFL) "belum ada sampel"
                    else "${status.sensorAgeMs} ms",
                )
                if (!status.sensorsFresh) {
                    Text(
                        "Data sensor tidak segar; kalibrasi dinonaktifkan sampai pembacaan kembali stabil.",
                        color = MaterialTheme.colorScheme.error,
                    )
                }
            }
        }
        item {
            SectionCard("Monitoring keluaran thruster") {
                status.thrusterPulsesUs.forEachIndexed { index, pulse ->
                    val neutral = device.settings.thrusters.getOrNull(index)?.pulseNeutral?.toIntOrNull()
                    ReadingRow(
                        "T${index + 1}",
                        "$pulse µs${if (neutral == pulse) " · netral" else ""}",
                    )
                }
            }
        }
        item {
            SectionCard("Kalibrasi sensor") {
                Text(
                    "Tare kedalaman dilakukan saat AUV berada di permukaan air. Kalibrasi IMU dilakukan saat AUV diam di bidang datar.",
                    style = MaterialTheme.typography.bodySmall,
                )
                Button(
                    onClick = { onCalibrate("calibrate_depth") },
                    enabled = safeForCalibration && status.pressureReady && !busy,
                    modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
                ) {
                    Text(if (status.depthCalibrated) "Kalibrasi ulang titik nol" else "Kalibrasi titik nol kedalaman")
                }
                OutlinedButton(
                    onClick = { onCalibrate("calibrate_level") },
                    enabled = safeForCalibration && status.imuReady && !busy,
                    modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
                ) {
                    Text("Kalibrasi level IMU")
                }
                if (!safeForCalibration) {
                    Text(
                        "Kalibrasi memerlukan Pico idle dan disarmed, Auto-Run nonaktif, keluaran netral, sensor segar, serta tidak ada parameter yang belum tersimpan.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.error,
                    )
                }
                Text(
                    "Kalibrasi ESC/motor tidak tersedia di Wi-Fi karena dapat menggerakkan thruster; lakukan lewat USB di bench dengan propeler dilepas.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }
    }
}

@Composable
private fun ReadingRow(label: String, value: String) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, modifier = Modifier.weight(1f))
        Text(value, style = MaterialTheme.typography.titleSmall)
    }
}

@Composable
private fun ParameterScreen(
    modifier: Modifier,
    connected: Boolean,
    snapshot: FirmwareSnapshot?,
    draft: AuvConfiguration?,
    busy: Boolean,
    errorMessage: String?,
    infoMessage: String?,
    onGoHome: () -> Unit,
    onDraftChange: (AuvConfiguration) -> Unit,
    onStage: () -> Unit,
    onApply: () -> Unit,
    onSave: () -> Unit,
    onDiscard: () -> Unit,
) {
    val device = snapshot
    val config = draft
    if (!connected || device == null || config == null) {
        ConnectionRequiredScreen(
            modifier = modifier,
            title = "Parameter belum tersedia",
            onGoHome = onGoHome,
        )
        return
    }

    LazyColumn(
        modifier = modifier.fillMaxWidth(),
        contentPadding = androidx.compose.foundation.layout.PaddingValues(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        errorMessage?.let { message ->
            item { MessageCard("Operasi gagal", message, isError = true) }
        }
        infoMessage?.let { message ->
            item { MessageCard("Pembaruan", message) }
        }
        item { DeviceStatusCard(device.status) }
        item {
            PidCard(
                title = "PID kedalaman",
                value = config.depthPid,
                enabled = !busy,
                onChange = { onDraftChange(config.copy(depthPid = it)) },
            )
        }
        item {
            PidCard(
                title = "PID roll",
                value = config.rollPid,
                enabled = !busy,
                onChange = { onDraftChange(config.copy(rollPid = it)) },
            )
        }
        item { SafetyCard(config, !busy, onDraftChange) }
        item { AutorunCard(config, !busy, onDraftChange) }
        item {
            StartupCalibrationCard(
                depthTareOnBoot = config.autoDepthTareOnBoot,
                imuLevelOnBoot = config.autoImuLevelOnBoot,
                canEdit = !busy,
                onDepthTareChange = {
                    onDraftChange(config.copy(autoDepthTareOnBoot = it))
                },
                onImuLevelChange = {
                    onDraftChange(config.copy(autoImuLevelOnBoot = it))
                },
            )
        }
        item {
            SectionCard("Batasan Wi-Fi") {
                Text(
                    "Pitch tidak didukung. Parameter hanya dapat dikirim saat idle dan disarmed; status Auto-Run tetap diubah melalui USB.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }
        items(config.thrusters, key = { it.id }) { thruster ->
            ThrusterCard(
                thruster = thruster,
                enabled = !busy,
                onChange = { updated ->
                    onDraftChange(
                        config.copy(
                            thrusters = config.thrusters.map {
                                if (it.id == updated.id) updated else it
                            },
                        ),
                    )
                },
            )
        }
        item {
            val changes = remember(device.settings, config) {
                describeChanges(device.settings, config)
            }
            WorkflowCard(
                dirty = device.dirty,
                localChanges = changes,
                enabled = !busy && device.status.idleAndDisarmed,
                canApply = device.dirty && device.status.idleAndDisarmed,
                onStage = onStage,
                onApply = onApply,
                onSave = onSave,
                onDiscard = onDiscard,
            )
        }
        item {
            Text(
                "Wi-Fi ini hanya untuk konfigurasi di atas air atau bench, bukan kontrol misi atau emergency stop. Lepas propeler saat pengujian.",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.error,
            )
        }
    }
}

@Composable
private fun FirmwareScreen(
    modifier: Modifier,
    connected: Boolean,
    filename: String?,
    password: String,
    onPasswordChange: (String) -> Unit,
    status: String,
    busy: Boolean,
    canUpload: Boolean,
    errorMessage: String?,
    infoMessage: String?,
    onGoHome: () -> Unit,
    onSelectFile: () -> Unit,
    onUpload: () -> Unit,
) {
    if (!connected) {
        ConnectionRequiredScreen(
            modifier = modifier,
            title = "Pico W belum tersambung",
            onGoHome = onGoHome,
        )
        return
    }

    LazyColumn(
        modifier = modifier.fillMaxWidth(),
        contentPadding = androidx.compose.foundation.layout.PaddingValues(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        errorMessage?.let { message ->
            item { MessageCard("Operasi gagal", message, isError = true) }
        }
        infoMessage?.let { message ->
            item { MessageCard("Pembaruan", message) }
        }
        item {
            FirmwarePanel(
                filename = filename,
                password = password,
                onPasswordChange = onPasswordChange,
                status = status,
                busy = busy,
                canUpload = canUpload,
                onSelectFile = onSelectFile,
                onUpload = onUpload,
            )
        }
    }
}

@Composable
private fun ConnectionRequiredScreen(
    modifier: Modifier,
    title: String,
    onGoHome: () -> Unit,
) {
    Column(
        modifier = modifier.fillMaxWidth().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        SectionCard(title) {
            Text("Hubungkan ke access point Pico W dari tab Ringkasan untuk melanjutkan.")
            Button(
                onClick = onGoHome,
                modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
            ) {
                Text("Buka Ringkasan")
            }
        }
    }
}

@Composable
private fun ConnectionCard(
    ssid: String,
    onSsidChange: (String) -> Unit,
    password: String,
    onPasswordChange: (String) -> Unit,
    connected: Boolean,
    busy: Boolean,
    status: String,
    rememberPassword: Boolean,
    credentialsLoaded: Boolean,
    credentialError: String?,
    onRememberPasswordChange: (Boolean) -> Unit,
    onClearCredentials: () -> Unit,
    hasSavedCredentials: Boolean,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit,
) {
    var passwordVisible by remember { mutableStateOf(false) }
    SectionCard("Access point Pico W") {
        Text(status, style = MaterialTheme.typography.titleSmall)
        if (connected) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.End,
            ) {
                OutlinedButton(onClick = onDisconnect, enabled = !busy) {
                    Text("Putuskan")
                }
            }
        } else {
            Text(
                "SSID tidak disiarkan. Masukkan nama SAUVAC-XXXX dari serial monitor Pico W, lalu hubungkan dari aplikasi.",
                style = MaterialTheme.typography.bodySmall,
            )
            OutlinedTextField(
                value = ssid,
                onValueChange = onSsidChange,
                label = { Text("SSID SAUVAC") },
                singleLine = true,
                enabled = !busy,
                modifier = Modifier.fillMaxWidth(),
            )
            OutlinedTextField(
                value = password,
                onValueChange = onPasswordChange,
                label = { Text("Password Wi-Fi") },
                singleLine = true,
                enabled = !busy,
                visualTransformation = if (passwordVisible) {
                    VisualTransformation.None
                } else {
                    PasswordVisualTransformation()
                },
                keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password),
                modifier = Modifier.fillMaxWidth(),
                trailingIcon = {
                    TextButton(onClick = { passwordVisible = !passwordVisible }) {
                        Text(if (passwordVisible) "Sembunyikan" else "Tampilkan")
                    }
                },
            )
            Row(
                modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                Column(modifier = Modifier.weight(1f)) {
                    Text("Simpan sandi")
                    Text(
                        "Dienkripsi di perangkat dengan Android Keystore.",
                        style = MaterialTheme.typography.bodySmall,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                Switch(
                    checked = rememberPassword,
                    onCheckedChange = onRememberPasswordChange,
                    enabled = !busy && credentialsLoaded,
                )
            }
            if (credentialError != null) {
                Text(credentialError, color = MaterialTheme.colorScheme.error)
                OutlinedButton(onClick = onClearCredentials, enabled = !busy) {
                    Text("Hapus sandi tersimpan")
                }
            } else if (hasSavedCredentials) {
                OutlinedButton(onClick = onClearCredentials, enabled = !busy) {
                    Text("Hapus sandi tersimpan")
                }
            }
            Button(onClick = onConnect, enabled = !busy && credentialsLoaded) {
                Text(if (busy) "Menghubungkan…" else "Hubungkan ke Pico W")
            }
        }
    }
}

@Composable
private fun FirmwarePanel(
    filename: String?,
    password: String,
    onPasswordChange: (String) -> Unit,
    status: String,
    busy: Boolean,
    canUpload: Boolean,
    onSelectFile: () -> Unit,
    onUpload: () -> Unit,
) {
    SectionCard("Pembaruan firmware lewat Wi-Fi") {
        Text(
            "Pilih firmware Pico W .bin (maks. 800 KiB). UF2 dan firmware OTA pertama tetap memerlukan BOOTSEL/USB.",
            style = MaterialTheme.typography.bodyMedium,
        )
        Text(
            "Sebelum upload, lepas propeler dan pastikan perangkat idle, disarmed, serta Auto-Run nonaktif. Firmware memeriksa kondisi aman kembali.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.error,
        )
        OutlinedButton(
            onClick = onSelectFile,
            enabled = !busy,
            modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
        ) {
            Text(filename ?: "Pilih file .bin")
        }
        OutlinedTextField(
            value = password,
            onValueChange = onPasswordChange,
            label = { Text("Password privat AP Pico W") },
            singleLine = true,
            enabled = !busy,
            visualTransformation = PasswordVisualTransformation(),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Password),
            modifier = Modifier.fillMaxWidth(),
        )
        Button(
            onClick = onUpload,
            enabled = canUpload,
            modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
        ) {
            Text(if (busy) "Mengunggah…" else "Unggah firmware & restart Pico")
        }
        if (busy) LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
        Text(status, style = MaterialTheme.typography.bodySmall)
    }
}

@Composable
private fun DeviceStatusCard(status: DeviceStatus) {
    SectionCard("Status Pico W") {
        Text(if (status.armed) "ARMED" else "DISARMED · ${fsmStateLabel(status.fsmState)}")
        val sensorAge = if (status.sensorAgeMs >= 0xFFFF_FFFFL) {
            "belum ada sampel"
        } else {
            "${status.sensorAgeMs} ms"
        }
        Text("IMU: ${readyLabel(status.imuReady)} · Tekanan: ${readyLabel(status.pressureReady)}")
        Text("Tare kedalaman: ${readyLabel(status.depthCalibrated)} · Umur sensor: $sensorAge")
        if (!status.sensorsFresh) {
            Text(
                "Data sensor tidak tersedia atau lebih tua dari 500 ms.",
                color = MaterialTheme.colorScheme.error,
            )
        }
    }
}

private fun fsmStateLabel(state: Int): String = when (state) {
    0 -> "IDLE"
    1 -> "COUNTDOWN"
    2 -> "MISSION"
    3 -> "MISSION SELESAI"
    4 -> "ABORTED"
    5 -> "KALIBRASI BOOT"
    else -> "FSM $state"
}

@Composable
private fun PidCard(
    title: String,
    value: PidDraft,
    enabled: Boolean,
    onChange: (PidDraft) -> Unit,
) {
    SectionCard(title) {
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            NumberField("Kp", value.kp, enabled, Modifier.weight(1f)) {
                onChange(value.copy(kp = it))
            }
            NumberField("Ki", value.ki, enabled, Modifier.weight(1f)) {
                onChange(value.copy(ki = it))
            }
            NumberField("Kd", value.kd, enabled, Modifier.weight(1f)) {
                onChange(value.copy(kd = it))
            }
        }
        Text("Gain harus 0–100. Perubahan belum dikirim sampai tombol Stage ditekan.")
    }
}

@Composable
private fun SafetyCard(
    config: AuvConfiguration,
    enabled: Boolean,
    onChange: (AuvConfiguration) -> Unit,
) {
    SectionCard("Batas daya dan trim") {
        NumberField("Batas daya (%)", config.powerLimit, enabled) {
            onChange(config.copy(powerLimit = it))
        }
        NumberField("Deadband ESC (µs)", config.deadbandUs, enabled) {
            onChange(config.copy(deadbandUs = it))
        }
        NumberField("Balance horizontal (%)", config.balanceHoriz, enabled) {
            onChange(config.copy(balanceHoriz = it))
        }
        NumberField("Balance vertikal (%)", config.balanceVert, enabled) {
            onChange(config.copy(balanceVert = it))
        }
    }
}

@Composable
private fun AutorunCard(
    config: AuvConfiguration,
    enabled: Boolean,
    onChange: (AuvConfiguration) -> Unit,
) {
    SectionCard("Timing autorun") {
        Text(
            "Autorun: ${if (config.autorunEnabled) "AKTIF" else "NONAKTIF"} (ubah status hanya melalui USB)",
        )
        NumberField("Delay (detik)", config.autorunDelay, enabled) {
            onChange(config.copy(autorunDelay = it))
        }
        NumberField("Tare (detik)", config.autorunTare, enabled) {
            onChange(config.copy(autorunTare = it))
        }
        NumberField("Level (detik)", config.autorunLevel, enabled) {
            onChange(config.copy(autorunLevel = it))
        }
        if (config.autorunEnabled) {
            Text("Matikan autorun melalui USB sebelum mengubah timing.", color = MaterialTheme.colorScheme.error)
        }
    }
}

@Composable
private fun StartupCalibrationCard(
    depthTareOnBoot: Boolean,
    imuLevelOnBoot: Boolean,
    canEdit: Boolean,
    onDepthTareChange: (Boolean) -> Unit,
    onImuLevelChange: (Boolean) -> Unit,
) {
    SectionCard("Kalibrasi saat boot") {
        CalibrationSwitchRow(
            title = "Nol kedalaman otomatis",
            checked = depthTareOnBoot,
            enabled = canEdit,
            onCheckedChange = onDepthTareChange,
        )
        Text(
            "Aktifkan hanya jika sensor tekanan berada di permukaan air saat dinyalakan. Nilai tekanan saat boot akan menjadi titik nol kedalaman.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        CalibrationSwitchRow(
            title = "Level IMU otomatis",
            checked = imuLevelOnBoot,
            enabled = canEdit,
            onCheckedChange = onImuLevelChange,
        )
        Text(
            "Aktifkan hanya jika AUV diam dan rata saat dinyalakan; posisi pitch/roll saat itu akan dianggap sebagai level.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Text(
            "Firmware menunggu sensor stabil hingga 15 detik. Jika tidak stabil, kalibrasi yang dipilih dilewati dan nilai sebelumnya dipertahankan. Thruster tidak digerakkan.",
            style = MaterialTheme.typography.bodySmall,
        )
        Text(
            "Perubahan berlaku setelah disimpan ke EEPROM dan perangkat dinyalakan ulang.",
            style = MaterialTheme.typography.bodySmall,
        )
    }
}

@Composable
private fun CalibrationSwitchRow(
    title: String,
    checked: Boolean,
    enabled: Boolean,
    onCheckedChange: (Boolean) -> Unit,
) {
    Row(
        modifier = Modifier.fillMaxWidth().heightIn(min = 48.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        Text(title, modifier = Modifier.weight(1f))
        Switch(
            checked = checked,
            onCheckedChange = onCheckedChange,
            enabled = enabled,
            modifier = Modifier.semantics { contentDescription = title },
        )
    }
}

@Composable
private fun ThrusterCard(
    thruster: ThrusterDraft,
    enabled: Boolean,
    onChange: (ThrusterDraft) -> Unit,
) {
    SectionCard("Thruster T${thruster.id} · GP${thruster.pin}") {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("Reverse", modifier = Modifier.weight(1f))
            Switch(
                checked = thruster.reverse,
                onCheckedChange = { onChange(thruster.copy(reverse = it)) },
                enabled = enabled,
            )
        }
        NumberField("Trim / scale", thruster.trim, enabled) {
            onChange(thruster.copy(trim = it))
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            NumberField("Min (µs)", thruster.pulseMin, enabled, Modifier.weight(1f)) {
                onChange(thruster.copy(pulseMin = it))
            }
            NumberField("Netral (µs)", thruster.pulseNeutral, enabled, Modifier.weight(1f)) {
                onChange(thruster.copy(pulseNeutral = it))
            }
            NumberField("Maks (µs)", thruster.pulseMax, enabled, Modifier.weight(1f)) {
                onChange(thruster.copy(pulseMax = it))
            }
        }
    }
}

@Composable
private fun WorkflowCard(
    dirty: Boolean,
    localChanges: List<String>,
    enabled: Boolean,
    canApply: Boolean,
    onStage: () -> Unit,
    onApply: () -> Unit,
    onSave: () -> Unit,
    onDiscard: () -> Unit,
) {
    SectionCard("Tinjau dan terapkan") {
        Text(if (dirty) "Firmware memiliki perubahan staged/belum tersimpan." else "Firmware tersinkron dengan nilai tersimpan.")
        if (localChanges.isEmpty()) {
            Text("Tidak ada draft lokal yang berubah.")
        } else {
            localChanges.forEach { Text("• $it", style = MaterialTheme.typography.bodySmall) }
        }
        if (!enabled) {
            Text("Tombol perubahan aktif hanya saat Pico W idle dan disarmed.", color = MaterialTheme.colorScheme.error)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onStage, enabled = enabled && localChanges.isNotEmpty()) {
                Text("Stage draft")
            }
            OutlinedButton(onClick = onApply, enabled = canApply && dirty) {
                Text("Apply")
            }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = onSave, enabled = canApply && dirty) {
                Text("Save EEPROM")
            }
            OutlinedButton(onClick = onDiscard, enabled = canApply && dirty) {
                Text("Discard")
            }
        }
    }
}

@Composable
private fun NumberField(
    label: String,
    value: String,
    enabled: Boolean,
    modifier: Modifier = Modifier.fillMaxWidth(),
    onValueChange: (String) -> Unit,
) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text(label) },
        singleLine = true,
        enabled = enabled,
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
        modifier = modifier,
    )
}

@Composable
private fun SectionCard(title: String, content: @Composable () -> Unit) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface),
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            Text(title, style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.primary)
            content()
        }
    }
}

@Composable
private fun MessageCard(title: String, message: String, isError: Boolean = false) {
    SectionCard(title) {
        Text(
            message,
            color = if (isError) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onSurface,
        )
    }
}

private fun Context.displayName(uri: Uri): String {
    val fromProvider = contentResolver.query(
        uri,
        arrayOf(OpenableColumns.DISPLAY_NAME),
        null,
        null,
        null,
    )?.use { cursor ->
        if (cursor.moveToFirst()) cursor.getString(0) else null
    }
    return fromProvider
        ?: uri.lastPathSegment?.substringAfterLast('/')?.substringAfterLast(':')
        ?: "firmware.bin"
}

private fun Context.readFirmwareFile(uri: Uri): ByteArray {
    val output = ByteArrayOutputStream()
    val buffer = ByteArray(8 * 1024)
    var total = 0
    val input = contentResolver.openInputStream(uri)
        ?: throw IllegalStateException("File firmware tidak dapat dibuka.")
    input.use {
        while (true) {
            val read = it.read(buffer)
            if (read < 0) break
            total += read
            require(total <= 800 * 1024) { "File firmware melebihi batas 800 KiB." }
            output.write(buffer, 0, read)
        }
    }
    require(total > 0) { "File firmware kosong." }
    return output.toByteArray()
}

private fun describeChanges(old: AuvConfiguration, new: AuvConfiguration): List<String> = buildList {
    if (old.depthPid != new.depthPid) add("PID kedalaman")
    if (old.rollPid != new.rollPid) add("PID roll")
    if (old.powerLimit != new.powerLimit) add("Batas daya")
    if (old.deadbandUs != new.deadbandUs) add("Deadband ESC")
    if (old.balanceHoriz != new.balanceHoriz) add("Balance horizontal")
    if (old.balanceVert != new.balanceVert) add("Balance vertikal")
    if (old.autorunDelay != new.autorunDelay ||
        old.autorunTare != new.autorunTare ||
        old.autorunLevel != new.autorunLevel) add("Timing autorun")
    if (old.autoDepthTareOnBoot != new.autoDepthTareOnBoot) add("Nol kedalaman saat boot")
    if (old.autoImuLevelOnBoot != new.autoImuLevelOnBoot) add("Level IMU saat boot")
    if (old.thrusters.zip(new.thrusters).any { (a, b) -> a != b }) add("Konfigurasi thruster")
}

private fun readyLabel(ready: Boolean) = if (ready) "SIAP" else "BELUM SIAP"

private fun remoteMutationReady(snapshot: FirmwareSnapshot): Boolean =
    snapshot.status.idleAndDisarmed &&
        !snapshot.settings.autorunEnabled &&
        snapshot.status.thrusterPulsesUs.size == snapshot.settings.thrusters.size &&
        snapshot.status.thrusterPulsesUs.zip(snapshot.settings.thrusters).all { (pulse, thruster) ->
            thruster.pulseNeutral.toIntOrNull() == pulse
        }

private fun formatReading(value: Double, decimals: Int): String =
    String.format(Locale.US, "%.${decimals}f", value)
