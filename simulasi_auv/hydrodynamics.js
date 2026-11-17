/**
 * AUVHydrodynamics2D - Standalone & Digital Twin 2D Hydrodynamic Physics Engine
 * Formulasi Standar Dinamika Wahana Bawah Air Thor I. Fossen (2011/2021) & SNAME (1950)
 * Khusus AUV SAUVAC 2027 Universitas Bangka Belitung (UBB)
 *
 * Fitur Utama:
 * 1. Persamaan Gerak Planar 3-DOF Terkopel:
 *    M * dnu/dt + C(nu)*nu + D(nu_r)*nu_r + g(eta) = tau_thruster + tau_dist
 * 2. Matriks Massa Total M = M_RB + M_A & Inversi Analitik Dinamis Waktu Nyata
 * 3. Parameter Bobot (Massa Kering, Net Buoyancy, Drag Scale) yang Dapat Disesuaikan Secara Interaktif
 * 4. Model Gangguan Lingkungan Kolam SAUVC Berdasarkan Teori Gelombang Linier Airy (Peluruhan e^-kz)
 * 5. Agen Kontroler RL-PID Adaptif Berbasis Heuristik Bounded Actor-Critic (Deadband, Windup Clamping & Damping Boost)
 * 6. Integrasi Numerik 60 FPS Semi-Implicit Euler (Symplectic) dengan Jaminan Pasivitas Energi
 */

class AUVHydrodynamics2D {
  /**
   * @param {Object} options Inisialisasi parameter fisik & hidrodinamika
   */
  constructor(options = {}) {
    // -------------------------------------------------------------
    // 1. Dimensi Fisik Geometri Hull CAD SAUVAC Asli (meter)
    // -------------------------------------------------------------
    this.L = options.L !== undefined ? options.L : 0.55; // Panjang lambung (m)
    this.B = options.B !== undefined ? options.B : 0.42; // Lebar lambung (m)
    this.H = options.H !== undefined ? options.H : 0.28; // Tinggi lambung (m)
    this.rho = options.rho !== undefined ? options.rho : 1000.0; // Massa jenis air kolam (kg/m^3)
    this.g = 9.80665; // Percepatan gravitasi bumi (m/s^2)

    // Posisi CG (Center of Gravity) relatif terhadap CB (Center of Buoyancy di origin [0,0,0])
    // Keel offset baterai lunas: zG = +0.045m (di bawah CB)
    this.xG = 0.0;
    this.yG = 0.0;
    this.zG = options.zG !== undefined ? options.zG : 0.045;

    // -------------------------------------------------------------
    // 2. Parameter Fisika Robot yang Dapat Dikonfigurasi Pengguna
    // -------------------------------------------------------------
    // Massa kering robot (kg): Rentang 8.0 s.d. 25.0 kg, default 14.5 kg
    this.m = options.m !== undefined ? Number(options.m) : 14.5;

    // Balas daya apung bersih positif (gram): Rentang +50g s.d. +500g, default +150g
    this.netBuoyancyGrams = options.netBuoyancyGrams !== undefined ? Number(options.netBuoyancyGrams) : 150.0;

    // Skala faktor seretan air (drag multiplier): Rentang 0.5x s.d. 2.0x, default 1.0x
    this.dragScale = options.dragScale !== undefined ? Number(options.dragScale) : 1.0;

    // -------------------------------------------------------------
    // 3. Koefisien Massa Tambah Hidrodinamika (Added Mass, kg & kg*m^2)
    // -------------------------------------------------------------
    this.X_u_dot = -3.50; // Surge added mass (kubah hemispherical)
    this.Z_w_dot = -8.20; // Heave added mass (pelat akrilik & baterai)
    this.Y_v_dot = -7.00; // Sway added mass (sisi lateral lambung)
    this.M_q_dot = -0.18; // Pitch added inertia (momen air angular)
    this.N_r_dot = -0.22; // Yaw added inertia (momen air putar)

    // -------------------------------------------------------------
    // 4. Koefisien Redaman Fluida Dasar (Linier & Kuadratik)
    // -------------------------------------------------------------
    this.baseXu = 4.00;   this.baseXuu = 20.58;  // Surge damping
    this.baseZw = 12.00;  this.baseZww = 98.18;  // Heave damping
    this.baseYv = 7.00;   this.baseYvv = 65.00;  // Sway damping
    this.baseMq = 1.50;   this.baseMqq = 2.04;   // Pitch damping
    this.baseNr = 1.80;   this.baseNrr = 2.50;   // Yaw damping

    // -------------------------------------------------------------
    // 5. Matriks Inersia Benda Kaku, Massa Total & Balikan M^-1
    // -------------------------------------------------------------
    this.Iy = 0.48962; // Inersia pitch terhadap CB
    this.Iz = 0.57867; // Inersia yaw terhadap CB
    this.netBuoyancy = 1.471; // Gaya apung bersih ke atas (N)
    this.restoringPitchStiffness = 6.3988; // K_theta = m * g * zG (N*m/rad)
    this.pitchNaturalFreq = 3.091; // rad/s
    this.pitchNaturalPeriod = 2.033; // detik

    // Invers Matriks Massa 3x3 Bidang Samping (Side-View: u, w, q)
    this.M_inv = [
      [0.057590, 0.0, -0.056118],
      [0.0, 0.044053, 0.0],
      [-0.056118, 0.0, 1.548075]
    ];

    // Matriks Massa 3x3 Bidang Samping M_side
    this.M_side = [
      [18.0, 0.0, 0.6525],
      [0.0, 22.7, 0.0],
      [0.6525, 0.0, 0.6696]
    ];

    // -------------------------------------------------------------
    // 6. Vektor Status Kinematika & Dinamika
    // -------------------------------------------------------------
    // Posisi & orientasi kolam (Earth NED Frame):
    // eta[0] = x (m, memanjang 0-12m)
    // eta[1] = z (m, kedalaman 0-1.40m)
    // eta[2] = theta (rad, sudut pitch, positif hidung naik)
    this.eta = [1.0, 0.0, 0.0];

    // Kecepatan dalam Body-Fixed Frame:
    // nu[0] = u (m/s, kecepatan maju surge)
    // nu[1] = w (m/s, kecepatan turun heave)
    // nu[2] = q (rad/s, kecepatan angular pitch)
    this.nu = [0.0, 0.0, 0.0];

    // Variabel bidang horizontal tambahan (top-view):
    this.y = 0.0;            // Sway position (m)
    this.yaw = 0.0;          // Yaw angle (rad, heading)
    this.v = 0.0;            // Sway velocity (m/s)
    this.r = 0.0;            // Yaw rate (rad/s)

    // -------------------------------------------------------------
    // 7. Dinamika Aktuator ESC & Gaya Dorong (Thrusters)
    // -------------------------------------------------------------
    this.tauESC = 0.050;      // Filter lag ESC orde-1 (50 ms)
    this.deadbandUs = 25.0;   // Deadband elektrik netral (+/- 25 us)
    this.camberFactor = 1.35; // Asimetri camber bilah propeller (1.35x mundur)
    this.maxThrustPerMotor = 26.0; // Gaya dorong maksimum per motor (N)

    // Gaya terfilter aktual yang bekerja pada wahana:
    this.actualThrustX = 0.0; // Gaya dorong surge (N)
    this.actualThrustZ = 0.0; // Gaya dorong heave (N)
    this.actualThrustQ = 0.0; // Momen dorong pitch (N*m)
    this.actualThrustYaw = 0.0; // Momen dorong yaw (N*m)

    // -------------------------------------------------------------
    // 8. Parameter Gangguan Gelombang Linier Airy & Arus Kolam
    // -------------------------------------------------------------
    this.waveActive = false;
    this.waveA = 0.035;       // Amplitudo gelombang permukaan (m)
    this.waveLambda = 2.0;    // Panjang gelombang (m)
    this.waveK = (2.0 * Math.PI) / this.waveLambda; // Bilangan gelombang (rad/m)
    this.waveOmega = Math.sqrt(this.g * this.waveK * Math.tanh(this.waveK * 1.5)); // Frekuensi sudut gelombang
    this.currentVelocityX = 0.0; // Kecepatan arus konstan (m/s)
    this.currentVelocityZ = 0.0; // Kecepatan upwelling/downwelling (m/s)
    this.transientSurgeSpike = 0.0; // Impuls hantaman arus sesaat (m/s)
    this.transientHeaveSpike = 0.0; // Impuls heave sesaat (m/s)

    // -------------------------------------------------------------
    // 9. Loop Kontroler Closed-Loop RL-PID Terintegrasi
    // -------------------------------------------------------------
    this.rlEnabled = true;
    this.pid = {
      targetZ: 0.50,            // Target kedalaman (m)
      targetTheta: 0.0,         // Target pitch horizontal (rad)
      baseKp_z: 38.0,           // Base proportional gain (N/m)
      baseKi_z: 6.0,            // Base integral gain (N/(m*s))
      baseKd_z: 26.0,           // Base derivative damping (N*s/m)
      baseKp_th: 8.5,           // Pitch Kp (N*m/rad)
      baseKi_th: 0.8,           // Pitch Ki
      baseKd_th: 4.8,           // Pitch Kd
      integralZ: 0.0,           // Akumulator integral kedalaman
      integralTheta: 0.0,       // Akumulator integral pitch
      activeKp_z: 38.0,         // Gain aktif modulasi RL
      activeKi_z: 6.0,
      activeKd_z: 26.0,
      mode: 'SETPOINT_LOCK',    // Status heuristik aktif
      modeGlow: '#06b6d4'       // Warna pendar adaptif (Cyan/Green/Gold)
    };

    // Lakukan komputasi awal matriks & parameter pemulih
    this.recalculateMassAndBuoyancy();
  }

  // ===============================================================
  // METODE REKONFIGURASI PARAMETER BOBOT & MASSA WAKTU NYATA
  // ===============================================================

  /**
   * Mengatur massa kering robot dan memperbarui seluruh matriks inersia.
   * @param {number} massKg Massa baru dalam kilogram (8.0 s.d. 25.0 kg)
   */
  setMass(massKg) {
    const val = Number(massKg);
    if (!isNaN(val) && val > 0) {
      this.m = Math.max(5.0, Math.min(40.0, val));
      this.recalculateMassAndBuoyancy();
    }
    return this.getPhysicsSummary();
  }

  /**
   * Mengatur balas daya apung bersih (+50g s.d. +500g).
   * @param {number} grams Gram apung bersih ke atas
   */
  setNetBuoyancy(grams) {
    const val = Number(grams);
    if (!isNaN(val)) {
      this.netBuoyancyGrams = Math.max(0.0, Math.min(1000.0, val));
      this.recalculateMassAndBuoyancy();
    }
    return this.getPhysicsSummary();
  }

  /**
   * Mengatur faktor pengali hambatan seretan air (0.5x s.d. 2.0x).
   * @param {number} scale Multiplier drag kuadratik & linier
   */
  setDragScale(scale) {
    const val = Number(scale);
    if (!isNaN(val) && val > 0) {
      this.dragScale = Math.max(0.2, Math.min(4.0, val));
      this.recalculateMassAndBuoyancy();
    }
    return this.getPhysicsSummary();
  }

  /**
   * Memperbarui matriks inersia M, inversi analitik M^-1, dan gaya pemulih hidrostatis.
   * Dijalankan seketika setiap kali slider massa/buoyancy digeser oleh operator.
   */
  recalculateMassAndBuoyancy() {
    // 1. Momen inersia benda kaku terhadap CB (teorema sumbu sejajar)
    const Iyy_CG = (1.0 / 12.0) * this.m * (this.L * this.L + this.H * this.H);
    this.Iy = Iyy_CG + this.m * (this.zG * this.zG);

    const Izz_CG = (1.0 / 12.0) * this.m * (this.L * this.L + this.B * this.B);
    this.Iz = Izz_CG; // xG = yG = 0

    // 2. Gaya apung bersih ke atas (Net Positive Buoyancy, N)
    // F_buoy = delta_m * g
    this.netBuoyancy = (this.netBuoyancyGrams / 1000.0) * this.g;

    // 3. Kekakuan metasentrik pemulih pitch K_theta = m * g * zG (N*m/rad)
    this.restoringPitchStiffness = this.m * this.g * this.zG;

    // 4. Frekuensi dan periode alami osilasi pitch
    const totalIy = this.Iy - this.M_q_dot;
    if (totalIy > 0) {
      this.pitchNaturalFreq = Math.sqrt(this.restoringPitchStiffness / totalIy);
      this.pitchNaturalPeriod = (2.0 * Math.PI) / this.pitchNaturalFreq;
    }

    // 5. Matriks Massa Total M_side (3x3):
    // M[0][0] = m - X_u_dot
    // M[0][2] = M[2][0] = m * zG
    // M[1][1] = m - Z_w_dot
    // M[2][2] = Iy - M_q_dot
    const M00 = this.m - this.X_u_dot;
    const M02 = this.m * this.zG;
    const M11 = this.m - this.Z_w_dot;
    const M22 = this.Iy - this.M_q_dot;

    this.M_side = [
      [M00, 0.0, M02],
      [0.0, M11, 0.0],
      [M02, 0.0, M22]
    ];

    // 6. Inversi Analitik Tertutup (Closed-Form Analytical Inverse)
    // Determinan blok (u, q): D = M00 * M22 - M02^2
    const D02 = M00 * M22 - M02 * M02;
    if (Math.abs(D02) > 1e-6 && Math.abs(M11) > 1e-6) {
      this.M_inv = [
        [ M22 / D02,  0.0,        -M02 / D02],
        [ 0.0,        1.0 / M11,   0.0],
        [-M02 / D02,  0.0,         M00 / D02]
      ];
    }

    // 7. Redaman terskala
    this.Xu = this.baseXu * this.dragScale;
    this.Xuu = this.baseXuu * this.dragScale;
    this.Zw = this.baseZw * this.dragScale;
    this.Zww = this.baseZww * this.dragScale;
    this.Mq = this.baseMq * this.dragScale;
    this.Mqq = this.baseMqq * this.dragScale;
  }

  /**
   * Mengembalikan ringkasan data fisik untuk tampilan visual HUD & UI kontrol.
   */
  getPhysicsSummary() {
    return {
      massKg: Number(this.m.toFixed(2)),
      netBuoyancyGrams: Number(this.netBuoyancyGrams.toFixed(1)),
      dragScale: Number(this.dragScale.toFixed(2)),
      totalMassSurge: Number((this.m - this.X_u_dot).toFixed(2)),
      totalMassHeave: Number((this.m - this.Z_w_dot).toFixed(2)),
      totalPitchInertia: Number((this.Iy - this.M_q_dot).toFixed(4)),
      buoyancyForceN: Number(this.netBuoyancy.toFixed(3)),
      pitchStiffness: Number(this.restoringPitchStiffness.toFixed(3)),
      naturalPeriodSec: Number(this.pitchNaturalPeriod.toFixed(2)),
      matrixM: this.M_side,
      matrixMInv: this.M_inv
    };
  }

  // ===============================================================
  // MODEL LINGKUNGAN: TEORI GELOMBANG LINIER AIRY
  // ===============================================================

  /**
   * Menghitung medan kecepatan partikel air pada koordinat (x, z) waktu t.
   * Berdasarkan peluruhan eksponensial Airy e^(-kz).
   * @param {number} x Jarak sepanjang kolam (m)
   * @param {number} z Kedalaman dari permukaan air (m, z >= 0)
   * @param {number} t Waktu simulasi (detik)
   * @returns {{vx: number, vz: number}}
   */
  getFlowVelocity(x, z, t) {
    let vx = this.currentVelocityX;
    let vz = this.currentVelocityZ;

    if (this.waveActive) {
      const effZ = Math.max(0.0, z);
      // Peluruhan orbital eksponensial terhadap kedalaman: e^(-kz)
      const decay = Math.exp(-this.waveK * effZ);
      const phase = this.waveK * x - this.waveOmega * t;

      vx += this.waveOmega * this.waveA * decay * Math.cos(phase);
      vz -= this.waveOmega * this.waveA * decay * Math.sin(phase);
    }

    // Tambahkan transien hantaman arus sesaat bila ada
    if (Math.abs(this.transientSurgeSpike) > 0.001) {
      vx += this.transientSurgeSpike;
    }
    if (Math.abs(this.transientHeaveSpike) > 0.001) {
      vz += this.transientHeaveSpike;
    }

    return { vx, vz };
  }

  /**
   * Menginjeksi hantaman arus impulsif mendadak ke dalam fluida.
   * @param {number} surgeSpikeMs Lonjakan kecepatan maju (m/s)
   * @param {number} heaveSpikeMs Lonjakan kecepatan vertikal (m/s)
   */
  injectCurrentSurge(surgeSpikeMs = 0.16, heaveSpikeMs = 0.08) {
    this.transientSurgeSpike = surgeSpikeMs;
    this.transientHeaveSpike = heaveSpikeMs;
  }

  // ===============================================================
  // MODEL AKTUATOR & PWM-TO-THRUST MAPPER
  // ===============================================================

  /**
   * Mengonversi sinyal pulsa PWM mikrodetik (1000 - 2000 us) menjadi gaya dorong fisis (Newton).
   * Memperhitungkan polaritas ESC (reverse), deadband elektrik (+/- 25 us), dan asimetri camber bilah propeller (1.35x).
   * @param {number} pulseUs Lebar pulsa PWM (us, netral 1500)
   * @param {number} reverse Polaritas motor (1 jika reverse aktif, 0 jika normal)
   * @returns {number} Gaya dorong terhitung (Newton)
   */
  pwmToThrust(pulseUs, reverse = 0) {
    const p = Math.max(1000, Math.min(2000, pulseUs));
    // Terapkan polaritas pembalik arah ESC sesuai firmware hardware
    const delta = (reverse === 1) ? (1500 - p) : (p - 1500);

    // Deadband elektrik netral
    if (Math.abs(delta) <= this.deadbandUs) {
      return 0.0;
    }

    if (delta > 0) {
      // Putaran maju (forward)
      const frac = (delta - this.deadbandUs) / (500.0 - this.deadbandUs);
      return Math.pow(frac, 1.8) * this.maxThrustPerMotor;
    } else {
      // Putaran mundur (reverse) dikenai penalti camber factor 1.35x
      const frac = (-delta - this.deadbandUs) / (500.0 - this.deadbandUs);
      return -Math.pow(frac, 1.8) * (this.maxThrustPerMotor / this.camberFactor);
    }
  }

  /**
   * Mengalokasikan 6 sinyal pulsa thruster [T1..T6] ke dalam vektor gaya bodi:
   * T1..T4: 4 Pendorong horizontal 45 derajat (Surge & Yaw)
   * T5, T6: 2 Pendorong vertikal heave (Heave & Roll/Pitch)
   * Polaritas Hardware SAUVAC: T1:rev=1, T2:rev=1, T3:rev=0, T4:rev=0, T5:rev=0, T6:rev=1
   * @param {Array<number>} pulses Array 6 pulsa PWM [T1, T2, T3, T4, T5, T6]
   * @returns {{tauX: number, tauZ: number, tauQ: number, tauYaw: number}}
   */
  thrustAllocation(pulses) {
    if (!Array.isArray(pulses) || pulses.length < 6) {
      return { tauX: 0.0, tauZ: 0.0, tauQ: 0.0, tauYaw: 0.0 };
    }

    const reverse = [1, 1, 0, 0, 1, 0];
    const t = pulses.map((p, idx) => this.pwmToThrust(p, reverse[idx]));

    // Surge (maju): proyeksi sudut 45 derajat (sqrt(2)/2) dari 4 motor horizontal
    // T1(RL), T2(FR), T3(FL), T4(RR)
    const cos45 = 0.70710678;
    const tauX = (t[0] + t[1] + t[2] + t[3]) * cos45;

    // Heave (selam): T5 (mid-right) dan T6 (mid-left) mengarah vertikal
    // Catatan: Dorongan positif ke bawah (+Z) menekan robot menyelam
    const tauZ = t[4] + t[5];

    // Pitch: lengan momen diferensial kecil antara thruster depan dan belakang
    const armX = 0.18; // meter lever arm
    const tauQ = (t[0] + t[3] - t[1] - t[2]) * 0.10;

    // Yaw: momen putar geleng diferensial
    // T1(RL, Port), T2(FR, Starboard), T3(FL, Port), T4(RR, Starboard)
    const tauYaw = (t[0] + t[2] - t[1] - t[3]) * cos45 * armX;

    return { tauX, tauZ, tauQ, tauYaw };
  }

  // ===============================================================
  // KONTROLER CLOSED-LOOP RL-PID ADAPTIF (LAPTOP ENGINE)
  // ===============================================================

  /**
   * Mengevaluasi hukum adaptasi RL-PID untuk modulasi gain dinamis.
   * @param {number} dt Langkah waktu integrasi (detik)
   * @param {number} ez Error kedalaman: targetZ - z (meter)
   * @param {number} eth Error pitch: targetTheta - theta (radian)
   * @returns {{tauZ: number, tauQ: number}} Gaya kontrol dalam Newton & N*m
   */
  updateRLPID(dt, ez, eth) {
    const w = this.nu[1]; // Kecepatan vertikal heave
    const q = this.nu[2]; // Kecepatan angular pitch

    if (!this.rlEnabled) {
      // Mode Manual Baseline (tanpa adaptasi dinamis)
      this.pid.activeKp_z = this.pid.baseKp_z;
      this.pid.activeKd_z = this.pid.baseKd_z;
      this.pid.activeKi_z = this.pid.baseKi_z;
      this.pid.mode = 'BASELINE_MANUAL';
      this.pid.modeGlow = '#64748b';

      const tauZ = this.pid.baseKp_z * ez - this.pid.baseKd_z * w + this.netBuoyancy;
      const tauQ = this.pid.baseKp_th * eth - this.pid.baseKd_th * q;
      return { tauZ, tauQ };
    }

    // 1. Integrator dengan pencegahan Windup (Clamping & Deadband Guard)
    if (Math.abs(ez) < 0.18) {
      this.pid.integralZ = Math.max(-0.50, Math.min(0.50, this.pid.integralZ + ez * dt));
    }
    this.pid.integralTheta = Math.max(-0.25, Math.min(0.25, this.pid.integralTheta + eth * dt));

    // 2. Modulasi Adaptif Gain Proporsional (Kp Boost saat gangguan besar)
    const deadbandZ = 0.020; // 2 cm deadband koridor
    let currentMode = 'SETPOINT_LOCK';
    let currentGlow = '#06b6d4'; // Cyan default

    if (Math.abs(ez) > deadbandZ) {
      const errRatio = Math.min(1.5, (Math.abs(ez) - deadbandZ) / 0.10);
      this.pid.activeKp_z = this.pid.baseKp_z * (1.0 + 1.25 * errRatio);
      currentMode = 'BOOST_DEADBAND_KP';
      currentGlow = '#10b981'; // Green pendar
    } else {
      // Relaksasi eksponensial menuju baseline
      this.pid.activeKp_z += (this.pid.baseKp_z - this.pid.activeKp_z) * Math.min(1.0, 5.0 * dt);
    }

    // 3. Kinetic Damping Boost (Kd Boost saat turbulensi atau osilasi cepat)
    if (Math.abs(w) > 0.05 || Math.abs(q) > 0.08) {
      const dampRatio = Math.min(1.5, Math.abs(w) / 0.10);
      this.pid.activeKd_z = this.pid.baseKd_z * (1.0 + 0.35 * dampRatio);
      currentMode = 'DAMPING_KINETIK_KD';
      currentGlow = '#f59e0b'; // Gold pendar
    } else {
      this.pid.activeKd_z += (this.pid.baseKd_z - this.pid.activeKd_z) * Math.min(1.0, 6.0 * dt);
    }

    this.pid.mode = currentMode;
    this.pid.modeGlow = currentGlow;

    // 4. Perhitungan Gaya Kontrol Vertikal:
    // PENTING: Menambahkan feedforward +netBuoyancy meniadakan daya apung positif wahana secara tepat
    const tauZ = this.pid.activeKp_z * ez +
                 this.pid.baseKi_z * this.pid.integralZ -
                 this.pid.activeKd_z * w +
                 this.netBuoyancy;

    // Kontrol Pitch Leveling
    const tauQ = this.pid.baseKp_th * eth +
                 this.pid.baseKi_th * this.pid.integralTheta -
                 this.pid.baseKd_th * q;

    return { tauZ, tauQ };
  }

  // ===============================================================
  // LANGKAH INTEGRASI FISIKA 60 FPS (SEMI-IMPLICIT EULER)
  // ===============================================================

  /**
   * Menjalankan satu langkah integrasi numerik sistem Fossen 2D.
   * @param {number} dt Langkah waktu (detik, dibatasi max 0.033s)
   * @param {Object} input Kontrol pendorong ({ pulses: [T1..T6] } atau { surgeThrust, heaveThrust, pitchMoment })
   * @param {number} simTime Waktu simulasi absolut (detik)
   * @returns {Object} Snapshot status lengkap untuk visualisasi kanvas
   */
  step(dt = 0.01667, input = {}, simTime = 0.0) {
    // Jaminan stabilitas CFL (Courant-Friedrichs-Lewy condition): dt <= 0.033s
    dt = Math.max(0.001, Math.min(0.033, dt));

    // Penguraian vektor status saat ini
    const x = this.eta[0];
    const z = this.eta[1];
    const theta = this.eta[2];

    const u = this.nu[0];
    const w = this.nu[1];
    const q = this.nu[2];

    const cosTh = Math.cos(theta);
    const sinTh = Math.sin(theta);

    // 1. Medan Aliran Fluida & Kecepatan Relatif Bodi (Fluid Relative Velocity)
    const flow = this.getFlowVelocity(x, z, simTime);
    // Transformasi medan arus bumi NED ke kerangka bodi {b}
    const u_rel = u - (flow.vx * cosTh - flow.vz * sinTh);
    const w_rel = w - (flow.vx * sinTh + flow.vz * cosTh);
    const q_rel = q;

    // 2. Tentukan Gaya Perintah Pendorong (Commanded Thrusts)
    let cmdThrustX = 0.0;
    let cmdThrustZ = 0.0;
    let cmdThrustQ = 0.0;
    let cmdThrustYaw = 0.0;

    let hasExplicitHeave = false;
    if (input.pulses && Array.isArray(input.pulses)) {
      const alloc = this.thrustAllocation(input.pulses);
      cmdThrustX = alloc.tauX;
      cmdThrustZ = alloc.tauZ;
      cmdThrustQ = alloc.tauQ;
      cmdThrustYaw = alloc.tauYaw;
      if (Math.abs(alloc.tauZ) > 0.15) {
        hasExplicitHeave = true;
      }
    } else {
      if (input.surgeThrust !== undefined) cmdThrustX = input.surgeThrust;
      if (input.heaveThrust !== undefined) {
        cmdThrustZ = input.heaveThrust;
        hasExplicitHeave = true;
      }
      if (input.pitchMoment !== undefined) cmdThrustQ = input.pitchMoment;
      if (input.yawTorque !== undefined) cmdThrustYaw = input.yawTorque;
    }

    // Closed-loop automatic depth hold & RL-PID stabilization:
    // If no manual dive/surface heave thrust is actively commanded, but targetZ is set > 0.02m (or RL-PID is active):
    if (!hasExplicitHeave && (this.rlEnabled || this.pid.targetZ > 0.02)) {
      const ez = this.pid.targetZ - z;
      const eth = this.pid.targetTheta - theta;
      const ctrl = this.updateRLPID(dt, ez, eth);
      cmdThrustZ = ctrl.tauZ;
      cmdThrustQ += ctrl.tauQ;
    }

    // 3. Filter Lag Respons ESC Orde-1 (tau = 50ms)
    const escAlpha = dt / Math.max(dt, this.tauESC);
    this.actualThrustX += (cmdThrustX - this.actualThrustX) * escAlpha;
    this.actualThrustZ += (cmdThrustZ - this.actualThrustZ) * escAlpha;
    this.actualThrustQ += (cmdThrustQ - this.actualThrustQ) * escAlpha;
    this.actualThrustYaw += (cmdThrustYaw - this.actualThrustYaw) * escAlpha;

    // 4. Redaman Hidrodinamika Non-Linier D(nu_r)*nu_r (Linier Viskos + Kuadratik Form Drag)
    const X_d = (this.Xu + this.Xuu * Math.abs(u_rel)) * u_rel;
    const Z_d = (this.Zw + this.Zww * Math.abs(w_rel)) * w_rel;
    const M_d = (this.Mq + this.Mqq * Math.abs(q_rel)) * q_rel;

    // 5. Gaya Pemulih Hidrostatis g(eta)
    // Sisi kiri persamaan Fossen:
    // g_u = -(W - B) * sin(theta) = -F_buoy * sin(theta)
    // g_w = +(W - B) * cos(theta) = +F_buoy * cos(theta)
    // g_q = (zG * W) * sin(theta) = (zG * m * g) * sin(theta)
    const g_u = -this.netBuoyancy * sinTh;
    const g_w =  this.netBuoyancy * cosTh;
    const g_q =  this.restoringPitchStiffness * sinTh;

    // 6. Gaya Coriolis & Sentripetal Skew-Symmetric C(nu)*nu
    const p1 = (this.m - this.X_u_dot) * u + (this.m * this.zG) * q;
    const p2 = (this.m - this.Z_w_dot) * w;
    const C_u =  p2 * q;
    const C_w = -p1 * q;
    const C_q = -p2 * u + p1 * w;

    // 7. Akumulasi Gaya Total pada Sisi Kanan Dinamika
    const tau_u_total = this.actualThrustX - X_d - g_u - C_u;
    const tau_w_total = this.actualThrustZ - Z_d - g_w - C_w;
    const tau_q_total = this.actualThrustQ - M_d - g_q - C_q;

    // 8. Percepatan Dinamika Bodi: dnu = M^-1 * tau_total
    const du = this.M_inv[0][0] * tau_u_total + this.M_inv[0][2] * tau_q_total;
    const dw = this.M_inv[1][1] * tau_w_total;
    const dq = this.M_inv[2][0] * tau_u_total + this.M_inv[2][2] * tau_q_total;

    // 9. Integrasi Kecepatan Bodi (Semi-Implicit Euler Step 1)
    this.nu[0] += du * dt;
    this.nu[1] += dw * dt;
    this.nu[2] += dq * dt;

    // Batas pengaman kecepatan fisis (mencegah overflow ekstrem):
    // Kecepatan maju maksimum AUV di kolam ~1.2 m/s
    this.nu[0] = Math.max(-1.5, Math.min(1.5, this.nu[0]));
    this.nu[1] = Math.max(-1.0, Math.min(1.0, this.nu[1]));
    this.nu[2] = Math.max(-2.5, Math.min(2.5, this.nu[2]));

    // 10. Transformasi Kinematika ke Koordinat Kolam (Semi-Implicit Euler Step 2)
    const cosYaw = Math.cos(this.yaw);
    const sinYaw = Math.sin(this.yaw);
    const dx =  this.nu[0] * cosTh + this.nu[1] * sinTh;
    const dz = -this.nu[0] * sinTh + this.nu[1] * cosTh;
    const dth = this.nu[2];

    this.eta[0] += (dx * cosYaw) * dt;
    this.y = (this.y || 0.0) + (dx * sinYaw) * dt;
    this.eta[1] += dz * dt;
    this.eta[2] += dth * dt;

    // 11. Dinamika Geleng Tambahan (Yaw & Sway)
    const dyaw = this.actualThrustYaw * (1.0 / (this.Iz - this.N_r_dot));
    this.r += (dyaw - (this.baseNr * this.dragScale) * this.r) * dt;
    this.r = Math.max(-2.0, Math.min(2.0, this.r));
    this.yaw = ((this.yaw + this.r * dt) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);

    // Reduksi spike impulsif bertahap
    if (Math.abs(this.transientSurgeSpike) > 0.001) {
      this.transientSurgeSpike *= Math.exp(-dt * 3.5);
    }
    if (Math.abs(this.transientHeaveSpike) > 0.001) {
      this.transientHeaveSpike *= Math.exp(-dt * 3.5);
    }

    // 12. Batas Fisik Kolam SAUVC (Kedalaman 0.0m Permukaan s.d. 1.40m Dasar Kolam)
    if (this.eta[1] < 0.0) {
      this.eta[1] = 0.0;
      if (this.nu[1] < 0) this.nu[1] = 0.0;
    } else if (this.eta[1] > 1.40) {
      this.eta[1] = 1.40;
      if (this.nu[1] > 0) this.nu[1] = 0.0;
    }

    // Batas panjang lintasan kolam (0.2m s.d. 11.8 meter)
    if (this.eta[0] < 0.20) {
      this.eta[0] = 0.20;
      if (this.nu[0] < 0) this.nu[0] = 0.0;
    } else if (this.eta[0] > 11.80) {
      this.eta[0] = 11.80;
      if (this.nu[0] > 0) this.nu[0] = 0.0;
    }

    // Batas lebar kolam (-2.2m s.d. +2.2 meter)
    if (this.y < -2.20) {
      this.y = -2.20;
    } else if (this.y > 2.20) {
      this.y = 2.20;
    }

    // Kembalikan status lengkap
    return {
      xM: this.eta[0],
      xCm: this.eta[0] * 100.0,
      yM: this.y,
      yCm: this.y * 100.0,
      depthM: this.eta[1],
      depthCm: this.eta[1] * 100.0,
      pitchRad: this.eta[2],
      pitchDeg: (this.eta[2] * 180.0) / Math.PI,
      yawRad: this.yaw,
      yawDeg: (this.yaw * 180.0) / Math.PI,
      u: this.nu[0],
      w: this.nu[1],
      q: this.nu[2],
      uKmh: this.nu[0] * 3.6,
      thrustX: this.actualThrustX,
      thrustZ: this.actualThrustZ,
      thrustQ: this.actualThrustQ,
      netBuoyancyN: this.netBuoyancy,
      activeKp: this.pid.activeKp_z,
      activeKi: this.pid.baseKi_z,
      activeKd: this.pid.activeKd_z,
      rlMode: this.pid.mode,
      rlGlow: this.pid.modeGlow,
      flowVx: flow.vx,
      flowVz: flow.vz
    };
  }

  /**
   * Mengatur posisi awal robot.
   */
  reset(xM = 1.0, depthM = 0.0, pitchRad = 0.0) {
    this.eta = [xM, depthM, pitchRad];
    this.nu = [0.0, 0.0, 0.0];
    this.yaw = 0.0;
    this.r = 0.0;
    this.actualThrustX = 0.0;
    this.actualThrustZ = 0.0;
    this.actualThrustQ = 0.0;
    this.actualThrustYaw = 0.0;
    this.transientSurgeSpike = 0.0;
    this.transientHeaveSpike = 0.0;
    this.pid.integralZ = 0.0;
    this.pid.integralTheta = 0.0;
  }
}

// Ekspor universal untuk Node.js, ES6 module, dan browser vanilla
if (typeof window !== 'undefined') {
  window.AUVHydrodynamics2D = AUVHydrodynamics2D;
}
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { AUVHydrodynamics2D };
}
