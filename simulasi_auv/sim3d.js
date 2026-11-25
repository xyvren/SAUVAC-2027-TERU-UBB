/**
 * AUV SAUVAC 2027 - High-Fidelity 3D Underwater Simulation Digital Twin
 * Universitas Bangka Belitung
 *
 * Built with Three.js r128, OrbitControls, and STLLoader.
 * Features:
 * - 12m x 5m x 2m SAUVC Competition Pool Environment
 * - Animated Water Surface & Dynamic Caustics
 * - SAUVC Competition Gate (PVC Pipe Frame & Buoys) at 5.0m
 * - High-fidelity Procedural AUV Digital Twin + Authentic CAD STL Asset Loader
 * - 6 Independent Spinning 3-Blade Propellers (T1..T6)
 * - Dynamic 3D Directional Thrust Vector Arrows
 * - Hydrodynamic Cavitation Micro-Bubble Particle Stream
 * - Smooth Camera Follow & Perspective Presets (Chase, Side, Top, Front, Gate)
 * - Real-time synchronization with Hydrodynamic Physics Engine & $TEL Telemetry
 */

(function () {
  'use strict';

  // Digital Twin Engine Singleton
  const Sim3D = {
    initialized: false,
    container: null,
    scene: null,
    camera: null,
    renderer: null,
    controls: null,

    // Scene Groups
    poolGroup: null,
    robotGroup: null,
    proceduralModelGroup: null,
    cadModelGroup: null,
    thrusterMeshes: [],
    propellerMeshes: [],
    thrustVectorArrows: [],
    bubbleParticles: null,

    // Environment Meshes
    waterMesh: null,
    causticsMesh: null,
    causticsTexture: null,
    causticsCanvas: null,
    causticsCtx: null,

    // States & Options
    cameraFollow: false,
    activeCameraPreset: 'pool', // 'pool', 'chase', 'side', 'top', 'front', 'gate', 'free'
    cadLoaded: false,
    modelMode: 'cad', // 'cad', 'procedural', 'hybrid'
    showThrustVectors: true,
    showBubbles: true,
    showWireframe: false,
    showGate: true,
    showCaustics: true,

    // Clocks and counters
    clock: null,
    lastFrameTime: 0,
    causticsCounter: 0,
    animationCallback: null,

    // Thruster Mounting Coordinates (Local Robot Space, in meters)
    // Authentic 6-Thruster Kinematics & Polarities (Matching SAUVAC hardware & rov_config.h)
    // T1: Rear-Left (-X, -Z), angled diagonal (yaw 45°), CW blade, reverse = 1
    // T2: Front-Right (+X, +Z), angled diagonal (yaw 135°), CW blade, reverse = 1
    // T3: Front-Left (+X, -Z), angled diagonal (yaw 45°), CCW blade, reverse = 0
    // T4: Rear-Right (-X, +Z), angled diagonal (yaw 135°), CCW blade, reverse = 0
    // T5: Mid-Right Vertical (+Z), vertical down, CW blade, reverse = 0
    // T6: Mid-Left Vertical (-Z), vertical down, CCW blade, reverse = 1
    thrusterConfigs: [
      { id: 'T1', name: 'Rear-Left (T1)', pos: [-0.217, 0.060, -0.260], yaw: Math.PI / 4, pitch: 0, type: 'horiz', bladeCw: true, reverse: 1, color: 0xfbbf24 },
      { id: 'T2', name: 'Front-Right (T2)', pos: [0.217, 0.059, 0.262], yaw: 3 * Math.PI / 4, pitch: 0, type: 'horiz', bladeCw: true, reverse: 1, color: 0xfbbf24 },
      { id: 'T3', name: 'Front-Left (T3)', pos: [0.217, 0.062, -0.261], yaw: Math.PI / 4, pitch: 0, type: 'horiz', bladeCw: false, reverse: 0, color: 0x38bdf8 },
      { id: 'T4', name: 'Rear-Right (T4)', pos: [-0.213, 0.061, 0.259], yaw: 3 * Math.PI / 4, pitch: 0, type: 'horiz', bladeCw: false, reverse: 0, color: 0x38bdf8 },
      { id: 'T5', name: 'Mid-Right Vert (T5)', pos: [-0.020, 0.080, 0.220], yaw: 0, pitch: Math.PI / 2, type: 'vert', bladeCw: true, reverse: 0, color: 0xfbbf24 },
      { id: 'T6', name: 'Mid-Left Vert (T6)', pos: [-0.020, 0.080, -0.220], yaw: 0, pitch: Math.PI / 2, type: 'vert', bladeCw: false, reverse: 1, color: 0x38bdf8 }
    ]
  };

  /**
   * Initialize 3D Engine
   */
  Sim3D.init = function (containerId) {
    if (this.initialized) return;

    this.container = document.getElementById(containerId || 'sim3dViewport');
    if (!this.container) {
      console.warn('[Sim3D] Container viewport element not found.');
      return;
    }

    if (typeof THREE === 'undefined') {
      console.error('[Sim3D] Three.js library is not loaded!');
      return;
    }

    const width = this.container.clientWidth || 1020;
    const height = this.container.clientHeight || 540;

    // 1. Scene & Underwater Fog Setup
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x102b39);
    this.scene.fog = new THREE.FogExp2(0x2a5968, 0.018);

    // 2. Camera Setup
    this.camera = new THREE.PerspectiveCamera(55, width / height, 0.1, 100.0);
    this.camera.position.set(15.0, 8.5, 11.0);

    // 3. High-Performance WebGL Renderer
    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({
        powerPreference: 'high-performance',
        antialias: true,
        alpha: true
      });
    } catch (error) {
      this.initialized = false;
      this.container.innerHTML =
        '<div style="height:100%;display:grid;place-items:center;text-align:center;color:#f87171;padding:24px">' +
        '<div><strong>WebGL 3D tidak dapat dimulai</strong><br><small>' +
        'Aktifkan hardware/WebGL pada browser, update driver GPU, atau buka aplikasi di Chromium/Edge.' +
        '</div></div>';
      console.error('[Sim3D] WebGL renderer unavailable:', error);
      window.Sim3DWebGLError = error;
      return;
    }
    this.renderer = renderer;
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;
    this.renderer.shadowMap.enabled = false; // Disable heavy shadow maps for silky 60fps

    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);
    this.renderer.domElement.style.outline = 'none';

    // 4. OrbitControls
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.07;
      this.controls.minDistance = 0.6;
      this.controls.maxDistance = 30.0;
      this.controls.maxPolarAngle = Math.PI / 2 + 0.08; // Keep camera inside/above pool water
      this.controls.target.set(6.0, -0.8, 0.0);

      this.controls.addEventListener('start', () => {
        if (Sim3D.cameraFollow) {
          Sim3D.cameraFollow = false;
          Sim3D.activeCameraPreset = 'free';
          const followButton = document.getElementById('btn3dFollow');
          if (followButton) followButton.textContent = 'Ikuti kamera: mati';
          document.querySelectorAll('.cam-preset-btn').forEach(btn => btn.classList.remove('active'));
          const freeButton = document.getElementById('btnCamFree');
          if (freeButton) freeButton.classList.add('active');
        }
      });
    }

    this.clock = new THREE.Clock();

    // 5. Lighting Setup
    this.setupLighting();

    // 6. Build Competition Pool Environment
    this.buildPoolEnvironment();

    // 7. Build Robot Model Hierarchies
    this.buildRobotGroup();
    this.setCameraPreset('pool');

    // 8. Build Cavitation Particle Engine
    this.buildCavitationSystem();

    // 9. Load Authentic CAD STL Asset Asynchronously
    this.loadAuthenticCadStl();

    // Render while the viewport is visible and the simulation is running.
    this.animationCallback = this.animate.bind(this);
    document.addEventListener('visibilitychange', () => this.syncAnimationState());

    // 11. Window Resize Handling
    window.addEventListener('resize', this.onResize.bind(this));
    if (typeof ResizeObserver !== 'undefined') {
      const ro = new ResizeObserver(() => this.onResize());
      ro.observe(this.container);
    }

    this.initialized = true;
    window.Sim3DReady = true;
    this.onResize();
    this.syncAnimationState();
    console.log('[Sim3D] AUV SAUVAC 3D Digital Twin Engine successfully initialized.');
  };

  Sim3D.syncAnimationState = function () {
    if (!this.renderer || !this.container || !this.animationCallback) return;
    const tabVisible = !document.querySelector('#tab-simulasi') ||
      document.querySelector('#tab-simulasi').classList.contains('active');
    const viewportVisible = this.container.isConnected &&
      this.container.getClientRects().length > 0 &&
      getComputedStyle(this.container).display !== 'none';
    const visible = document.visibilityState === 'visible' &&
      tabVisible &&
      viewportVisible;
    this.renderer.setAnimationLoop(visible ? this.animationCallback : null);
    if (visible) this.animate();
  };

  /**
   * Lighting: Sunlight through surface, ambient underwater scatter, and caustics fill
   */
  Sim3D.setupLighting = function () {
    // Soft cyan ambient fill
    const ambientLight = new THREE.AmbientLight(0x7ca9b8, 1.35);
    this.scene.add(ambientLight);

    // Sunlight from pool surface
    const sunLight = new THREE.DirectionalLight(0xd8f4ff, 2.1);
    sunLight.position.set(6.0, 8.0, 2.0);
    this.scene.add(sunLight);

    // Front/Side fill light
    const fillLight = new THREE.DirectionalLight(0x37a9c5, 0.75);
    fillLight.position.set(6.0, -1.0, -6.0);
    this.scene.add(fillLight);

    // Under-robot upward soft bounce
    const bounceLight = new THREE.DirectionalLight(0x3b889b, 0.45);
    bounceLight.position.set(6.0, -4.0, 0.0);
    this.scene.add(bounceLight);
  };

  /**
   * Procedural Textures Generation (Pool floor tiles, depth marks, caustics)
   */
  Sim3D.createTileTexture = function () {
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 512;
    const ctx = canvas.getContext('2d');

    // Olympic blue base
    ctx.fillStyle = '#16455a';
    ctx.fillRect(0, 0, 512, 512);

    const tileSize = 32;
    for (let y = 0; y < 512; y += tileSize) {
      for (let x = 0; x < 512; x += tileSize) {
        // Subtle color variation per tile
        const shade = Math.floor(Math.random() * 14);
        ctx.fillStyle = `rgb(${22 + shade}, ${66 + shade * 2}, ${91 + shade * 3})`;
        ctx.fillRect(x + 1, y + 1, tileSize - 2, tileSize - 2);

        // Grout line
        ctx.strokeStyle = '#0c2e3b';
        ctx.lineWidth = 1;
        ctx.strokeRect(x + 0.5, y + 0.5, tileSize - 1, tileSize - 1);
      }
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(12, 5);
    return texture;
  };

  /**
   * Competition Pool Environment (12m x 5m x 2m)
   */
  Sim3D.buildPoolEnvironment = function () {
    this.poolGroup = new THREE.Group();
    this.scene.add(this.poolGroup);

    // 1. Tiled pool floor and a solid concrete base.
    const floorGeo = new THREE.PlaneGeometry(12.0, 5.0);
    const floorTex = this.createTileTexture();
    const floorMat = new THREE.MeshStandardMaterial({
      map: floorTex,
      roughness: 0.78,
      metalness: 0.02
    });
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.position.set(6.0, -2.0, 0.0);
    this.poolGroup.add(floorMesh);

    const baseMat = new THREE.MeshStandardMaterial({
      color: 0x225062,
      roughness: 0.88,
      metalness: 0.02
    });
    const floorBase = new THREE.Mesh(new THREE.BoxGeometry(12.16, 0.18, 5.16), baseMat);
    floorBase.position.set(6.0, -2.09, 0.0);
    this.poolGroup.add(floorBase);

    // 2. Center Olympic Lane Line (Black Line on floor along Z = 0)
    const laneGeo = new THREE.PlaneGeometry(11.0, 0.24);
    const laneMat = new THREE.MeshBasicMaterial({ color: 0x061523 });
    const laneMesh = new THREE.Mesh(laneGeo, laneMat);
    laneMesh.rotation.x = -Math.PI / 2;
    laneMesh.position.set(6.0, -1.995, 0.0);
    this.poolGroup.add(laneMesh);

    // Crosshairs & Distance Crossbars on Floor
    for (let cm = 100; cm <= 1100; cm += 100) {
      const m = cm / 100;
      const isMajor = cm % 200 === 0;
      const crossGeo = new THREE.PlaneGeometry(0.12, isMajor ? 1.0 : 0.5);
      const crossMat = new THREE.MeshBasicMaterial({
        color: isMajor ? 0xe85a24 : 0x0284c7,
        transparent: true,
        opacity: 0.8
      });
      const crossMesh = new THREE.Mesh(crossGeo, crossMat);
      crossMesh.rotation.x = -Math.PI / 2;
      crossMesh.position.set(m, -1.994, 0.0);
      this.poolGroup.add(crossMesh);
    }

    // 3. Full-height structural walls give the basin visible depth and thickness.
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x3b7280,
      roughness: 0.72,
      metalness: 0.04
    });

    const backWallGeo = new THREE.BoxGeometry(12.0, 2.0, 0.16);
    const backWall = new THREE.Mesh(backWallGeo, wallMat);
    backWall.position.set(6.0, -1.0, 2.5);
    this.poolGroup.add(backWall);

    const startWallGeo = new THREE.BoxGeometry(0.16, 2.0, 5.0);
    const startWall = new THREE.Mesh(startWallGeo, wallMat);
    startWall.position.set(0.0, -1.0, 0.0);
    this.poolGroup.add(startWall);

    const endWall = new THREE.Mesh(startWallGeo, wallMat);
    endWall.position.set(12.0, -1.0, 0.0);
    this.poolGroup.add(endWall);

    // Front viewing wall uses a thin glass shell rather than an opaque plane.
    const frontWallGeo = new THREE.BoxGeometry(12.0, 2.0, 0.12);
    const glassMat = new THREE.MeshPhysicalMaterial({
      color: 0x4ab6ce,
      transparent: true,
      opacity: 0.20,
      roughness: 0.14,
      metalness: 0.02,
      transmission: 0.55,
      depthWrite: false,
      side: THREE.DoubleSide
    });
    const frontWall = new THREE.Mesh(frontWallGeo, glassMat);
    frontWall.position.set(6.0, -1.0, -2.5);
    this.poolGroup.add(frontWall);

    // 4. Pool Perimeter Coping / Rim (Deck edge at Y = 0 to 0.1m)
    const rimMat = new THREE.MeshStandardMaterial({ color: 0x385560, roughness: 0.82 });
    const rimGeoLong = new THREE.BoxGeometry(12.4, 0.15, 0.3);
    const rimBack = new THREE.Mesh(rimGeoLong, rimMat);
    rimBack.position.set(6.0, 0.075, 2.65);
    this.poolGroup.add(rimBack);

    const rimFront = new THREE.Mesh(rimGeoLong, rimMat);
    rimFront.position.set(6.0, 0.075, -2.65);
    this.poolGroup.add(rimFront);

    const rimGeoShort = new THREE.BoxGeometry(0.3, 0.15, 5.6);
    const rimStart = new THREE.Mesh(rimGeoShort, rimMat);
    rimStart.position.set(-0.15, 0.075, 0.0);
    this.poolGroup.add(rimStart);

    const rimEnd = new THREE.Mesh(rimGeoShort, rimMat);
    rimEnd.position.set(12.15, 0.075, 0.0);
    this.poolGroup.add(rimEnd);

    // Concrete walkways make the pool read as a built basin rather than a floating plane.
    const deckMat = new THREE.MeshStandardMaterial({
      color: 0x52686f,
      roughness: 0.92,
      metalness: 0.02
    });
    const longDeckGeo = new THREE.BoxGeometry(12.8, 0.16, 0.9);
    const shortDeckGeo = new THREE.BoxGeometry(0.9, 0.16, 5.8);
    [-1, 1].forEach(side => {
      const deck = new THREE.Mesh(longDeckGeo, deckMat);
      deck.position.set(6.0, 0.08, side * 3.05);
      this.poolGroup.add(deck);
    });
    [-1, 1].forEach(end => {
      const deck = new THREE.Mesh(shortDeckGeo, deckMat);
      deck.position.set(end < 0 ? -0.55 : 12.55, 0.08, 0.0);
      this.poolGroup.add(deck);
    });

    // 5. Water Surface (Y = 0, dynamic wave vertices)
    const waterGeo = new THREE.PlaneGeometry(12.0, 5.0, 64, 28);
    const waterMat = new THREE.MeshPhysicalMaterial({
      color: 0x2293ad,
      transparent: true,
      opacity: 0.42,
      roughness: 0.24,
      metalness: 0.02,
      depthWrite: false,
      side: THREE.DoubleSide
    });
    this.waterMesh = new THREE.Mesh(waterGeo, waterMat);
    this.waterMesh.rotation.x = -Math.PI / 2;
    this.waterMesh.position.set(6.0, 0.0, 0.0);
    this.poolGroup.add(this.waterMesh);

    // 6. Dynamic Animated Caustics Overlay on Pool Floor
    this.setupCausticsPlane();

    // 7. SAUVC Competition Gate at 5.0m
    this.buildSauvcGate();

    // 8. 3D Depth Measuring Mast & Distance Floats
    this.buildDepthMastAndFloats();
  };

  /**
   * Procedural Dancing Caustics on Pool Floor
   */
  Sim3D.setupCausticsPlane = function () {
    this.causticsCanvas = document.createElement('canvas');
    this.causticsCanvas.width = 160;
    this.causticsCanvas.height = 160;
    this.causticsCtx = this.causticsCanvas.getContext('2d');

    this.causticsTexture = new THREE.CanvasTexture(this.causticsCanvas);
    this.causticsTexture.wrapS = THREE.RepeatWrapping;
    this.causticsTexture.wrapT = THREE.RepeatWrapping;
    this.causticsTexture.repeat.set(6, 3);

    const causticsMat = new THREE.MeshBasicMaterial({
      map: this.causticsTexture,
      transparent: true,
      opacity: 0.65,
      blending: THREE.AdditiveBlending,
      depthWrite: false
    });

    const causticsGeo = new THREE.PlaneGeometry(11.9, 4.9);
    this.causticsMesh = new THREE.Mesh(causticsGeo, causticsMat);
    this.causticsMesh.rotation.x = -Math.PI / 2;
    this.causticsMesh.position.set(6.0, -1.985, 0.0);
    this.poolGroup.add(this.causticsMesh);
  };

  /**
   * Update Caustics Texture Canvas
   */
  Sim3D.updateCaustics = function (timeSec) {
    if (!this.causticsCtx) return;
    const ctx = this.causticsCtx;
    const w = 160, h = 160;
    const imgData = ctx.createImageData(w, h);
    const data = imgData.data;

    const t1 = timeSec * 1.8;
    const t2 = timeSec * 2.4;
    const t3 = timeSec * 1.2;

    let p = 0;
    for (let y = 0; y < h; y++) {
      const ny = y * 0.12;
      for (let x = 0; x < w; x++) {
        const nx = x * 0.12;
        const v = Math.sin(nx * 1.5 + t1) + Math.cos(ny * 1.8 - t2) + Math.sin((nx + ny) * 1.2 + t3);
        const intensity = Math.max(0, Math.pow(1.0 - Math.abs(v * 0.33), 3.2));
        const val = Math.floor(intensity * 180);

        data[p] = Math.min(255, val + 20);     // R
        data[p + 1] = Math.min(255, val + 60); // G
        data[p + 2] = 255;                    // B
        data[p + 3] = val;                    // Alpha
        p += 4;
      }
    }

    ctx.putImageData(imgData, 0, 0);
    if (this.causticsTexture) this.causticsTexture.needsUpdate = true;
  };

  /**
   * SAUVC Competition Gate (Yellow PVC Frame + Orange Float Buoys at 5.0m)
   */
  Sim3D.buildSauvcGate = function () {
    const gateGroup = new THREE.Group();
    gateGroup.position.set(5.0, 0.0, 0.0); // 5.0m along pool

    const pvcMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15, // Bright yellow PVC
      roughness: 0.25,
      metalness: 0.1
    });

    const buoyMat = new THREE.MeshStandardMaterial({
      color: 0xe85a24, // UBB Orange
      roughness: 0.3,
      metalness: 0.2
    });

    const poleRadius = 0.035;
    const gateHeight = 1.6; // From -1.9m floor up to -0.3m depth
    const gateWidth = 1.50;  // 1.5m wide

    // Left vertical pole (Z = -0.75m)
    const poleGeo = new THREE.CylinderGeometry(poleRadius, poleRadius, gateHeight, 16);
    const poleL = new THREE.Mesh(poleGeo, pvcMat);
    poleL.position.set(0, -1.1, -gateWidth / 2);
    gateGroup.add(poleL);

    // Right vertical pole (Z = +0.75m)
    const poleR = new THREE.Mesh(poleGeo, pvcMat);
    poleR.position.set(0, -1.1, gateWidth / 2);
    gateGroup.add(poleR);

    // Top crossbar (horizontal across Z, at Y = -0.30m)
    const barGeo = new THREE.CylinderGeometry(poleRadius, poleRadius, gateWidth, 16);
    const topBar = new THREE.Mesh(barGeo, pvcMat);
    topBar.rotation.x = Math.PI / 2;
    topBar.position.set(0, -0.30, 0);
    gateGroup.add(topBar);

    // Bottom anchor frame
    const bottomBar = new THREE.Mesh(barGeo, pvcMat);
    bottomBar.rotation.x = Math.PI / 2;
    bottomBar.position.set(0, -1.90, 0);
    gateGroup.add(bottomBar);

    // Two Top Orange Float Marker Buoys
    const buoyGeo = new THREE.SphereGeometry(0.12, 16, 16);
    const buoyL = new THREE.Mesh(buoyGeo, buoyMat);
    buoyL.position.set(0, -0.22, -gateWidth / 2);
    gateGroup.add(buoyL);

    const buoyR = new THREE.Mesh(buoyGeo, buoyMat);
    buoyR.position.set(0, -0.22, gateWidth / 2);
    gateGroup.add(buoyR);

    // Glowing Qualification Target Laser / Plane inside gate
    const gateTargetGeo = new THREE.PlaneGeometry(0.02, gateHeight - 0.2);
    const laserMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      transparent: true,
      opacity: 0.75
    });
    const laserL = new THREE.Mesh(gateTargetGeo, laserMat);
    laserL.position.set(0, -1.1, -gateWidth / 2 + 0.05);
    gateGroup.add(laserL);

    const laserR = new THREE.Mesh(gateTargetGeo, laserMat);
    laserR.position.set(0, -1.1, gateWidth / 2 - 0.05);
    gateGroup.add(laserR);

    this.poolGroup.add(gateGroup);
  };

  /**
   * 3D Depth Mast & Surface Lane Floats
   */
  Sim3D.buildDepthMastAndFloats = function () {
    // Vertical Depth Ruler at X = 0.2m, Z = -2.3m
    const rulerGroup = new THREE.Group();
    rulerGroup.position.set(0.2, 0.0, -2.35);

    const mastGeo = new THREE.CylinderGeometry(0.025, 0.025, 2.1, 16);
    const mastMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.7, roughness: 0.3 });
    const mast = new THREE.Mesh(mastGeo, mastMat);
    mast.position.y = -1.0;
    rulerGroup.add(mast);

    // Graduated depth tick rings (every 25cm down to 200cm)
    for (let d = 0; d <= 200; d += 25) {
      const ringGeo = new THREE.TorusGeometry(0.035, 0.008, 8, 24);
      const isMajor = d % 50 === 0;
      const ringMat = new THREE.MeshBasicMaterial({ color: isMajor ? 0xe85a24 : 0x38bdf8 });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.rotation.x = Math.PI / 2;
      ring.position.y = -(d / 100);
      rulerGroup.add(ring);
    }
    this.poolGroup.add(rulerGroup);

    // Surface Floating Lane Marker Buoys (Red/White alternating along edges)
    const floatGroup = new THREE.Group();
    for (let x = 0.5; x <= 11.5; x += 0.5) {
      const isRed = Math.round(x * 2) % 2 === 0;
      const fMat = new THREE.MeshStandardMaterial({
        color: isRed ? 0xd64545 : 0xf8fafc,
        roughness: 0.2
      });
      const fGeo = new THREE.SphereGeometry(0.055, 12, 12);

      const fBack = new THREE.Mesh(fGeo, fMat);
      fBack.position.set(x, 0.02, 2.4);
      floatGroup.add(fBack);

      const fFront = new THREE.Mesh(fGeo, fMat);
      fFront.position.set(x, 0.02, -2.4);
      floatGroup.add(fFront);
    }
    this.poolGroup.add(floatGroup);
  };

  /**
   * Build AUV SAUVAC Robot Groups (Procedural + CAD Hierarchy)
   */
  Sim3D.buildRobotGroup = function () {
    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);

    // Procedural Fallback Model Group
    this.proceduralModelGroup = new THREE.Group();
    this.robotGroup.add(this.proceduralModelGroup);

    // Authentic CAD STL Model Group (loaded asynchronously)
    this.cadModelGroup = new THREE.Group();
    this.cadModelGroup.visible = false; // Turned on when STL finishes loading
    this.robotGroup.add(this.cadModelGroup);

    // Build the high-fidelity procedural twin immediately
    this.buildProceduralRobotModel();

    // Build 6 Actuator Propellers & Thrust Vectors at exact T1..T6 locations
    this.buildActuatorsAndThrustVectors();
  };

  /**
   * High-Fidelity Procedural AUV SAUVAC Digital Twin
   */
  Sim3D.buildProceduralRobotModel = function () {
    const pGroup = this.proceduralModelGroup;

    // Materials
    const navyHullMat = new THREE.MeshPhysicalMaterial({
      color: 0x0f2b48,
      metalness: 0.5,
      roughness: 0.3,
      clearcoat: 0.4
    });

    const orangeFrameMat = new THREE.MeshStandardMaterial({
      color: 0xe85a24,
      metalness: 0.25,
      roughness: 0.35
    });

    const acrylicMat = new THREE.MeshPhysicalMaterial({
      color: 0x67e8f9,
      transparent: true,
      opacity: 0.45,
      roughness: 0.08,
      metalness: 0.05,
      transmission: 0.85
    });

    const metalPodMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.8,
      roughness: 0.2
    });

    // 1. Main Pressure Hull Tube (Diameter 0.17m, Length 0.52m along X)
    const hullGeo = new THREE.CylinderGeometry(0.085, 0.085, 0.48, 32);
    const hullMesh = new THREE.Mesh(hullGeo, navyHullMat);
    hullMesh.rotation.z = Math.PI / 2;
    pGroup.add(hullMesh);

    // 2. Transparent Acrylic Bow Dome (at +X forward)
    const domeGeo = new THREE.SphereGeometry(0.085, 24, 16, 0, Math.PI * 2, 0, Math.PI / 2);
    const domeMesh = new THREE.Mesh(domeGeo, acrylicMat);
    domeMesh.rotation.z = -Math.PI / 2;
    domeMesh.position.set(0.24, 0, 0);
    pGroup.add(domeMesh);

    // Interior Front Camera inside Acrylic Dome
    const camGeo = new THREE.CylinderGeometry(0.025, 0.03, 0.04, 16);
    const camMat = new THREE.MeshStandardMaterial({ color: 0x030712, metalness: 0.9, roughness: 0.1 });
    const camMesh = new THREE.Mesh(camGeo, camMat);
    camMesh.rotation.z = Math.PI / 2;
    camMesh.position.set(0.22, 0, 0);
    pGroup.add(camMesh);

    // Glowing LED optical beacon
    const ledGeo = new THREE.SphereGeometry(0.012, 12, 12);
    const ledMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const ledMesh = new THREE.Mesh(ledGeo, ledMat);
    ledMesh.position.set(0.25, 0.03, 0);
    pGroup.add(ledMesh);

    // 3. Stern Endcap (Anodized Aluminum at -X rear)
    const capGeo = new THREE.CylinderGeometry(0.086, 0.086, 0.03, 32);
    const capMesh = new THREE.Mesh(capGeo, metalPodMat);
    capMesh.rotation.z = Math.PI / 2;
    capMesh.position.set(-0.25, 0, 0);
    pGroup.add(capMesh);

    // 4. Top & Bottom CNC Machined Chassis Plates (UBB Orange)
    const plateGeo = new THREE.BoxGeometry(0.58, 0.012, 0.42);

    const topPlate = new THREE.Mesh(plateGeo, orangeFrameMat);
    topPlate.position.set(0.0, 0.11, 0.0);
    pGroup.add(topPlate);

    const bottomPlate = new THREE.Mesh(plateGeo, orangeFrameMat);
    bottomPlate.position.set(0.0, -0.11, 0.0);
    pGroup.add(bottomPlate);

    // 5. Aluminum Standoff Spacer Columns
    const postGeo = new THREE.CylinderGeometry(0.009, 0.009, 0.22, 12);
    const postPositions = [
      [0.22, 0, 0.18], [0.22, 0, -0.18],
      [-0.22, 0, 0.18], [-0.22, 0, -0.18]
    ];
    postPositions.forEach(pos => {
      const post = new THREE.Mesh(postGeo, metalPodMat);
      post.position.set(pos[0], pos[1], pos[2]);
      pGroup.add(post);
    });

    // 6. Dual Lateral Battery / Electronics Tubes (Port & Starboard)
    const tubeGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.42, 20);
    const tubeR = new THREE.Mesh(tubeGeo, metalPodMat);
    tubeR.rotation.z = Math.PI / 2;
    tubeR.position.set(0.0, 0.02, 0.15);
    pGroup.add(tubeR);

    const tubeL = new THREE.Mesh(tubeGeo, metalPodMat);
    tubeL.rotation.z = Math.PI / 2;
    tubeL.position.set(0.0, 0.02, -0.15);
    pGroup.add(tubeL);

    // 7. Shroud Ducts (Nozzles) for T1..T6
    this.thrusterConfigs.forEach(cfg => {
      const ductGroup = new THREE.Group();
      ductGroup.position.set(cfg.pos[0], cfg.pos[1], cfg.pos[2]);

      // Apply thruster orientation
      if (cfg.pitch !== 0) {
        ductGroup.rotation.x = cfg.pitch;
      } else {
        ductGroup.rotation.y = cfg.yaw;
      }

      // Shroud nozzle cylinder
      const ductGeo = new THREE.CylinderGeometry(0.048, 0.048, 0.07, 20, 1, true);
      const ductMesh = new THREE.Mesh(ductGeo, metalPodMat);
      ductMesh.rotation.x = Math.PI / 2;
      ductGroup.add(ductMesh);

      pGroup.add(ductGroup);
    });
  };

  /**
   * Build 6 Actuator Propellers & Dynamic Thrust Vector Arrows
   */
  Sim3D.buildActuatorsAndThrustVectors = function () {
    this.propellerMeshes = [];
    this.thrustVectorArrows = [];

    const propMat = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      metalness: 0.6,
      roughness: 0.25
    });

    this.thrusterConfigs.forEach((cfg, idx) => {
      // Actuator Mount Pivot Group
      const mountGroup = new THREE.Group();
      mountGroup.position.set(cfg.pos[0], cfg.pos[1], cfg.pos[2]);

      if (cfg.pitch !== 0) {
        mountGroup.rotation.x = cfg.pitch;
      } else {
        mountGroup.rotation.y = cfg.yaw;
      }

      // 1. Shroud Nozzle Duct for this thruster (always visible in CAD & procedural modes)
      const ductMat = new THREE.MeshPhysicalMaterial({
        color: 0x3b4d61, // Sleek metallic slate grey contrasting sharply with hull
        metalness: 0.85,
        roughness: 0.25,
        clearcoat: 0.3
      });
      const ductGeo = new THREE.CylinderGeometry(0.056, 0.056, 0.088, 24, 1, true);
      const duct = new THREE.Mesh(ductGeo, ductMat);
      duct.rotation.x = Math.PI / 2;
      mountGroup.add(duct);

      // Inner contrasting shroud lining
      const innerDuctGeo = new THREE.CylinderGeometry(0.053, 0.053, 0.086, 24, 1, true);
      const innerDuctMat = new THREE.MeshBasicMaterial({ color: 0x111c26, side: THREE.BackSide });
      const innerDuct = new THREE.Mesh(innerDuctGeo, innerDuctMat);
      innerDuct.rotation.x = Math.PI / 2;
      mountGroup.add(innerDuct);

      // Bold Accent Rim on Nozzle Lip (Gold for CW, Cyan for CCW)
      const rimGeo = new THREE.TorusGeometry(0.056, 0.005, 12, 32);
      const rimMat = new THREE.MeshBasicMaterial({ color: cfg.bladeCw ? 0xffcc00 : 0x00e5ff });
      const rim = new THREE.Mesh(rimGeo, rimMat);
      rim.position.z = 0.044;
      mountGroup.add(rim);

      // 2. Propeller Hub & Glowing Aerodynamic Blades
      const propGroup = new THREE.Group();
      const hubGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.038, 16);
      const hub = new THREE.Mesh(hubGeo, propMat);
      hub.rotation.x = Math.PI / 2;
      propGroup.add(hub);

      // Bullet nosecone tip on hub
      const coneHubGeo = new THREE.ConeGeometry(0.016, 0.024, 16);
      const coneHub = new THREE.Mesh(coneHubGeo, propMat);
      coneHub.position.z = 0.028;
      coneHub.rotation.x = Math.PI / 2;
      propGroup.add(coneHub);

      // Distinct glowing blade material matching physical robot:
      // CW propellers: Radiant Gold/Amber (0xffc107)
      // CCW propellers: Brilliant Cyan (0x00f5d4)
      const bladeMat = new THREE.MeshStandardMaterial({
        color: cfg.bladeCw ? 0xffea00 : 0x00f5d4,
        emissive: cfg.bladeCw ? 0xffaa00 : 0x00d5ff,
        emissiveIntensity: 0.95,
        metalness: 0.3,
        roughness: 0.15,
        depthTest: false // Ensures blades are never clipped by CAD shell
      });

      // 3 Aerodynamic Propeller Blades (filling the nozzle duct prominently)
      const bladeGeo = new THREE.BoxGeometry(0.012, 0.052, 0.018);
      for (let b = 0; b < 3; b++) {
        const blade = new THREE.Mesh(bladeGeo, bladeMat);
        blade.renderOrder = 998;
        blade.position.y = 0.026;
        blade.rotation.z = (b * Math.PI * 2) / 3;
        blade.rotation.y = cfg.bladeCw ? 0.42 : -0.42; // Camber angle reflecting CW vs CCW
        propGroup.add(blade);
      }

      // Spinning motion blur disc
      const discGeo = new THREE.RingGeometry(0.016, 0.052, 24);
      const discMat = new THREE.MeshBasicMaterial({
        color: cfg.bladeCw ? 0xffcc00 : 0x00f5d4,
        transparent: true,
        opacity: 0.45,
        depthTest: false,
        side: THREE.DoubleSide
      });
      const spinDisc = new THREE.Mesh(discGeo, discMat);
      spinDisc.renderOrder = 997;
      propGroup.add(spinDisc);

      mountGroup.add(propGroup);
      propGroup.position.z = 0.035; // Sit prominently forward at nozzle mouth
      this.robotGroup.add(mountGroup);
      this.propellerMeshes.push(propGroup);

      // 3. Dynamic 3D Directional Thrust Vector Arrow & Cavitation Jet Wake Cone
      const arrowGroup = new THREE.Group();
      arrowGroup.renderOrder = 999;
      mountGroup.add(arrowGroup);

      // Turbulent Cavitation Water Jet Stream Cone (streaming opposite to thrust)
      const jetConeGeo = new THREE.ConeGeometry(0.08, 0.65, 16, 1, true);
      const jetConeMat = new THREE.MeshBasicMaterial({
        color: 0xe0f7fa,
        transparent: true,
        opacity: 0.55,
        side: THREE.DoubleSide,
        depthWrite: false
      });
      const jetCone = new THREE.Mesh(jetConeGeo, jetConeMat);
      jetCone.position.z = -0.36;
      jetCone.rotation.x = -Math.PI / 2;
      arrowGroup.add(jetCone);

      // Glowing Halo Ring at Nozzle Exit
      const ringGeo = new THREE.RingGeometry(0.040, 0.058, 24);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0x10b981,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.92,
        depthTest: false
      });
      const haloRing = new THREE.Mesh(ringGeo, ringMat);
      haloRing.position.z = 0.048;
      arrowGroup.add(haloRing);

      // Shaft Cylinder (bold & visible)
      const shaftGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.38, 16);
      const arrowMat = new THREE.MeshBasicMaterial({
        color: 0x10b981,
        transparent: true,
        opacity: 0.94,
        depthTest: false
      });
      const shaft = new THREE.Mesh(shaftGeo, arrowMat);
      shaft.position.z = 0.24;
      shaft.rotation.x = Math.PI / 2;
      arrowGroup.add(shaft);

      // Cone Tip (broad & prominent)
      const coneGeo = new THREE.ConeGeometry(0.046, 0.12, 16);
      const cone = new THREE.Mesh(coneGeo, arrowMat);
      cone.position.z = 0.45;
      cone.rotation.x = Math.PI / 2;
      arrowGroup.add(cone);

      arrowGroup.visible = false;
      this.thrustVectorArrows.push({
        group: arrowGroup,
        shaft: shaft,
        cone: cone,
        ring: haloRing,
        jetCone: jetCone,
        jetConeMat: jetConeMat,
        mat: arrowMat,
        ringMat: ringMat,
        mountGroup: mountGroup,
        config: cfg
      });
    });
  };

  /**
   * Asynchronous Authentic CAD STL Loader (robot_sauvac.stl)
   */
  Sim3D.loadAuthenticCadStl = function () {
    const badge = document.getElementById('sim3dStlBadge');
    if (window.AndroidWifiBridge) {
      if (badge) {
        badge.textContent = 'STL: Prosedural (Android)';
        badge.style.color = 'var(--accent)';
      }
      return;
    }

    if (badge) {
      badge.textContent = 'STL: Memuat CAD (19.8MB)...';
      badge.style.color = 'var(--warning)';
      badge.style.borderColor = 'var(--warning)';
    }

    if (typeof THREE.STLLoader === 'undefined') {
      console.warn('[Sim3D] STLLoader is not available. Using procedural twin.');
      if (badge) badge.textContent = 'STL: Prosedural Fallback Aktif';
      return;
    }

    const loader = new THREE.STLLoader();
    loader.load(
      'assets/robot_sauvac.stl',
      (geometry) => {
        try {
          // Center geometry at origin (0, 0, 0)
          geometry.center();
          geometry.computeVertexNormals();

          // Authentic Navy Body PBR Material with UBB Accents
          const stlMat = new THREE.MeshPhysicalMaterial({
            color: 0x6b8f9c,
            metalness: 0.28,
            roughness: 0.46,
            clearcoat: 0.3
          });

          const cadMesh = new THREE.Mesh(geometry, stlMat);

          // Scaling & Orientation:
          // STL vertices in mm -> scale to meters (0.001)
          cadMesh.scale.set(0.001, 0.001, 0.001);

          // Rotation to align STL Bow to +X, Up to +Y, Starboard to +Z (T5/T6 on sides, T2/T3 at bow)
          cadMesh.rotation.order = 'XYZ';
          cadMesh.rotation.set(-Math.PI / 2, 0, Math.PI);

          this.cadMesh = cadMesh;
          this.cadMaterial = stlMat;
          this.cadModelGroup.add(cadMesh);
          this.cadLoaded = true;

          // Activate CAD STL model as default primary view
          this.setModelMode('cad');

          if (badge) {
            badge.textContent = 'STL: CAD Aktif (396k Poli)';
            badge.style.color = 'var(--success)';
            badge.style.borderColor = 'var(--success)';
            badge.style.background = 'rgba(45, 165, 122, 0.15)';
          }

          console.log('[Sim3D] Authentic CAD STL model loaded & positioned successfully.');
        } catch (err) {
          console.error('[Sim3D] Error parsing STL geometry:', err);
          if (badge) badge.textContent = 'STL: Error (Fallback Aktif)';
        }
      },
      (xhr) => {
        if (xhr.lengthComputable && badge) {
          const percent = Math.floor((xhr.loaded / xhr.total) * 100);
          badge.textContent = `STL: Mengunduh ${percent}%`;
        }
      },
      (error) => {
        console.warn('[Sim3D] Failed to load STL asset. Using procedural fallback.', error);
        if (badge) {
          badge.textContent = 'STL: Prosedural Fallback Aktif';
          badge.style.color = 'var(--accent)';
          badge.style.borderColor = 'var(--accent)';
        }
        this.setModelMode('procedural');
      }
    );
  };

  /**
   * Set Model View Mode: 'cad', 'procedural', or 'hybrid'
   */
  Sim3D.setModelMode = function (mode) {
    this.modelMode = mode;
    const btn = document.getElementById('btn3dCadModel');

    if (mode === 'cad' && this.cadLoaded) {
      this.cadModelGroup.visible = true;
      this.proceduralModelGroup.visible = false;
      if (btn) btn.textContent = 'Model: CAD STL';
    } else if (mode === 'procedural') {
      this.cadModelGroup.visible = false;
      this.proceduralModelGroup.visible = true;
      if (btn) btn.textContent = 'Model: Prosedural';
    } else if (mode === 'hybrid' && this.cadLoaded) {
      this.cadModelGroup.visible = true;
      this.proceduralModelGroup.visible = true;
      if (this.cadMaterial) {
        this.cadMaterial.transparent = true;
        this.cadMaterial.opacity = 0.55;
      }
      if (btn) btn.textContent = 'Model: Hibrida';
    }
  };

  Sim3D.toggleCadMode = function () {
    if (!this.cadLoaded) {
      this.setModelMode('procedural');
      return;
    }
    if (this.modelMode === 'cad') {
      this.setModelMode('procedural');
    } else if (this.modelMode === 'procedural') {
      this.setModelMode('hybrid');
    } else {
      if (this.cadMaterial) {
        this.cadMaterial.transparent = false;
        this.cadMaterial.opacity = 1.0;
      }
      this.setModelMode('cad');
    }
  };

  Sim3D.toggleWireframe = function () {
    this.showWireframe = !this.showWireframe;
    const btn = document.getElementById('btn3dWireframe');
    if (btn) btn.classList.toggle('active', this.showWireframe);

    if (this.cadMaterial) {
      this.cadMaterial.wireframe = this.showWireframe;
    }
  };

  /**
   * Hydrodynamic Cavitation Micro-Bubble System (3D Instanced Mesh Spheres)
   */
  Sim3D.buildCavitationSystem = function () {
    const bubbleCount = 350;
    const sphereGeo = new THREE.SphereGeometry(0.048, 14, 14);
    const bubbleMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      emissive: 0xdff9fb,
      emissiveIntensity: 0.95,
      roughness: 0.08,
      metalness: 0.15,
      transparent: true,
      opacity: 0.95,
      depthWrite: true
    });

    this.bubbleMesh = new THREE.InstancedMesh(sphereGeo, bubbleMat, bubbleCount);
    this.bubbleMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.bubbleMesh.frustumCulled = false;
    this.bubbleMesh.renderOrder = 998;
    this.scene.add(this.bubbleMesh);

    const dummy = new THREE.Object3D();
    dummy.position.set(0, -10, 0);
    dummy.scale.set(0, 0, 0);
    dummy.updateMatrix();

    const positions = new Float32Array(bubbleCount * 3);
    const velocities = new Float32Array(bubbleCount * 3);
    const scales = new Float32Array(bubbleCount);
    const life = new Float32Array(bubbleCount);
    const decay = new Float32Array(bubbleCount);

    for (let i = 0; i < bubbleCount; i++) {
      positions[i * 3] = 0;
      positions[i * 3 + 1] = -10;
      positions[i * 3 + 2] = 0;
      velocities[i * 3] = 0;
      velocities[i * 3 + 1] = 0;
      velocities[i * 3 + 2] = 0;
      scales[i] = 0.0;
      life[i] = 0.0;
      decay[i] = 0.22 + Math.random() * 0.25;
      this.bubbleMesh.setMatrixAt(i, dummy.matrix);
    }
    this.bubbleMesh.instanceMatrix.needsUpdate = true;

    this.bubbleDummy = dummy;
    this.bubbleData = {
      positions,
      velocities,
      scales,
      life,
      decay,
      spawnIndex: 0
    };
  };

  /**
   * Spawn a micro-bubble from a thruster world position
   */
  Sim3D.spawnBubble = function (worldPos, thrustDir, intensity) {
    if (!this.bubbleMesh) return;
    const data = this.bubbleData;
    const idx = data.spawnIndex;

    data.positions[idx * 3] = worldPos.x + (Math.random() - 0.5) * 0.05;
    data.positions[idx * 3 + 1] = worldPos.y + (Math.random() - 0.5) * 0.05;
    data.positions[idx * 3 + 2] = worldPos.z + (Math.random() - 0.5) * 0.05;

    // Realistic bubble speed: moves initially with jet, but quickly decelerates due to fluid drag
    const speed = 0.50 + intensity * 0.85;
    data.velocities[idx * 3] = thrustDir.x * speed + (Math.random() - 0.5) * 0.12;
    data.velocities[idx * 3 + 1] = thrustDir.y * speed + 0.15 + Math.random() * 0.12; // Gentle buoyancy rise
    data.velocities[idx * 3 + 2] = thrustDir.z * speed + (Math.random() - 0.5) * 0.12;

    data.scales[idx] = 0.8 + Math.random() * 1.1 + intensity * 0.6;
    data.life[idx] = 1.0;
    data.decay[idx] = 0.32 + Math.random() * 0.28;

    data.spawnIndex = (data.spawnIndex + 1) % data.life.length;
  };

  /**
   * Camera View Preset Switcher
   */
  Sim3D.setCameraPreset = function (preset) {
    this.activeCameraPreset = preset;
    const robotPos = this.robotGroup ? this.robotGroup.position : new THREE.Vector3(1, -0.5, 0);

    if (preset === 'pool') {
      this.cameraFollow = false;
      this.camera.position.set(15.0, 8.5, 11.0);
      if (this.controls) this.controls.target.set(6.0, -0.8, 0.0);
      const btn = document.getElementById('btn3dFollow');
      if (btn) btn.textContent = 'Tampak seluruh kolam';
    } else if (preset === 'chase') {
      this.cameraFollow = true;
      const btn = document.getElementById('btn3dFollow');
      if (btn) btn.textContent = 'Ikuti kamera: aktif';
    } else if (preset === 'side') {
      this.cameraFollow = false;
      this.camera.position.set(robotPos.x, -1.0, 4.2);
      if (this.controls) this.controls.target.set(robotPos.x, -1.0, 0);
    } else if (preset === 'top') {
      this.cameraFollow = false;
      this.camera.position.set(robotPos.x, 5.5, 0.01);
      if (this.controls) this.controls.target.set(robotPos.x, -0.5, 0);
    } else if (preset === 'front') {
      this.cameraFollow = false;
      this.camera.position.set(robotPos.x + 2.5, -0.5, 0);
      if (this.controls) this.controls.target.set(robotPos.x, -0.5, 0);
    } else if (preset === 'gate') {
      this.cameraFollow = false;
      this.camera.position.set(6.2, -0.5, 1.2);
      if (this.controls) this.controls.target.set(5.0, -0.8, 0.0);
    } else if (preset === 'free') {
      this.cameraFollow = false;
      const btn = document.getElementById('btn3dFollow');
      if (btn) btn.textContent = 'Ikuti kamera: mati';
    }

    document.querySelectorAll('.cam-preset-btn').forEach(btn => btn.classList.remove('active'));
    const activeButton = document.getElementById(`btnCam${preset.charAt(0).toUpperCase() + preset.slice(1)}`);
    if (activeButton) activeButton.classList.add('active');
    if (this.controls) this.controls.update();
  };

  Sim3D.toggleFollowCam = function () {
    this.setCameraPreset(this.cameraFollow ? 'pool' : 'chase');
  };

  Sim3D.resetCamera = function () {
    this.setCameraPreset('pool');
  };

  /**
   * Main Render Loop (Invoked Unconditionally)
   */
  Sim3D.animate = function () {
    const dt = Math.min(0.06, this.clock.getDelta());
    const elapsedTime = this.clock.getElapsedTime();

    // 1. Water Surface Wave Animation
    if (this.waterMesh && this.waterMesh.geometry) {
      const pos = this.waterMesh.geometry.attributes.position;
      for (let i = 0; i < pos.count; i++) {
        const u = pos.getX(i);
        const v = pos.getY(i);
        const wave =
          Math.sin(u * 1.8 + elapsedTime * 2.2) * 0.025 +
          Math.cos(v * 2.5 + elapsedTime * 1.8) * 0.015 +
          Math.sin((u + v) * 1.2 + elapsedTime * 1.5) * 0.012;
        pos.setZ(i, wave);
      }
      pos.needsUpdate = true;
    }

    // 2. Dynamic Caustics Update (~30 Hz)
    this.causticsCounter++;
    if (this.causticsCounter % 2 === 0) {
      this.updateCaustics(elapsedTime);
    }

    // 3. Read Real-time Telemetry & Hydrodynamics State
    const phys = (typeof window !== 'undefined' && window.simPhysics) ? window.simPhysics : {
      x: 100.0,
      y: 0.0,
      depth: 0.0,
      yaw: 0.0,
      pitch: 0.0,
      roll: 0.0,
      virtualMissionActive: false,
      manualDrive: 'idle',
      virtualPulses: [1500, 1500, 1500, 1500, 1500, 1500]
    };

    const appState = (typeof window !== 'undefined' && window.appState) ? window.appState : { pulses: [1500, 1500, 1500, 1500, 1500, 1500] };

    // Resolve Active Pulses
    let pulses = [1500, 1500, 1500, 1500, 1500, 1500];
    if (phys.simulationRunning === false) {
      pulses = [1500, 1500, 1500, 1500, 1500, 1500];
    } else if (phys.virtualMissionActive || phys.manualDrive !== 'idle') {
      pulses = phys.virtualPulses || pulses;
    } else if (Array.isArray(phys.appliedPulses)) {
      pulses = phys.appliedPulses;
    } else if (appState.pulses) {
      pulses = appState.pulses;
    }

    // 4. Synchronize 3D Robot Position & Attitude
    if (this.robotGroup) {
      // Metric coordinates: cm / 100 -> meters
      const targetX = (phys.x !== undefined ? phys.x : 100.0) / 100.0;
      const targetY = -(phys.depth !== undefined ? phys.depth : 0.0) / 100.0;
      const targetZ = -(phys.y !== undefined ? phys.y : 0.0) / 100.0;

      this.robotGroup.position.set(targetX, targetY, targetZ);

      // Attitude rotations
      this.robotGroup.rotation.order = 'YXZ';
      this.robotGroup.rotation.y = -(phys.yaw * Math.PI) / 180.0;
      this.robotGroup.rotation.z = (phys.pitch * Math.PI) / 180.0;
      this.robotGroup.rotation.x = (phys.roll * Math.PI) / 180.0;

      // Ensure matrixWorld is immediately updated for world coordinate queries
      this.robotGroup.updateMatrixWorld(true);

      // 5. Spin Propellers & Update Thrust Vector Arrows
      for (let i = 0; i < 6; i++) {
        const cfg = this.thrusterConfigs[i];
        const pulse = pulses[i] || 1500;

        // Calculate effective thrust according to firmware reverse polarity:
        // T1, T2, T6 have reverse = 1 (pulse < 1500 is forward/dive thrust)
        // T3, T4, T5 have reverse = 0 (pulse > 1500 is forward/dive thrust)
        const effectiveDelta = (cfg.reverse === 1) ? (1500 - pulse) : (pulse - 1500);

        const propMesh = this.propellerMeshes[i];
        const arrow = this.thrustVectorArrows[i];

        // Spin propeller blade around its central axis (local Z)
        if (propMesh && Math.abs(effectiveDelta) > 5) {
          // Clockwise when forward for bladeCw:true; Counter-clockwise for bladeCw:false
          const spinSign = cfg.bladeCw ? -1.0 : 1.0;
          propMesh.rotation.z += (effectiveDelta / 500.0) * spinSign * 55.0 * dt;
        }

        // Thrust vector arrow
        if (arrow) {
          if (Math.abs(effectiveDelta) > 15) {
            arrow.group.visible = true;
            const mag = Math.min(1.0, Math.abs(effectiveDelta) / 500.0);
            const isForward = effectiveDelta > 0;
            const dirSign = isForward ? 1.0 : -1.0;

            const thrustLength = 0.22 + mag * 0.35;
            arrow.shaft.scale.set(1, 0.6 + mag * 1.3, 1);
            arrow.shaft.position.z = (0.12 + thrustLength * 0.45) * dirSign;
            arrow.cone.position.z = (0.12 + thrustLength + 0.05) * dirSign;
            arrow.cone.rotation.x = isForward ? (Math.PI / 2) : (-Math.PI / 2);

            // Forward/Dive -> Emerald Green; Reverse/Surface -> Coral Red
            const colorHex = isForward ? 0x10b981 : 0xef4444;
            arrow.mat.color.setHex(colorHex);
            if (arrow.ringMat) arrow.ringMat.color.setHex(colorHex);

            // Animate rearward turbulent water jet stream cone
            if (arrow.jetCone) {
              arrow.jetCone.visible = true;
              arrow.jetCone.scale.set(0.6 + mag * 0.9, 0.4 + mag * 1.6, 0.6 + mag * 0.9);
              arrow.jetCone.position.z = (-0.15 - mag * 0.30) * dirSign;
              arrow.jetCone.rotation.x = isForward ? (-Math.PI / 2) : (Math.PI / 2);
              if (arrow.jetConeMat) {
                arrow.jetConeMat.opacity = 0.40 + Math.random() * 0.30;
              }
            }

            // Spawn Cavitation Bubbles streaming in opposite direction to thrust (Newton's 3rd Law)
            const bubbleRate = Math.floor(2 + mag * 5);
            for (let b = 0; b < bubbleRate; b++) {
              const exhaustPos = new THREE.Vector3(0, 0, -0.09 * dirSign);
              exhaustPos.applyMatrix4(arrow.mountGroup.matrixWorld);

              const jetWorldDir = new THREE.Vector3(0, 0, -dirSign);
              jetWorldDir.transformDirection(arrow.mountGroup.matrixWorld);

              this.spawnBubble(exhaustPos, jetWorldDir, mag);
            }
          } else {
            arrow.group.visible = false;
            if (arrow.jetCone) arrow.jetCone.visible = false;
          }
        }
      }
    }

    // 6. Update Cavitation Bubbles Particle Simulation
    this.updateCavitationParticles(dt);

    // 7. Dynamic Camera Follow
    if (this.cameraFollow && this.robotGroup && this.controls) {
      const rx = this.robotGroup.position.x;
      const ry = this.robotGroup.position.y;
      const rz = this.robotGroup.position.z;
      const yaw = -(phys.yaw * Math.PI) / 180.0;

      // Keep the chase camera underwater rather than filming the AUV from poolside.
      const dist = 1.0;
      const height = -0.35;
      const lateral = 0.45;

      const idealX = Math.max(0.15, Math.min(11.85, rx - Math.cos(yaw) * dist + Math.sin(yaw) * lateral));
      const idealZ = Math.max(-2.35, Math.min(2.35, rz - Math.sin(yaw) * dist - Math.cos(yaw) * lateral));
      const idealY = Math.max(-1.8, Math.min(-0.2, ry + height));
      const targetX = Math.max(0.15, Math.min(11.85, rx + Math.cos(yaw) * 0.5));
      const targetZ = Math.max(-2.35, Math.min(2.35, rz + Math.sin(yaw) * 0.5));

      this.camera.position.lerp(new THREE.Vector3(idealX, idealY, idealZ), 0.06);
      this.controls.target.lerp(new THREE.Vector3(targetX, ry, targetZ), 0.08);
      this.controls.update();
    } else if (this.controls) {
      this.controls.update();
    }

    // 8. Update Viewport HUD Readout
    this.updateHudOverlay(phys, pulses);

    // 9. Render Scene
    this.renderer.render(this.scene, this.camera);
  };

  /**
   * Update Bubble Particle System physics (Instanced Mesh Spheres)
   */
  Sim3D.updateCavitationParticles = function (dt) {
    if (!this.bubbleMesh) return;
    const data = this.bubbleData;
    const count = data.life.length;
    const dummy = this.bubbleDummy;
    let needsUpdate = false;

    for (let i = 0; i < count; i++) {
      if (data.life[i] > 0) {
        needsUpdate = true;
        // Position update
        data.positions[i * 3] += data.velocities[i * 3] * dt;
        data.positions[i * 3 + 1] += data.velocities[i * 3 + 1] * dt;
        data.positions[i * 3 + 2] += data.velocities[i * 3 + 2] * dt;

        // Water drag & buoyant lift
        data.velocities[i * 3] *= 0.88;
        data.velocities[i * 3 + 2] *= 0.88;
        data.velocities[i * 3 + 1] += 0.16 * dt; // Bubble rises gently

        // Life decay
        data.life[i] -= data.decay[i] * dt;

        // Surface pop or expired
        if (data.positions[i * 3 + 1] >= 0.0 || data.life[i] <= 0) {
          data.life[i] = 0.0;
          dummy.position.set(0, -10, 0);
          dummy.scale.set(0, 0, 0);
        } else {
          dummy.position.set(data.positions[i * 3], data.positions[i * 3 + 1], data.positions[i * 3 + 2]);
          const currentScale = data.scales[i] * Math.min(1.0, data.life[i] * 2.2);
          dummy.scale.set(currentScale, currentScale, currentScale);
        }
        dummy.updateMatrix();
        this.bubbleMesh.setMatrixAt(i, dummy.matrix);
      }
    }

    if (needsUpdate) {
      this.bubbleMesh.instanceMatrix.needsUpdate = true;
    }
  };

  /**
   * Update Sleek 3D Viewport On-Screen HUD Overlay
   */
  Sim3D.updateHudOverlay = function (phys, pulses) {
    const el = document.getElementById('sim3dTelemetryOverlay');
    if (!el) return;

    const x = (phys.x || 0).toFixed(0);
    const d = (phys.depth || 0).toFixed(1);
    const yaw = (phys.yaw || 0).toFixed(1);
    const pitch = (phys.pitch || 0).toFixed(1);
    const roll = (phys.roll || 0).toFixed(1);
    const tStr = `T1:${pulses[0]} T2:${pulses[1]} T3:${pulses[2]} T4:${pulses[3]} T5:${pulses[4]} T6:${pulses[5]}`;

    el.textContent = `POS: X: ${x}cm | DEPTH: ${d}cm   ·   ATT: YAW: ${yaw}° | PITCH: ${pitch}° | ROLL: ${roll}°   ·   THRUST: ${tStr}`;
  };

  /**
   * Resize Listener
   */
  Sim3D.onResize = function () {
    if (!this.container || !this.renderer || !this.camera) return;
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (w === 0 || h === 0) return;

    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  };

  // Expose to global window object
  window.Sim3D = Sim3D;
})();
