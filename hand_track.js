/**
 * RoboHead Hand Tracking Studio (2-Column Vision Control)
 * Column 1: 3D Robot Head Digital Twin (Three.js) — Left / Right Pan Only
 * Column 2: Camera Hand Tracking Feed (MediaPipe HandLandmarker + 3-Zone Hysteresis)
 * Serial Protocol: Exact Arduino Uno Web Serial (9600 Baud) sending LEFT / HOME / RIGHT
 */

// ============================================================================
// Constants & State Management
// ============================================================================
const TARGETS = {
  LEFT: { angle: 140, name: 'Left' },
  HOME: { angle: 90, name: 'Center' },
  RIGHT: { angle: 50, name: 'Right' }
};

const TILT_TARGETS = {
  UP: { angle: 140, name: 'Up' },
  CENTER: { angle: 90, name: 'Center' },
  DOWN: { angle: 65, name: 'Down' }
};

const VISION_CDN = 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14';
const MODEL_URL =
  'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task';

const state = {
  pan: 90,
  targetPan: 90,
  currentZone: 'HOME',
  moveTimer: null,

  tilt: 90,
  targetTilt: 90,
  currentTiltZone: 'CENTER',
  tiltTimer: null,

  // Web Serial Port (9600 baud)
  isConnected: false,
  serialPort: null,
  serialWriter: null,
  serialReader: null,
  serialReadPromise: null,
  keepReadingSerial: false,
  baudRate: 9600,

  // Hand Tracker (Two-Phase: 1. Establish Center Reference -> 2. Intentional Movement)
  hand: {
    active: false,
    stream: null,
    handLandmarker: null,
    usingLandmarker: false,
    rafId: null,
    lastVideoTime: -1,
    lastTimestamp: 0,
    hasHand: false,
    mirrorX: false,         // Default: Hand Right -> Head Left, Hand Left -> Head Right
    enableTilt: false,      // Optional Up/Down vertical tilt

    // Two-stage Reference & Movement State Machine
    // 'SEARCHING' -> 'WAIT_CENTER' -> 'CALIBRATING' -> 'READY' -> 'MOVING'
    phase: 'SEARCHING',
    refX: null,             // Established neutral X reference in center of frame
    refY: null,             // Established neutral Y reference in center of frame
    smoothX: null,          // EMA-filtered palm center X (suppresses detection jitter)
    smoothY: null,          // EMA-filtered palm center Y
    prevRawX: null,         // Previous raw X for spike/teleport rejection
    prevRawY: null,
    centerStableFrames: 0,  // Consecutive frames hand held steady in center to lock reference
    pendingZone: null,      // Candidate pan direction awaiting consecutive confirmation
    pendingFrames: 0,       // Consecutive frames confirming intentional pan movement
    pendingTiltZone: null,  // Candidate tilt direction awaiting confirmation
    pendingTiltFrames: 0,
    hasTriggeredMove: false,
    lastHandSeenAt: 0,

    // Gesture override state (Closed Hand / Fist OR Showing 1 -> Center / Home)
    centerGestureFrames: 0,
    pendingCenterGesture: null,
    activeCenterGesture: null,

    // Tuning constants (Big Center Column = 24% wide, matching original 0.38..0.62)
    centerBoxMinX: 0.36,    // Hand must enter center column (36%..64%) to set reference
    centerBoxMaxX: 0.64,
    deadZoneX: 0.12,        // ±12% around reference -> BIG 24% center dead zone column
    returnBandX: 0.085,     // Hysteresis return band inside dead zone
    deadZoneY: 0.15,        // ±15% vertical dead zone (when Up/Down is enabled)
    returnBandY: 0.10,
    emaAlpha: 0.28,         // Smooth low-pass filter against noisy landmarks
    maxJumpDelta: 0.24,     // Reject sudden unstable detection jumps
    centerLockRequired: 6,  // ~200ms steady in center to lock reference without moving robot
    intentFramesRequired: 4,// ~130ms intentional movement beyond dead zone before sending command

    // Offline fallback buffer
    prevFrameData: null,
    offCanvas: null,
    offCtx: null
  },

  // Smart USB Camera Manager (Auto-enables Robot USB Cam & disables Laptop Cam)
  camera: {
    activeDeviceId: null,
    activeLabel: '',
    isRobotCam: false,
    hasExternalCam: false,
    preferredExternalId: null,
    isSwitching: false,
    deviceChangeTimer: null
  }
};

const textEncoder = new TextEncoder();

// ============================================================================
// DOM Elements
// ============================================================================
const dom = {
  // Connection
  connIndicator: document.getElementById('connIndicator'),
  connStatusText: document.getElementById('connStatusText'),
  btnQuickConnect: document.getElementById('btnQuickConnect'),
  offlineNotice: document.getElementById('offlineNotice'),
  btnBannerConnect: document.getElementById('btnBannerConnect'),

  // Column 1: 3D Face
  threeContainer: document.getElementById('threeContainer'),
  faceViewport: document.getElementById('faceViewport'),
  hudYaw: document.getElementById('hudYaw'),
  hudPitch: document.getElementById('hudPitch'),
  servoPanVal: document.getElementById('servoPanVal'),
  servoTiltVal: document.getElementById('servoTiltVal'),
  chipPanVal: document.getElementById('chipPanVal'),
  chipTiltVal: document.getElementById('chipTiltVal'),
  chipTrackState: document.getElementById('chipTrackState'),
  btnCenterHead: document.getElementById('btnCenterHead'),

  // Column 2: Camera
  cameraViewport: document.getElementById('cameraViewport'),
  handWebcam: document.getElementById('handWebcam'),
  handCanvas: document.getElementById('handCanvas'),
  camSourceBadge: document.getElementById('camSourceBadge'),
  selCameraSource: document.getElementById('selCameraSource'),
  handStatusBadge: document.getElementById('handStatusBadge'),
  cameraGuide: document.getElementById('cameraGuide'),
  guideText: document.getElementById('guideText'),
  handMetricCoords: document.getElementById('handMetricCoords'),
  handValGesture: document.getElementById('handValGesture'),
  serialDispatchRate: document.getElementById('serialDispatchRate'),
  chkHandMirror: document.getElementById('chkHandMirror'),
  chkEnableTilt: document.getElementById('chkEnableTilt'),
  btnRestartCamera: document.getElementById('btnRestartCamera')
};

// ============================================================================
// Web Serial Communication (Exact Working 9600 Baud Arduino Protocol)
// ============================================================================

function setSerialStatusLog(text, color) {
  if (!dom.serialDispatchRate) return;
  dom.serialDispatchRate.textContent = text;
  dom.serialDispatchRate.title = text;
  if (color) {
    dom.serialDispatchRate.style.color = color;
  }
}

function updateConnectionUI(connected) {
  if (!dom.connIndicator) return;
  if (connected) {
    dom.connIndicator.classList.add('online');
    dom.connIndicator.classList.remove('error');
    if (dom.connStatusText) dom.connStatusText.textContent = 'USB ONLINE';
    if (dom.btnQuickConnect) {
      dom.btnQuickConnect.classList.add('connected');
      dom.btnQuickConnect.innerHTML = '<span class="btn-icon">⚡</span> DISCONNECT';
    }
    setSerialStatusLog('CONNECTED (9600 BAUD)', '#00ff88');
    if (dom.offlineNotice) dom.offlineNotice.style.display = 'none';
  } else {
    dom.connIndicator.classList.remove('online');
    if (dom.connStatusText) dom.connStatusText.textContent = 'DISCONNECTED';
    if (dom.btnQuickConnect) {
      dom.btnQuickConnect.classList.remove('connected');
      dom.btnQuickConnect.innerHTML = '<span class="btn-icon">⚡</span> CONNECT USB';
    }
    setSerialStatusLog('OFFLINE (CONNECT USB)', '#ff3b5c');
    if (dom.offlineNotice) dom.offlineNotice.style.display = 'flex';
  }
}

async function readSerialLoop() {
  if (!state.serialPort || !state.serialPort.readable) return;
  const decoder = new TextDecoderStream();
  const readableClosed = state.serialPort.readable.pipeTo(decoder.writable);
  state.serialReader = decoder.readable.getReader();

  let buffer = '';
  try {
    while (state.keepReadingSerial) {
      const { value, done } = await state.serialReader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split(/\r?\n/);
      buffer = lines.pop() || '';
      for (const line of lines) {
        const trimmed = line.trim();
        if (trimmed) {
          console.log('[ARDUINO RX]', trimmed);
          setSerialStatusLog(trimmed, '#00ff88');
        }
      }
    }
  } catch (err) {
    if (state.keepReadingSerial) {
      console.warn('[SERIAL READ]', err.message);
    }
  } finally {
    try {
      if (state.serialReader) state.serialReader.releaseLock();
    } catch (e) {}
    await readableClosed.catch(() => {});
  }
}

async function openSerialPort(port) {
  await port.open({ baudRate: state.baudRate });
  state.serialPort = port;
  state.serialWriter = state.serialPort.writable.getWriter();
  state.keepReadingSerial = true;
  state.isConnected = true;
  state.serialReadPromise = readSerialLoop();
  updateConnectionUI(true);

  // Arduino is now connected: immediately enable Robot USB camera & disable laptop camera
  if (state.hand.active) {
    refreshAndSwitchCamera(true).catch((err) => console.warn('[CAMERA ARDUINO SWITCH]', err));
  } else {
    startHandTracking();
  }
}

async function disconnectArduino() {
  state.keepReadingSerial = false;
  state.isConnected = false;
  try {
    if (state.serialReader) {
      await state.serialReader.cancel();
      state.serialReader = null;
    }
    if (state.serialReadPromise) {
      await state.serialReadPromise;
      state.serialReadPromise = null;
    }
    if (state.serialWriter) {
      state.serialWriter.releaseLock();
      state.serialWriter = null;
    }
    if (state.serialPort) {
      await state.serialPort.close();
      state.serialPort = null;
    }
  } catch (err) {
    console.warn('[SERIAL DISCONNECT]', err.message);
  } finally {
    state.serialPort = null;
    state.serialWriter = null;
    state.serialReader = null;
    state.isConnected = false;
    updateConnectionUI(false);
  }
}

async function autoReconnectSerial(retryCount = 0) {
  if (!('serial' in navigator)) return;
  if (state.isConnected) return;
  try {
    const ports = await navigator.serial.getPorts();
    if (ports.length > 0) {
      console.log(`[SERIAL] Paired USB port detected (attempt ${retryCount + 1}). Auto-connecting...`);
      await openSerialPort(ports[0]);
      console.log('[SERIAL] Auto-connected to Arduino Uno at 9600 baud!');
      return;
    }
  } catch (err) {
    console.warn(`[SERIAL AUTO-CONNECT] Attempt ${retryCount + 1}:`, err.message);
    if (retryCount < 3) {
      setTimeout(() => autoReconnectSerial(retryCount + 1), 450);
    }
  }
}

async function connectSerial() {
  if (!('serial' in navigator)) {
    alert(
      'Web Serial API is available in Google Chrome, Microsoft Edge, and Opera.\nPlease open this page in Chrome or Edge to connect your robot.'
    );
    return;
  }

  if (state.isConnected && state.serialPort) {
    await disconnectArduino();
    return;
  }

  try {
    const port = await navigator.serial.requestPort();
    await openSerialPort(port);
  } catch (err) {
    if (err.name === 'NotFoundError') return; // User canceled picker
    console.error('[SERIAL CONNECT ERROR]', err);
    if (err.name === 'NetworkError' || (err.message && err.message.toLowerCase().includes('open'))) {
      alert(
        '⚠️ COM Port is Busy!\n\n1. If Arduino IDE Serial Monitor is open, CLOSE IT.\n2. Close duplicate tabs.\n3. Unplug and replug the USB cable, then click CONNECT USB again.'
      );
    }
    await disconnectArduino();
  }
}

async function sendToArduino(cmd) {
  if (!state.serialWriter || !state.isConnected) return;
  try {
    await state.serialWriter.write(textEncoder.encode(cmd + '\n'));
    setSerialStatusLog(`SENT: ${cmd}`, '#00f0ff');
    console.log('[ARDUINO TX]', cmd);
  } catch (err) {
    console.error('[SERIAL SEND ERR]', err);
    setSerialStatusLog(`TX ERR: ${err.message}`, '#ff3b5c');
  }
}

if ('serial' in navigator) {
  navigator.serial.addEventListener('disconnect', () => {
    if (state.serialPort) disconnectArduino();
  });
}

// ============================================================================
// Left / Home / Right State Machine (Matches Arduino Firmware 1:1)
// ============================================================================

function moveTo(zone, { forceSend = false } = {}) {
  const target = TARGETS[zone];
  if (!target) return;

  const unchanged = state.currentZone === zone && state.targetPan === target.angle;
  if (unchanged) {
    if (forceSend) sendToArduino(zone);
    return;
  }

  state.currentZone = zone;
  state.targetPan = target.angle;

  // Send exact command understood by Arduino Uno firmware: LEFT, HOME, or RIGHT
  sendToArduino(zone);

  if (state.moveTimer) return;

  // Smoothly step visual 3D head at 1 deg / 15 ms (matches Arduino thirdSpeed = 15)
  state.moveTimer = setInterval(() => {
    if (state.pan < state.targetPan) {
      state.pan++;
    } else if (state.pan > state.targetPan) {
      state.pan--;
    } else {
      clearInterval(state.moveTimer);
      state.moveTimer = null;
    }
    update3DHeadAngles(state.pan, state.tilt);
  }, 15);
}

function moveTiltTo(tiltZone) {
  const target = TILT_TARGETS[tiltZone];
  if (!target) return;

  if (state.currentTiltZone === tiltZone && state.targetTilt === target.angle) return;

  state.currentTiltZone = tiltZone;
  state.targetTilt = target.angle;

  if (tiltZone === 'UP' || tiltZone === 'DOWN') {
    sendToArduino(tiltZone);
  }

  if (state.tiltTimer) return;

  state.tiltTimer = setInterval(() => {
    if (state.tilt < state.targetTilt) {
      state.tilt++;
    } else if (state.tilt > state.targetTilt) {
      state.tilt--;
    } else {
      clearInterval(state.tiltTimer);
      state.tiltTimer = null;
    }
    update3DHeadAngles(state.pan, state.tilt);
  }, 15);
}

/**
 * Stops all active robot head motion immediately (e.g. when hand leaves camera frame).
 */
function stopAllMovement() {
  if (state.moveTimer) {
    clearInterval(state.moveTimer);
    state.moveTimer = null;
  }
  if (state.tiltTimer) {
    clearInterval(state.tiltTimer);
    state.tiltTimer = null;
  }
  state.targetPan = state.pan;
  state.targetTilt = state.tilt;
  update3DHeadAngles(state.pan, state.tilt);
}

/**
 * Clears the established hand reference so next detection must re-establish at center.
 */
function resetHandReference() {
  state.hand.phase = 'SEARCHING';
  state.hand.refX = null;
  state.hand.refY = null;
  state.hand.smoothX = null;
  state.hand.smoothY = null;
  state.hand.prevRawX = null;
  state.hand.prevRawY = null;
  state.hand.centerStableFrames = 0;
  state.hand.pendingZone = null;
  state.hand.pendingFrames = 0;
  state.hand.pendingTiltZone = null;
  state.hand.pendingTiltFrames = 0;
  state.hand.hasTriggeredMove = false;
  state.hand.centerGestureFrames = 0;
  state.hand.pendingCenterGesture = null;
  state.hand.activeCenterGesture = null;
}

function centerHead() {
  resetHandReference();
  moveTo('HOME', { forceSend: true });
  if (state.hand.enableTilt) {
    moveTiltTo('CENTER');
  }
}

// ============================================================================
// Column 1: Three.js 3D Robot Head Visualizer
// ============================================================================
let scene, camera, renderer;
let robotHeadGroup, neckPivot;
let targetEulerYaw = 0;
let currentEulerYaw = 0;
let targetEulerPitch = 0;
let currentEulerPitch = 0;

function initThreeJS() {
  const container = dom.threeContainer;
  const width = container.clientWidth || 600;
  const height = container.clientHeight || 500;

  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x04060a, 0.035);

  camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  camera.position.set(0, 0.45, 4.4);
  camera.lookAt(0, 0.25, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  const ambientLight = new THREE.AmbientLight(0x667799, 1.4);
  scene.add(ambientLight);

  const keyLight = new THREE.DirectionalLight(0x00f0ff, 1.6);
  keyLight.position.set(3, 4, 3);
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0xff0055, 1.2);
  fillLight.position.set(-3, 2, -2);
  scene.add(fillLight);

  const gridHelper = new THREE.GridHelper(6, 20, 0x00f0ff, 0x152238);
  gridHelper.position.y = -1.25;
  scene.add(gridHelper);

  buildRobotModel();
  setupViewDragging(container);
  window.addEventListener('resize', onWindowResize);
  animate();
}

function buildRobotModel() {
  const pedestalGeo = new THREE.CylinderGeometry(0.85, 1.05, 0.1, 32);
  const pedestalMat = new THREE.MeshStandardMaterial({
    color: 0x0c1322,
    metalness: 0.85,
    roughness: 0.3
  });
  const pedestal = new THREE.Mesh(pedestalGeo, pedestalMat);
  pedestal.position.y = -1.2;
  scene.add(pedestal);

  const ringGeo = new THREE.TorusGeometry(0.95, 0.02, 16, 64);
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, transparent: true, opacity: 0.5 });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = Math.PI / 2;
  ring.position.y = -1.14;
  scene.add(ring);

  neckPivot = new THREE.Group();
  scene.add(neckPivot);

  robotHeadGroup = new THREE.Group();
  neckPivot.add(robotHeadGroup);

  const textureLoader = new THREE.TextureLoader();
  const faceTextureSrc =
    typeof HEAD_IMAGE_DATA !== 'undefined' && HEAD_IMAGE_DATA ? HEAD_IMAGE_DATA : 'head_mesh.png';
  const faceTexture = textureLoader.load(faceTextureSrc);
  faceTexture.minFilter = THREE.LinearFilter;
  faceTexture.magFilter = THREE.LinearFilter;

  const faceWidth = 2.05;
  const faceHeight = 2.68;
  const splitRatio = 0.1813;
  const jawHeight = faceHeight * splitRatio;
  const upperHeight = faceHeight * (1.0 - splitRatio);
  const halfW = faceWidth / 2;
  const totalHalfH = faceHeight / 2;
  const seamY = -0.6041;
  const upperCenterY = seamY + upperHeight / 2;
  const jawCenterY = -jawHeight / 2;

  const faceMat = new THREE.MeshBasicMaterial({
    map: faceTexture,
    transparent: true,
    alphaTest: 0.05,
    side: THREE.DoubleSide
  });

  // Upper Face
  const upperGeo = new THREE.PlaneGeometry(faceWidth, upperHeight, 48, 36);
  const upperPos = upperGeo.attributes.position;
  const upperUv = upperGeo.attributes.uv;

  for (let i = 0; i < upperPos.count; i++) {
    const x = upperPos.getX(i);
    const localY = upperPos.getY(i);
    const worldY = localY + upperCenterY;
    const normX = x / halfW;
    const normY = (worldY - 0.25) / totalHalfH;

    const archX = Math.cos(normX * (Math.PI * 0.48));
    const archY = Math.max(0.32, 1.0 - Math.pow(normY * 0.75, 2));
    upperPos.setZ(i, archX * 0.44 * archY);

    const v = upperUv.getY(i);
    upperUv.setY(i, splitRatio + v * (1.0 - splitRatio));
  }
  upperGeo.computeVertexNormals();
  const upperMesh = new THREE.Mesh(upperGeo, faceMat);
  upperMesh.position.set(0, upperCenterY, 0);
  robotHeadGroup.add(upperMesh);

  // Lower Jaw
  const jawPivot = new THREE.Group();
  jawPivot.position.set(0, seamY, 0);
  robotHeadGroup.add(jawPivot);

  const jawGeo = new THREE.PlaneGeometry(faceWidth, jawHeight, 48, 12);
  const jawPos = jawGeo.attributes.position;
  const jawUv = jawGeo.attributes.uv;

  for (let i = 0; i < jawPos.count; i++) {
    const x = jawPos.getX(i);
    const localY = jawPos.getY(i);
    const worldY = seamY + jawCenterY + localY;
    const normX = x / halfW;
    const normY = (worldY - 0.25) / totalHalfH;

    const archX = Math.cos(normX * (Math.PI * 0.48));
    const archY = Math.max(0.32, 1.0 - Math.pow(normY * 0.75, 2));
    jawPos.setZ(i, archX * 0.44 * archY);

    const v = jawUv.getY(i);
    jawUv.setY(i, v * splitRatio);
  }
  jawGeo.computeVertexNormals();
  const jawMesh = new THREE.Mesh(jawGeo, faceMat);
  jawMesh.position.set(0, jawCenterY, 0);
  jawPivot.add(jawMesh);

  const visorLight = new THREE.PointLight(0x00f0ff, 1.2, 2.5);
  visorLight.position.set(0, 0.35, 0.1);
  robotHeadGroup.add(visorLight);
}

function update3DHeadAngles(panAngle, tiltAngle = 90) {
  // 90° = CENTER (0 rad)
  // 50° = RIGHT -> -(50 - 90) = +40° rad -> nose points RIGHT (+X)
  // 140° = LEFT -> -(140 - 90) = -50° rad -> nose points LEFT (-X)
  targetEulerYaw = -((panAngle - 90) * (Math.PI / 180));
  targetEulerPitch = state.hand.enableTilt ? -((tiltAngle - 90) * (Math.PI / 180)) * 0.75 : 0;

  const relYaw = -(panAngle - 90);
  const dirTag = relYaw > 4 ? 'RIGHT' : relYaw < -4 ? 'LEFT' : 'CENTER';

  if (dom.hudYaw) dom.hudYaw.textContent = `${relYaw > 0 ? '+' : ''}${relYaw.toFixed(0)}° (${dirTag})`;
  if (dom.servoPanVal) dom.servoPanVal.textContent = `${panAngle}°`;
  if (dom.chipPanVal) dom.chipPanVal.textContent = `${panAngle}° (${dirTag})`;

  if (state.hand.enableTilt) {
    const relPitch = tiltAngle - 90;
    const tiltTag = relPitch > 4 ? 'UP' : relPitch < -4 ? 'DOWN' : 'CENTER';
    if (dom.hudPitch) dom.hudPitch.textContent = `${relPitch > 0 ? '+' : ''}${relPitch.toFixed(0)}° (${tiltTag})`;
    if (dom.servoTiltVal) dom.servoTiltVal.textContent = `${tiltAngle}°`;
    if (dom.chipTiltVal) dom.chipTiltVal.textContent = `${tiltAngle}° (${tiltTag})`;
  } else {
    if (dom.hudPitch) dom.hudPitch.textContent = 'LOCKED';
    if (dom.servoTiltVal) dom.servoTiltVal.textContent = '90° (LOCK)';
    if (dom.chipTiltVal) dom.chipTiltVal.textContent = 'LOCKED';
  }
}

function animate() {
  requestAnimationFrame(animate);
  currentEulerYaw += (targetEulerYaw - currentEulerYaw) * 0.25;
  currentEulerPitch += (targetEulerPitch - currentEulerPitch) * 0.25;
  if (neckPivot) neckPivot.rotation.y = currentEulerYaw;
  if (robotHeadGroup) robotHeadGroup.rotation.x = state.hand.enableTilt ? currentEulerPitch : 0;
  renderer.render(scene, camera);
}

function onWindowResize() {
  if (!renderer || !camera || !dom.threeContainer) return;
  const width = dom.threeContainer.clientWidth;
  const height = dom.threeContainer.clientHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

function setupViewDragging(container) {
  let isDragging = false;
  let prevMouseX = 0;
  let prevMouseY = 0;

  container.addEventListener('mousedown', (e) => {
    isDragging = true;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;
  });

  window.addEventListener('mouseup', () => {
    isDragging = false;
  });

  window.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    const deltaX = e.clientX - prevMouseX;
    const deltaY = e.clientY - prevMouseY;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;

    if (camera) {
      camera.position.x += deltaX * 0.005;
      camera.position.y -= deltaY * 0.005;
      camera.position.x = Math.max(-2.5, Math.min(2.5, camera.position.x));
      camera.position.y = Math.max(-1.0, Math.min(2.5, camera.position.y));
      camera.lookAt(0, 0.25, 0);
    }
  });
}

// ============================================================================
// Column 2: MediaPipe HandLandmarker + 3-Zone Hysteresis (LEFT / HOME / RIGHT)
// ============================================================================

async function initHandLandmarker() {
  if (state.hand.handLandmarker) return state.hand.handLandmarker;

  const { FilesetResolver, HandLandmarker } = await import(`${VISION_CDN}/vision_bundle.mjs`);
  const vision = await FilesetResolver.forVisionTasks(`${VISION_CDN}/wasm`);

  try {
    state.hand.handLandmarker = await HandLandmarker.createFromOptions(vision, {
      baseOptions: { modelAssetPath: MODEL_URL, delegate: 'GPU' },
      runningMode: 'VIDEO',
      numHands: 1
    });
  } catch {
    state.hand.handLandmarker = await HandLandmarker.createFromOptions(vision, {
      baseOptions: { modelAssetPath: MODEL_URL, delegate: 'CPU' },
      runningMode: 'VIDEO',
      numHands: 1
    });
  }

  state.hand.usingLandmarker = true;
  return state.hand.handLandmarker;
}

/**
 * Computes a stable palm center from 5 core palm landmarks (Wrist + 4 MCP knuckles)
 * in mirrored selfie coordinates (0 = left edge, 1 = right edge).
 */
function computePalmCenter(rawLandmarks) {
  const indices = [0, 5, 9, 13, 17];
  let sumX = 0;
  let sumY = 0;
  for (const idx of indices) {
    sumX += 1 - rawLandmarks[idx].x;
    sumY += rawLandmarks[idx].y;
  }
  return {
    x: sumX / indices.length,
    y: sumY / indices.length
  };
}

/**
 * Analyzes hand landmarks to detect:
 *   1. CLOSED HAND (Fist) -> all 4 fingers (index, middle, ring, pinky) folded
 *   2. SHOWS 1 -> index finger extended, while middle, ring, and pinky are folded
 * Both gestures trigger instant return to CENTER / HOME position (90°, 90°).
 */
function detectHandGesture(lm) {
  if (!lm || lm.length < 21) return { type: 'NONE', isCenterGesture: false, name: 'NO HAND' };

  const dist = (i, j) => Math.hypot(lm[i].x - lm[j].x, lm[i].y - lm[j].y);

  // A finger is extended if tip is further from wrist than PIP AND tip is higher than PIP (in upright selfie view)
  const indexExtended = (dist(8, 0) > dist(6, 0) * 1.10) && (lm[8].y < lm[6].y || dist(8, 0) > dist(6, 0) * 1.25);
  const middleExtended = (dist(12, 0) > dist(10, 0) * 1.10) && (lm[12].y < lm[10].y || dist(12, 0) > dist(10, 0) * 1.25);
  const ringExtended = (dist(16, 0) > dist(14, 0) * 1.10) && (lm[16].y < lm[14].y || dist(16, 0) > dist(14, 0) * 1.25);
  const pinkyExtended = (dist(20, 0) > dist(18, 0) * 1.10) && (lm[20].y < lm[18].y || dist(20, 0) > dist(18, 0) * 1.25);

  const isFist = !indexExtended && !middleExtended && !ringExtended && !pinkyExtended;
  const isOne = indexExtended && !middleExtended && !ringExtended && !pinkyExtended;

  if (isFist) {
    return { type: 'FIST', isCenterGesture: true, name: '✊ CLOSED HAND' };
  }
  if (isOne) {
    return { type: 'ONE', isCenterGesture: true, name: '☝️ SHOWING 1' };
  }

  return { type: 'OPEN', isCenterGesture: false, name: '🖐️ OPEN HAND' };
}

/**
 * Two-Phase Hand Detection & Intentional Movement State Machine:
 *   Phase 1 (Detection / Reference):
 *     - Hand must be detected and held steady inside the BIG center column (36%..64%)
 *       for `centerLockRequired` frames to establish (refX, refY).
 *     - ZERO robot movement occurs during detection or calibration.
 *   Phase 2 (Intentional Movement beyond Dead Zone):
 *     - Inside the dead zone (±12% X around refX, 24% total center column width):
 *       Robot stays completely stationary.
 *     - Beyond the dead zone for `intentFramesRequired` consecutive frames:
 *       Triggers smooth robot movement in that direction (Right -> LEFT, Left -> RIGHT).
 *   Gesture Override:
 *     - Closing hand (✊ Fist) OR showing 1 (☝️ Index up) overrides any zone
 *       and immediately returns the head to CENTER / HOME (90°, 90°).
 */
function evaluateHandTracking(rawX, rawY, gesture = null) {
  const h = state.hand;

  // Reject single-frame unstable detection jumps/teleports
  if (h.prevRawX !== null && h.prevRawY !== null) {
    const jump = Math.hypot(rawX - h.prevRawX, rawY - h.prevRawY);
    if (jump > h.maxJumpDelta) {
      h.prevRawX = rawX;
      h.prevRawY = rawY;
      return {
        phase: h.phase,
        panZone: null,
        tiltZone: null,
        smoothX: h.smoothX ?? rawX,
        smoothY: h.smoothY ?? rawY,
        lockProgress: h.centerStableFrames / h.centerLockRequired,
        gesture
      };
    }
  }
  h.prevRawX = rawX;
  h.prevRawY = rawY;

  // Low-pass EMA smoothing to remove minor tremor & landmark jitter
  if (h.smoothX === null || h.smoothY === null) {
    h.smoothX = rawX;
    h.smoothY = rawY;
  } else {
    h.smoothX += (rawX - h.smoothX) * h.emaAlpha;
    h.smoothY += (rawY - h.smoothY) * h.emaAlpha;
  }

  // -------------------------------------------------------------------------
  // GESTURE OVERRIDE: Closed Hand (Fist) or Showing 1 -> Return to Center / Home
  // -------------------------------------------------------------------------
  if (gesture && gesture.isCenterGesture) {
    if (h.pendingCenterGesture === gesture.type) {
      h.centerGestureFrames++;
    } else {
      h.pendingCenterGesture = gesture.type;
      h.centerGestureFrames = 1;
    }

    if (h.centerGestureFrames >= 2) {
      h.activeCenterGesture = gesture.type;
      h.phase = 'CENTER_GESTURE';
      h.pendingZone = null;
      h.pendingFrames = 0;
      h.pendingTiltZone = null;
      h.pendingTiltFrames = 0;
      h.hasTriggeredMove = false;
      // Anchor reference point so re-opening hand stays in dead zone
      h.refX = h.smoothX;
      h.refY = h.smoothY;

      return {
        phase: 'CENTER_GESTURE',
        panZone: 'HOME',
        tiltZone: state.hand.enableTilt ? 'CENTER' : null,
        smoothX: h.smoothX,
        smoothY: h.smoothY,
        lockProgress: 1,
        gesture
      };
    }
  } else {
    h.centerGestureFrames = 0;
    h.pendingCenterGesture = null;
    h.activeCenterGesture = null;
    if (h.phase === 'CENTER_GESTURE') {
      h.phase = 'READY';
    }
  }

  // -------------------------------------------------------------------------
  // PHASE 1: Establish hand reference at the center of the camera frame
  // (NEVER moves the robot!)
  // -------------------------------------------------------------------------
  if (h.refX === null || h.refY === null) {
    const inCenterZone = h.smoothX >= h.centerBoxMinX && h.smoothX <= h.centerBoxMaxX;

    if (!inCenterZone) {
      h.phase = 'WAIT_CENTER';
      h.centerStableFrames = 0;
      return {
        phase: 'WAIT_CENTER',
        panZone: null,
        tiltZone: null,
        smoothX: h.smoothX,
        smoothY: h.smoothY,
        lockProgress: 0
      };
    }

    h.phase = 'CALIBRATING';
    h.centerStableFrames++;

    if (h.centerStableFrames >= h.centerLockRequired) {
      // Lock neutral reference position — do NOT trigger any robot movement!
      h.refX = h.smoothX;
      h.refY = h.smoothY;
      h.phase = 'READY';
      h.pendingZone = null;
      h.pendingFrames = 0;
      h.pendingTiltZone = null;
      h.pendingTiltFrames = 0;
      h.hasTriggeredMove = false;
    }

    return {
      phase: h.phase,
      panZone: null,
      tiltZone: null,
      smoothX: h.smoothX,
      smoothY: h.smoothY,
      lockProgress: Math.min(1, h.centerStableFrames / h.centerLockRequired)
    };
  }

  // -------------------------------------------------------------------------
  // PHASE 2: Reference locked — only move on intentional motion beyond dead zone
  // -------------------------------------------------------------------------
  const dx = h.smoothX - h.refX;
  const dy = h.smoothY - h.refY;

  // Default (mirrorX = false): Hand Right (+dx) -> Head LEFT, Hand Left (-dx) -> Head RIGHT
  const rightMoveZone = h.mirrorX ? 'RIGHT' : 'LEFT';
  const leftMoveZone = h.mirrorX ? 'LEFT' : 'RIGHT';

  let candidatePan = null;
  if (dx >= h.deadZoneX) {
    candidatePan = rightMoveZone;
  } else if (dx <= -h.deadZoneX) {
    candidatePan = leftMoveZone;
  } else if (Math.abs(dx) <= h.returnBandX) {
    // Inside dead zone: stay completely stationary if we haven't moved yet;
    // return to HOME only if user previously moved out and intentionally returned to center.
    if (h.hasTriggeredMove && state.currentZone !== 'HOME') {
      candidatePan = 'HOME';
    } else {
      h.pendingZone = null;
      h.pendingFrames = 0;
      if (!h.hasTriggeredMove || state.currentZone === 'HOME') {
        h.phase = 'READY';
      }
    }
  } else {
    // Hysteresis ring between returnBandX and deadZoneX: hold current state stably
    h.pendingZone = null;
    h.pendingFrames = 0;
  }

  let confirmedPan = null;
  if (candidatePan !== null && candidatePan !== state.currentZone) {
    if (h.pendingZone === candidatePan) {
      h.pendingFrames++;
    } else {
      h.pendingZone = candidatePan;
      h.pendingFrames = 1;
    }
    if (h.pendingFrames >= h.intentFramesRequired) {
      confirmedPan = candidatePan;
      h.hasTriggeredMove = true;
      h.phase = candidatePan === 'HOME' ? 'READY' : 'MOVING';
      h.pendingZone = null;
      h.pendingFrames = 0;
    }
  } else if (candidatePan === state.currentZone && candidatePan !== null) {
    h.pendingZone = null;
    h.pendingFrames = 0;
    h.phase = candidatePan === 'HOME' ? 'READY' : 'MOVING';
  }

  // Optional vertical Up/Down tilt when explicitly enabled
  let confirmedTilt = null;
  if (h.enableTilt) {
    let candidateTilt = null;
    if (dy <= -h.deadZoneY) candidateTilt = 'UP';
    else if (dy >= h.deadZoneY) candidateTilt = 'DOWN';
    else if (Math.abs(dy) <= h.returnBandY && state.currentTiltZone !== 'CENTER') {
      candidateTilt = 'CENTER';
    }

    if (candidateTilt !== null && candidateTilt !== state.currentTiltZone) {
      if (h.pendingTiltZone === candidateTilt) {
        h.pendingTiltFrames++;
      } else {
        h.pendingTiltZone = candidateTilt;
        h.pendingTiltFrames = 1;
      }
      if (h.pendingTiltFrames >= h.intentFramesRequired) {
        confirmedTilt = candidateTilt;
        h.hasTriggeredMove = true;
        h.pendingTiltZone = null;
        h.pendingTiltFrames = 0;
      }
    } else {
      h.pendingTiltZone = null;
      h.pendingTiltFrames = 0;
    }
  }

  return {
    phase: h.phase,
    panZone: confirmedPan,
    tiltZone: confirmedTilt,
    smoothX: h.smoothX,
    smoothY: h.smoothY,
    lockProgress: 1
  };
}

function drawZoneOverlay(ctx, w, h, handPoint, mirroredLandmarks, activeZone, phase = 'SEARCHING', lockProgress = 0, outcomeGesture = null) {
  // BIG Center Column (24% wide — identical to original 0.38..0.62 column!)
  const refCenter = state.hand.refX !== null ? state.hand.refX : 0.5;
  const dz = state.hand.deadZoneX; // 0.12 -> total center column width = 0.24 (24%)
  const leftLine = Math.max(40, Math.min(w * 0.44, (refCenter - dz) * w));
  const rightLine = Math.max(w * 0.56, Math.min(w - 40, (refCenter + dz) * w));
  const leftBoxZone = state.hand.mirrorX ? 'LEFT' : 'RIGHT';
  const rightBoxZone = state.hand.mirrorX ? 'RIGHT' : 'LEFT';

  // Highlight appropriate column
  if (phase === 'CENTER_GESTURE') {
    // Glowing emerald green highlight over center column
    ctx.fillStyle = 'rgba(0, 255, 136, 0.22)';
    ctx.fillRect(leftLine, 0, rightLine - leftLine, h);
  } else if (phase === 'WAIT_CENTER' || phase === 'CALIBRATING') {
    // Highlight the BIG center column to invite user to place hand in center
    ctx.fillStyle = phase === 'CALIBRATING' ? 'rgba(0, 255, 136, 0.13)' : 'rgba(255, 184, 0, 0.13)';
    ctx.fillRect(leftLine, 0, rightLine - leftLine, h);
  } else if (handPoint && phase === 'MOVING' && activeZone) {
    ctx.fillStyle = 'rgba(0, 240, 255, 0.15)';
    if (activeZone === leftBoxZone) ctx.fillRect(0, 0, leftLine, h);
    else if (activeZone === 'HOME') ctx.fillRect(leftLine, 0, rightLine - leftLine, h);
    else if (activeZone === rightBoxZone) ctx.fillRect(rightLine, 0, w - rightLine, h);
  } else if (handPoint && phase === 'READY') {
    // Subtle neutral glow in center dead zone (robot completely stationary)
    ctx.fillStyle = 'rgba(0, 240, 255, 0.08)';
    ctx.fillRect(leftLine, 0, rightLine - leftLine, h);
  }

  // Big Center Column divider dashed lines
  ctx.save();
  ctx.strokeStyle =
    phase === 'CENTER_GESTURE'
      ? 'rgba(0, 255, 136, 0.85)'
      : phase === 'WAIT_CENTER'
        ? 'rgba(255, 184, 0, 0.65)'
        : phase === 'CALIBRATING'
          ? 'rgba(0, 255, 136, 0.65)'
          : 'rgba(0, 240, 255, 0.48)';
  ctx.lineWidth = phase === 'CENTER_GESTURE' ? 2.5 : 2;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  ctx.moveTo(leftLine, 0);
  ctx.lineTo(leftLine, h);
  ctx.moveTo(rightLine, 0);
  ctx.lineTo(rightLine, h);
  ctx.stroke();

  // Draw Reference Point crosshair when neutral reference is established
  if (state.hand.refX !== null && state.hand.refY !== null) {
    const rx = state.hand.refX * w;
    const ry = state.hand.refY * h;
    ctx.setLineDash([]);
    ctx.strokeStyle = phase === 'CENTER_GESTURE' ? '#00ff88' : 'rgba(0, 240, 255, 0.55)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(rx, ry, 10, 0, Math.PI * 2);
    ctx.moveTo(rx - 16, ry);
    ctx.lineTo(rx + 16, ry);
    ctx.moveTo(rx, ry - 16);
    ctx.lineTo(rx, ry + 16);
    ctx.stroke();
  }
  ctx.setLineDash([]);

  // Zone headers
  ctx.font = '700 13px Orbitron, sans-serif';
  ctx.textAlign = 'center';
  const isMoving = handPoint && phase === 'MOVING';
  ctx.fillStyle = isMoving && activeZone === leftBoxZone ? '#00ff88' : 'rgba(226, 232, 240, 0.72)';
  ctx.fillText(leftBoxZone === 'RIGHT' ? 'HEAD → RIGHT' : '← HEAD LEFT', leftLine / 2, 28);

  const centerTitle =
    phase === 'CENTER_GESTURE' && outcomeGesture
      ? `${outcomeGesture.name} → CENTER`
      : phase === 'WAIT_CENTER'
        ? 'PLACE HAND HERE'
        : phase === 'CALIBRATING'
          ? 'LOCKING CENTER...'
          : phase === 'READY'
            ? 'CENTER (DEAD ZONE)'
            : 'CENTER (HOME)';
  ctx.fillStyle =
    phase === 'CENTER_GESTURE'
      ? '#00ff88'
      : phase === 'WAIT_CENTER'
        ? '#ffb800'
        : phase === 'CALIBRATING' || phase === 'READY'
          ? '#00f0ff'
          : 'rgba(226, 232, 240, 0.78)';
  ctx.fillText(centerTitle, (leftLine + rightLine) / 2, 28);

  ctx.fillStyle = isMoving && activeZone === rightBoxZone ? '#00ff88' : 'rgba(226, 232, 240, 0.72)';
  ctx.fillText(rightBoxZone === 'LEFT' ? 'HEAD → LEFT' : 'HEAD RIGHT →', rightLine + (w - rightLine) / 2, 28);
  ctx.restore();

  // Draw hand skeleton if landmarks are available
  if (mirroredLandmarks) {
    drawCyberHandSkeleton(ctx, mirroredLandmarks, w, h, phase === 'CENTER_GESTURE');
  }

  // Tracked palm marker
  if (handPoint) {
    const px = handPoint.x * w;
    const py = handPoint.y * h;
    const accent =
      phase === 'CENTER_GESTURE'
        ? '#00ff88'
        : phase === 'WAIT_CENTER'
          ? '#ffb800'
          : phase === 'CALIBRATING'
            ? '#00f0ff'
            : phase === 'READY'
              ? '#00f0ff'
              : '#00ff88';

    ctx.save();
    ctx.beginPath();
    ctx.arc(px, py, 18, 0, Math.PI * 2);
    ctx.fillStyle =
      phase === 'CENTER_GESTURE'
        ? 'rgba(0, 255, 136, 0.28)'
        : phase === 'WAIT_CENTER'
          ? 'rgba(255, 184, 0, 0.22)'
          : phase === 'MOVING'
            ? 'rgba(0, 255, 136, 0.22)'
            : 'rgba(0, 240, 255, 0.22)';
    ctx.fill();
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = accent;
    ctx.shadowColor = accent;
    ctx.shadowBlur = 10;
    ctx.stroke();

    // Calibration progress arc while locking center reference
    if (phase === 'CALIBRATING' && lockProgress > 0) {
      ctx.beginPath();
      ctx.arc(px, py, 24, -Math.PI / 2, -Math.PI / 2 + Math.PI * 2 * lockProgress);
      ctx.strokeStyle = '#00ff88';
      ctx.lineWidth = 3;
      ctx.stroke();
    }

    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    ctx.font = '700 11px JetBrains Mono, monospace';
    ctx.fillStyle = accent;
    let markerText = 'HOLD IN DEAD ZONE';
    if (phase === 'CENTER_GESTURE') markerText = `${outcomeGesture ? outcomeGesture.name : 'GESTURE'} → HOME (90°)`;
    else if (phase === 'WAIT_CENTER') markerText = 'MOVE TO CENTER FIRST';
    else if (phase === 'CALIBRATING') markerText = `SETTING REF ${Math.round(lockProgress * 100)}%`;
    else if (phase === 'READY') markerText = 'REF LOCKED (STATIONARY)';
    else if (phase === 'MOVING') markerText = `${TARGETS[activeZone].name.toUpperCase()} (${TARGETS[activeZone].angle}°)`;

    ctx.fillText(markerText, px + 26, py + 4);
    ctx.restore();
  }
}

function drawCyberHandSkeleton(ctx, lm, w, h, isCenterGesture = false) {
  const connections = [
    [0, 1], [1, 2], [2, 3], [3, 4],
    [0, 5], [5, 6], [6, 7], [7, 8],
    [5, 9], [9, 10], [10, 11], [11, 12],
    [9, 13], [13, 14], [14, 15], [15, 16],
    [13, 17], [17, 18], [18, 19], [19, 20],
    [0, 17]
  ];

  const strokeColor = isCenterGesture ? '#00ff88' : '#00f0ff';
  ctx.save();
  ctx.lineWidth = isCenterGesture ? 2.5 : 2;
  ctx.strokeStyle = strokeColor;
  ctx.shadowColor = strokeColor;
  ctx.shadowBlur = isCenterGesture ? 10 : 6;

  connections.forEach(([i, j]) => {
    ctx.beginPath();
    ctx.moveTo(lm[i].x * w, lm[i].y * h);
    ctx.lineTo(lm[j].x * w, lm[j].y * h);
    ctx.stroke();
  });

  ctx.shadowBlur = 0;
  lm.forEach((pt, i) => {
    ctx.beginPath();
    ctx.arc(pt.x * w, pt.y * h, i === 4 || i === 8 || i === 12 || i === 16 || i === 20 ? 4 : 2.5, 0, Math.PI * 2);
    ctx.fillStyle = isCenterGesture ? '#ffffff' : (i === 4 || i === 8 || i === 12 || i === 16 || i === 20 ? '#ff0055' : '#00f0ff');
    ctx.fill();
  });
  ctx.restore();
}

function applyTrackingOutcome(ctx, w, h, outcome, mirroredLandmarks) {
  // Trigger robot motor movement ONLY on confirmed intentional movement beyond dead zone
  if (outcome.panZone) {
    moveTo(outcome.panZone);
  }
  if (outcome.tiltZone && state.hand.enableTilt) {
    moveTiltTo(outcome.tiltZone);
  } else if (outcome.phase === 'CENTER_GESTURE' && state.tilt !== 90) {
    moveTiltTo('CENTER');
  }

  const displayZone = outcome.panZone || state.currentZone;
  drawZoneOverlay(
    ctx,
    w,
    h,
    { x: outcome.smoothX, y: outcome.smoothY },
    mirroredLandmarks,
    displayZone,
    outcome.phase,
    outcome.lockProgress,
    outcome.gesture
  );

  if (outcome.phase === 'CENTER_GESTURE' && outcome.gesture) {
    updateTrackingBadge(`${outcome.gesture.name} // HOME`, 'tracking');
    if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${outcome.smoothX.toFixed(2)} (CENTER / HOME)`;
    if (dom.handValGesture) dom.handValGesture.textContent = `${outcome.gesture.name} → CENTER`;
    if (dom.chipTrackState) dom.chipTrackState.textContent = 'CENTER (HOME)';
  } else if (outcome.phase === 'WAIT_CENTER') {
    updateTrackingBadge('MOVE TO CENTER', 'searching');
    if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${outcome.smoothX.toFixed(2)} (OFF CENTER)`;
    if (dom.handValGesture) dom.handValGesture.textContent = 'PLACE HAND IN CENTER';
    if (dom.chipTrackState) dom.chipTrackState.textContent = 'SET CENTER REF';
  } else if (outcome.phase === 'CALIBRATING') {
    const pct = Math.round(outcome.lockProgress * 100);
    updateTrackingBadge(`LOCKING REF ${pct}%`, 'searching');
    if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${outcome.smoothX.toFixed(2)} (CENTER)`;
    if (dom.handValGesture) dom.handValGesture.textContent = `ESTABLISHING REF ${pct}%`;
    if (dom.chipTrackState) dom.chipTrackState.textContent = 'CALIBRATING REF';
  } else if (outcome.phase === 'READY') {
    updateTrackingBadge('READY // DEAD ZONE', 'searching');
    if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${outcome.smoothX.toFixed(2)} (DEAD ZONE)`;
    if (dom.handValGesture) dom.handValGesture.textContent = 'STATIONARY (MOVE HAND)';
    if (dom.chipTrackState) dom.chipTrackState.textContent = 'STATIONARY (DEAD ZONE)';
  } else {
    const zoneLabel = TARGETS[displayZone].name.toUpperCase();
    updateTrackingBadge(`MOVING: ${zoneLabel}`, 'tracking');
    if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${outcome.smoothX.toFixed(2)} (${zoneLabel})`;
    if (dom.handValGesture) dom.handValGesture.textContent = `MOVED → ${zoneLabel}`;
    if (dom.chipTrackState) dom.chipTrackState.textContent = `FOLLOWING → ${zoneLabel}`;
  }
}

function handleHandLost(ctx, w, h, now) {
  state.hand.hasHand = false;
  // Brief 200ms tolerance for 1-frame flicker; once hand disappears, STOP all robot movement immediately!
  if (state.hand.phase !== 'SEARCHING' && now - state.hand.lastHandSeenAt > 200) {
    stopAllMovement();
    resetHandReference();
  }
  if (dom.cameraGuide) dom.cameraGuide.classList.remove('hidden');
  drawZoneOverlay(ctx, w, h, null, null, state.currentZone, 'SEARCHING', 0);
  updateTrackingBadge('HAND LOST // STOPPED', 'searching');
  if (dom.handValGesture) dom.handValGesture.textContent = 'STOPPED (NO HAND)';
  if (dom.chipTrackState) dom.chipTrackState.textContent = 'STOPPED (NO HAND)';
}

function trackFrame() {
  if (!state.hand.active) return;

  const video = dom.handWebcam;
  const canvas = dom.handCanvas;
  if (!video || !canvas) return;
  const ctx = canvas.getContext('2d');

  if (video.readyState >= 2 && video.currentTime !== state.hand.lastVideoTime) {
    state.hand.lastVideoTime = video.currentTime;

    if (video.videoWidth > 0 && (canvas.width !== video.videoWidth || canvas.height !== video.videoHeight)) {
      canvas.width = video.videoWidth;
      canvas.height = video.videoHeight;
    }

    const w = canvas.width;
    const h = canvas.height;

    // Draw mirrored selfie video
    ctx.save();
    ctx.clearRect(0, 0, w, h);
    ctx.translate(w, 0);
    ctx.scale(-1, 1);
    ctx.drawImage(video, 0, 0, w, h);
    ctx.restore();

    // Subtle sci-fi tint
    ctx.fillStyle = 'rgba(7, 10, 18, 0.22)';
    ctx.fillRect(0, 0, w, h);

    if (state.hand.handLandmarker) {
      const now = performance.now();
      const ts = now <= state.hand.lastTimestamp ? state.hand.lastTimestamp + 1 : now;
      state.hand.lastTimestamp = ts;

      const result = state.hand.handLandmarker.detectForVideo(video, ts);

      if (result.landmarks && result.landmarks.length > 0) {
        state.hand.hasHand = true;
        state.hand.lastHandSeenAt = now;
        if (dom.cameraGuide) dom.cameraGuide.classList.add('hidden');

        const lm = result.landmarks[0];
        const mirroredLandmarks = lm.map((pt) => ({
          x: 1 - pt.x,
          y: pt.y,
          z: pt.z
        }));

        // Compute stable 5-point palm center (mirrored coordinates)
        const palmCenter = computePalmCenter(lm);
        const gesture = detectHandGesture(lm);
        const outcome = evaluateHandTracking(palmCenter.x, palmCenter.y, gesture);
        applyTrackingOutcome(ctx, w, h, outcome, mirroredLandmarks);
      } else {
        handleHandLost(ctx, w, h, now);
      }
    } else {
      // Offline motion fallback if MediaPipe CDN couldn't load
      runMotionFallbackFrame(ctx, video, w, h);
    }
  }

  state.hand.rafId = requestAnimationFrame(trackFrame);
}

function runMotionFallbackFrame(ctx, video, w, h) {
  if (!state.hand.offCanvas) {
    state.hand.offCanvas = document.createElement('canvas');
    state.hand.offCanvas.width = 80;
    state.hand.offCanvas.height = 60;
    state.hand.offCtx = state.hand.offCanvas.getContext('2d', { willReadFrequently: true });
  }

  const offW = 80;
  const offH = 60;
  state.hand.offCtx.drawImage(video, 0, 0, offW, offH);
  const curImgData = state.hand.offCtx.getImageData(0, 0, offW, offH);
  const curPixels = curImgData.data;

  if (state.hand.prevFrameData) {
    let sumX = 0;
    let sumY = 0;
    let totalDiff = 0;
    const prevPixels = state.hand.prevFrameData.data;

    for (let y = 0; y < offH; y++) {
      for (let x = 0; x < offW; x++) {
        const idx = (y * offW + x) * 4;
        const diff =
          (Math.abs(curPixels[idx] - prevPixels[idx]) +
            Math.abs(curPixels[idx + 1] - prevPixels[idx + 1]) +
            Math.abs(curPixels[idx + 2] - prevPixels[idx + 2])) /
          3;
        if (diff > 25) {
          const mx = offW - 1 - x;
          sumX += mx * diff;
          sumY += y * diff;
          totalDiff += diff;
        }
      }
    }

    const now = performance.now();
    if (totalDiff > 1200) {
      const visualX = sumX / totalDiff / offW;
      const visualY = sumY / totalDiff / offH;
      state.hand.hasHand = true;
      state.hand.lastHandSeenAt = now;
      if (dom.cameraGuide) dom.cameraGuide.classList.add('hidden');

      const outcome = evaluateHandTracking(visualX, visualY);
      applyTrackingOutcome(ctx, w, h, outcome, null);
    } else {
      handleHandLost(ctx, w, h, now);
    }
  }

  state.hand.prevFrameData = curImgData;
}

// ============================================================================
// Primary / Secondary Camera Manager
// - PRIMARY   : External USB Web Camera (always active when connected)
// - SECONDARY : Built-in Laptop Camera (enabled ONLY when Web Camera is NOT connected)
// ============================================================================

const PRIMARY_CAM_STORAGE_KEY = 'robohead_primary_webcam_label';

const BUILTIN_CAM_PATTERNS = [
  'integrated', 'built-in', 'builtin', 'internal', 'laptop',
  'facetime', 'truevision', 'user facing', 'hd user',
  'acer hd', 'acer crystal', 'crystal eye', 'surface camera',
  'ov56', 'ov27', 'ov97'
];

const VIRTUAL_OR_IR_PATTERNS = [
  'ir camera', 'windows hello', 'infrared', 'depth',
  'obs virtual', 'virtual camera', 'snap camera', 'manycam'
];

const EXTERNAL_WEBCAM_PATTERNS = [
  'usb', 'uvc', 'webcam', 'web cam', 'pc camera', 'external',
  'logitech', 'c270', 'c310', 'c920', 'c922', 'brio',
  'zebronics', 'quantum', 'iball', 'intex', 'rapoo', 'anker',
  'razer', 'elgato', 'lifecam', 'creative', 'arducam',
  'hd camera', 'fhd', '1080p', '720p', '2.0', '3.0', 'general', 'sonix', 'microdia'
];

function isVirtualOrIrCamera(label) {
  const lower = (label || '').toLowerCase();
  if (!lower) return false;
  return VIRTUAL_OR_IR_PATTERNS.some((kw) => lower.includes(kw));
}

function isBuiltInLaptopCamera(label) {
  const lower = (label || '').toLowerCase();
  if (!lower) return false;
  return BUILTIN_CAM_PATTERNS.some((kw) => lower.includes(kw));
}

function isExternalWebCamera(label) {
  const lower = (label || '').toLowerCase();
  if (!lower) return false;
  if (isVirtualOrIrCamera(lower) || isBuiltInLaptopCamera(lower)) return false;
  return EXTERNAL_WEBCAM_PATTERNS.some((kw) => lower.includes(kw));
}

/**
 * Splits available cameras into Primary (External Web Camera) and Secondary (Laptop Camera).
 * Rule:
 *   1. If an external Web Camera is connected -> Web Camera is PRIMARY, Laptop Camera is DISABLED.
 *   2. If NO external Web Camera is connected -> Laptop Camera (SECONDARY) is enabled.
 */
function chooseBestCamera(videoDevices) {
  if (!videoDevices || videoDevices.length === 0) {
    return { device: null, isPrimaryWebCam: false, hasWebCam: false, builtInIds: new Set() };
  }

  // Exclude IR / virtual cameras unless they are the only devices available
  const realCams = videoDevices.filter((d) => !isVirtualOrIrCamera(d.label));
  const candidates = realCams.length > 0 ? realCams : videoDevices;

  // Only 1 real camera on the machine
  if (candidates.length === 1) {
    const onlyCam = candidates[0];
    const isExt = isExternalWebCamera(onlyCam.label);
    return {
      device: onlyCam,
      isPrimaryWebCam: isExt,
      hasWebCam: isExt,
      builtInIds: isExt ? new Set() : new Set([onlyCam.deviceId])
    };
  }

  // 2+ real cameras detected -> An external Web Camera is connected!
  // Identify built-in laptop camera(s) so they are strictly disabled.
  let builtInCams = candidates.filter((d) => isBuiltInLaptopCamera(d.label));
  let externalCams = candidates.filter((d) => !isBuiltInLaptopCamera(d.label));

  // If no label matched built-in patterns (generic OS names), Windows puts built-in laptop cam at index 0
  if (builtInCams.length === 0 && externalCams.length >= 2) {
    const explicitExt = externalCams.filter((d) => isExternalWebCamera(d.label));
    if (explicitExt.length > 0 && explicitExt.length < externalCams.length) {
      builtInCams = externalCams.filter((d) => !explicitExt.includes(d));
      externalCams = explicitExt;
    } else {
      builtInCams = [candidates[0]];
      externalCams = candidates.slice(1);
    }
  } else if (externalCams.length === 0) {
    // Edge case: all labels matched built-in keywords — treat last plugged camera as primary external
    builtInCams = candidates.slice(0, -1);
    externalCams = [candidates[candidates.length - 1]];
  }

  const builtInIds = new Set(builtInCams.map((d) => d.deviceId));

  // 1. Honor manually selected external web camera if still connected
  if (state.camera.preferredExternalId) {
    const preferred = externalCams.find((d) => d.deviceId === state.camera.preferredExternalId);
    if (preferred) {
      return { device: preferred, isPrimaryWebCam: true, hasWebCam: true, builtInIds };
    }
  }

  // 2. Honor saved primary web camera label from localStorage if connected
  try {
    const savedLabel = localStorage.getItem(PRIMARY_CAM_STORAGE_KEY);
    if (savedLabel) {
      const savedCam = externalCams.find((d) => d.label === savedLabel);
      if (savedCam) {
        return { device: savedCam, isPrimaryWebCam: true, hasWebCam: true, builtInIds };
      }
    }
  } catch (e) {}

  // 3. Prefer explicitly recognized USB/UVC external web cameras first
  const explicitWebCam = externalCams.find((d) => isExternalWebCamera(d.label));
  const chosen = explicitWebCam || externalCams[0];
  return { device: chosen, isPrimaryWebCam: true, hasWebCam: true, builtInIds };
}

function updateCameraSourceUI(videoDevices, selectedDevice, isPrimaryWebCam, hasWebCam, builtInIds = new Set()) {
  if (dom.camSourceBadge) {
    if (!selectedDevice) {
      dom.camSourceBadge.textContent = '❌ NO CAMERA';
      dom.camSourceBadge.className = 'studio-pill cam-pill';
      dom.camSourceBadge.title = 'No camera detected';
    } else if (isPrimaryWebCam) {
      dom.camSourceBadge.textContent = '🎥 WEB CAM (PRIMARY)';
      dom.camSourceBadge.className = 'studio-pill cam-pill robot-cam';
      dom.camSourceBadge.title = `Primary Web Camera: ${selectedDevice.label || 'USB Web Camera'} (Secondary Laptop Camera disabled)`;
    } else {
      dom.camSourceBadge.textContent = '💻 LAPTOP CAM (SECONDARY)';
      dom.camSourceBadge.className = 'studio-pill cam-pill laptop-cam';
      dom.camSourceBadge.title = `Web Camera not connected — using Secondary Laptop Camera (${selectedDevice.label || 'Built-in'})`;
    }
  }

  if (dom.selCameraSource) {
    dom.selCameraSource.innerHTML = '';
    if (!videoDevices || videoDevices.length === 0) {
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = 'No cameras detected';
      dom.selCameraSource.appendChild(opt);
      return;
    }

    videoDevices.forEach((dev, idx) => {
      const opt = document.createElement('option');
      opt.value = dev.deviceId;
      const rawName = dev.label || `Camera ${idx + 1}`;
      const isSecondaryLaptop = builtInIds.has(dev.deviceId) || isBuiltInLaptopCamera(rawName);

      if (hasWebCam && isSecondaryLaptop) {
        opt.disabled = true;
        opt.textContent = `🚫 ${rawName} (Secondary - Disabled)`;
      } else if (hasWebCam && !isVirtualOrIrCamera(rawName)) {
        opt.textContent = `🎥 [PRIMARY] ${rawName}`;
      } else {
        opt.textContent = `💻 [SECONDARY] ${rawName}`;
      }

      if (selectedDevice && dev.deviceId === selectedDevice.deviceId) {
        opt.selected = true;
      }
      dom.selCameraSource.appendChild(opt);
    });
  }
}

function stopCameraStreamOnly() {
  if (state.hand.stream) {
    state.hand.stream.getTracks().forEach((t) => {
      t.onended = null;
      t.stop();
    });
    state.hand.stream = null;
  }
  if (dom.handWebcam) {
    dom.handWebcam.srcObject = null;
  }
}

async function openCameraStreamForDevice(targetDevice, retryAttempt = 0) {
  // Always stop previous stream tracks first so the disabled camera's LED turns OFF immediately
  stopCameraStreamOnly();

  const videoConstraints =
    targetDevice && targetDevice.deviceId
      ? { deviceId: { exact: targetDevice.deviceId }, width: { ideal: 640 }, height: { ideal: 480 } }
      : { width: { ideal: 640 }, height: { ideal: 480 } };

  let stream;
  try {
    stream = await navigator.mediaDevices.getUserMedia({ video: videoConstraints, audio: false });
  } catch (err) {
    // Retry once/twice if a newly plugged-in USB web camera driver is still initializing
    if (
      retryAttempt < 2 &&
      (err.name === 'NotReadableError' || err.name === 'AbortError' || err.name === 'OverconstrainedError')
    ) {
      await new Promise((r) => setTimeout(r, 550));
      if (targetDevice && targetDevice.deviceId && err.name === 'OverconstrainedError') {
        stream = await navigator.mediaDevices.getUserMedia({
          video: { deviceId: { exact: targetDevice.deviceId } },
          audio: false
        });
      } else {
        return openCameraStreamForDevice(targetDevice, retryAttempt + 1);
      }
    } else if (targetDevice && targetDevice.deviceId) {
      stream = await navigator.mediaDevices.getUserMedia({
        video: { deviceId: { exact: targetDevice.deviceId } },
        audio: false
      });
    } else {
      throw err;
    }
  }

  state.hand.stream = stream;
  state.hand.lastVideoTime = -1;

  const activeTrack = stream.getVideoTracks()[0];
  const settings = activeTrack ? activeTrack.getSettings() : {};
  state.camera.activeDeviceId = settings.deviceId || (targetDevice ? targetDevice.deviceId : null);
  state.camera.activeLabel = (activeTrack && activeTrack.label) || (targetDevice ? targetDevice.label : '');

  // If the active primary web camera is unplugged, immediately fall back to secondary laptop camera
  if (activeTrack) {
    activeTrack.onended = () => {
      console.log('[CAMERA] Active camera disconnected — switching to available camera...');
      state.camera.activeDeviceId = null;
      if (state.hand.active && !state.camera.isSwitching) {
        setTimeout(() => {
          refreshAndSwitchCamera(true).catch((e) => console.warn('[CAMERA FALLBACK]', e));
        }, 300);
      }
    };
  }

  if (dom.handWebcam) {
    dom.handWebcam.srcObject = stream;
    await dom.handWebcam.play();
  }

  return stream;
}

async function refreshAndSwitchCamera(forceRescan = false) {
  if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) return;
  if (state.camera.isSwitching) return;
  state.camera.isSwitching = true;

  try {
    let devices = await navigator.mediaDevices.enumerateDevices();
    let videoDevices = devices.filter((d) => d.kind === 'videoinput');

    // If labels are hidden (first load before camera permission), request permission once
    const labelsMissing = videoDevices.length > 0 && videoDevices.every((d) => !d.label);
    if (labelsMissing || !state.hand.stream) {
      const initialChoice = chooseBestCamera(videoDevices);
      await openCameraStreamForDevice(initialChoice.device);

      // Re-enumerate now that permission is granted and exact device labels are visible
      devices = await navigator.mediaDevices.enumerateDevices();
      videoDevices = devices.filter((d) => d.kind === 'videoinput');
    }

    const best = chooseBestCamera(videoDevices);
    state.camera.isRobotCam = best.isPrimaryWebCam;
    state.camera.hasExternalCam = best.hasWebCam;

    const needsSwitch =
      best.device &&
      (best.device.deviceId !== state.camera.activeDeviceId || !state.hand.stream || forceRescan);

    if (needsSwitch && best.device) {
      if (best.device.deviceId !== state.camera.activeDeviceId || !state.hand.stream) {
        console.log(
          `[CAMERA] Activating ${best.isPrimaryWebCam ? 'PRIMARY WEB CAMERA' : 'SECONDARY LAPTOP CAMERA'}: ${best.device.label || best.device.deviceId}`
        );
        await openCameraStreamForDevice(best.device);
      }
    }

    updateCameraSourceUI(videoDevices, best.device, best.isPrimaryWebCam, best.hasWebCam, best.builtInIds);
  } catch (err) {
    console.error('[CAMERA SWITCH ERROR]', err);
    throw err;
  } finally {
    state.camera.isSwitching = false;
  }
}

async function startHandTracking() {
  if (state.hand.active) return;

  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    if (dom.guideText) dom.guideText.textContent = 'WEBCAM API UNAVAILABLE (USE CHROME/EDGE)';
    return;
  }

  updateTrackingBadge('LOADING MODEL...', 'searching');
  if (dom.guideText) dom.guideText.textContent = 'LOADING HAND TRACKING MODEL...';

  // Load MediaPipe HandLandmarker (same working engine as head-movement-sample)
  try {
    await initHandLandmarker();
  } catch (err) {
    console.warn('[HAND TRACK] HandLandmarker CDN unavailable, using offline motion fallback:', err);
  }

  try {
    if (dom.guideText) dom.guideText.textContent = 'DETECTING ROBOT USB CAMERA...';
    await refreshAndSwitchCamera(true);

    state.hand.active = true;
    state.hand.lastVideoTime = -1;

    if (dom.guideText) dom.guideText.textContent = 'PLACE HAND IN CENTER | ✊ CLOSE HAND OR ☝️ SHOW "1" TO CENTER';
    updateTrackingBadge('SEARCHING HAND', 'searching');
    state.hand.rafId = requestAnimationFrame(trackFrame);
  } catch (err) {
    console.error('[CAMERA ERROR]', err);
    updateTrackingBadge('CAMERA DENIED', 'locked');
    if (dom.guideText) dom.guideText.textContent = 'CAMERA PERMISSION REQUIRED - ALLOW IN BROWSER';
  }
}

function stopHandTracking() {
  state.hand.active = false;
  if (state.hand.rafId) {
    cancelAnimationFrame(state.hand.rafId);
    state.hand.rafId = null;
  }
  stopAllMovement();
  resetHandReference();
  stopCameraStreamOnly();
  state.camera.activeDeviceId = null;
}

function updateTrackingBadge(text, className) {
  if (dom.handStatusBadge) {
    dom.handStatusBadge.textContent = text;
    dom.handStatusBadge.className = `hand-badge ${className}`;
  }
}

// Automatically detect USB camera plug-in / unplug and disable laptop camera immediately
if (navigator.mediaDevices && navigator.mediaDevices.addEventListener) {
  navigator.mediaDevices.addEventListener('devicechange', () => {
    clearTimeout(state.camera.deviceChangeTimer);
    state.camera.deviceChangeTimer = setTimeout(() => {
      console.log('[CAMERA] USB device change detected — re-scanning cameras...');
      if (state.hand.active) {
        refreshAndSwitchCamera(true).catch((err) => console.warn('[CAMERA HOTPLUG]', err));
      }
    }, 350);
  });
}

// ============================================================================
// Initialization on Page Load
// ============================================================================
window.addEventListener('DOMContentLoaded', () => {
  // 1. Initialize 3D Robot Head (Column 1)
  initThreeJS();
  update3DHeadAngles(90);

  // 2. Event Listeners
  if (dom.btnQuickConnect) {
    dom.btnQuickConnect.addEventListener('click', connectSerial);
  }
  if (dom.btnBannerConnect) {
    dom.btnBannerConnect.addEventListener('click', connectSerial);
  }
  if (dom.btnCenterHead) {
    dom.btnCenterHead.addEventListener('click', centerHead);
  }
  if (dom.chkHandMirror) {
    dom.chkHandMirror.addEventListener('change', (e) => {
      state.hand.mirrorX = e.target.checked;
    });
  }
  if (dom.chkEnableTilt) {
    dom.chkEnableTilt.addEventListener('change', (e) => {
      state.hand.enableTilt = e.target.checked;
      if (!state.hand.enableTilt) {
        state.tilt = 90;
        state.targetTilt = 90;
        state.currentTiltZone = 'CENTER';
        if (state.tiltTimer) {
          clearInterval(state.tiltTimer);
          state.tiltTimer = null;
        }
      }
      update3DHeadAngles(state.pan, state.tilt);
    });
  }
  if (dom.selCameraSource) {
    dom.selCameraSource.addEventListener('change', async (e) => {
      const chosenId = e.target.value;
      if (chosenId) {
        state.camera.preferredExternalId = chosenId;
        try {
          const devices = await navigator.mediaDevices.enumerateDevices();
          const picked = devices.find((d) => d.kind === 'videoinput' && d.deviceId === chosenId);
          if (picked && picked.label && !isBuiltInLaptopCamera(picked.label)) {
            localStorage.setItem(PRIMARY_CAM_STORAGE_KEY, picked.label);
          }
        } catch (err) {}
        refreshAndSwitchCamera(true).catch((err) => console.warn('[CAMERA SELECT]', err));
      }
    });
  }
  if (dom.btnRestartCamera) {
    dom.btnRestartCamera.addEventListener('click', async () => {
      state.camera.preferredExternalId = null;
      stopHandTracking();
      await startHandTracking();
    });
  }

  // Return cleanly to Cockpit, closing USB port so index.html can open it immediately
  const btnBack = document.querySelector('.btn-back-cockpit');
  if (btnBack) {
    btnBack.addEventListener('click', async (e) => {
      e.preventDefault();
      stopHandTracking();
      await disconnectArduino();
      window.location.href = 'index.html';
    });
  }

  // 3. Auto-reconnect Web Serial if previously paired
  autoReconnectSerial();

  // 4. Start Webcam Hand Tracking (Auto-selects Robot USB Camera & disables Laptop Camera)
  startHandTracking();
});
