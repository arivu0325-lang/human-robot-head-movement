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

const VISION_CDN = 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14';
const MODEL_URL =
  'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task';

const state = {
  pan: 90,
  targetPan: 90,
  currentZone: 'HOME',
  moveTimer: null,

  // Web Serial Port (9600 baud)
  isConnected: false,
  serialPort: null,
  serialWriter: null,
  serialReader: null,
  serialReadPromise: null,
  keepReadingSerial: false,
  baudRate: 9600,

  // Hand Tracker
  hand: {
    active: false,
    stream: null,
    handLandmarker: null,
    usingLandmarker: false,
    rafId: null,
    lastVideoTime: -1,
    lastTimestamp: 0,
    hasHand: false,
    mirrorX: true,
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
    update3DHeadAngles(state.pan);
  }, 15);
}

function centerHead() {
  moveTo('HOME', { forceSend: true });
}

// ============================================================================
// Column 1: Three.js 3D Robot Head Visualizer (Left / Right Only)
// ============================================================================
let scene, camera, renderer;
let robotHeadGroup, neckPivot;
let targetEulerYaw = 0;
let currentEulerYaw = 0;

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

function update3DHeadAngles(panAngle) {
  // 90° = CENTER (0 rad)
  // 50° = RIGHT -> -(50 - 90) = +40° rad -> nose points RIGHT (+X)
  // 140° = LEFT -> -(140 - 90) = -50° rad -> nose points LEFT (-X)
  targetEulerYaw = -((panAngle - 90) * (Math.PI / 180));

  const relYaw = -(panAngle - 90);
  const dirTag = relYaw > 4 ? 'RIGHT' : relYaw < -4 ? 'LEFT' : 'CENTER';

  if (dom.hudYaw) dom.hudYaw.textContent = `${relYaw > 0 ? '+' : ''}${relYaw.toFixed(0)}° (${dirTag})`;
  if (dom.hudPitch) dom.hudPitch.textContent = 'LOCKED';
  if (dom.servoPanVal) dom.servoPanVal.textContent = `${panAngle}°`;
  if (dom.servoTiltVal) dom.servoTiltVal.textContent = '90° (LOCK)';
  if (dom.chipPanVal) dom.chipPanVal.textContent = `${panAngle}° (${dirTag})`;
  if (dom.chipTiltVal) dom.chipTiltVal.textContent = 'LOCKED';
}

function animate() {
  requestAnimationFrame(animate);
  currentEulerYaw += (targetEulerYaw - currentEulerYaw) * 0.25;
  if (neckPivot) neckPivot.rotation.y = currentEulerYaw;
  if (robotHeadGroup) robotHeadGroup.rotation.x = 0;
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

function zoneFromMirroredX(x) {
  // Hysteresis around 0.38 and 0.62 to avoid flicker at zone edges
  if (state.currentZone === 'LEFT') {
    if (x > 0.62) return 'RIGHT';
    if (x > 0.43) return 'HOME';
    return 'LEFT';
  }
  if (state.currentZone === 'RIGHT') {
    if (x < 0.38) return 'LEFT';
    if (x < 0.57) return 'HOME';
    return 'RIGHT';
  }
  if (x < 0.38) return 'LEFT';
  if (x > 0.62) return 'RIGHT';
  return 'HOME';
}

function drawZoneOverlay(ctx, w, h, handPoint, mirroredLandmarks, activeZone) {
  const leftLine = w * 0.38;
  const rightLine = w * 0.62;

  // Highlight active zone when hand is visible
  if (handPoint) {
    ctx.fillStyle = 'rgba(0, 240, 255, 0.14)';
    if (activeZone === 'LEFT') ctx.fillRect(0, 0, leftLine, h);
    else if (activeZone === 'HOME') ctx.fillRect(leftLine, 0, rightLine - leftLine, h);
    else if (activeZone === 'RIGHT') ctx.fillRect(rightLine, 0, w - rightLine, h);
  }

  // Zone divider dashed lines
  ctx.save();
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.42)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  ctx.moveTo(leftLine, 0);
  ctx.lineTo(leftLine, h);
  ctx.moveTo(rightLine, 0);
  ctx.lineTo(rightLine, h);
  ctx.stroke();
  ctx.setLineDash([]);

  // Zone headers
  ctx.font = '700 13px Orbitron, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillStyle = activeZone === 'LEFT' && handPoint ? '#00ff88' : 'rgba(226, 232, 240, 0.72)';
  ctx.fillText('← LEFT', leftLine / 2, 28);
  ctx.fillStyle = activeZone === 'HOME' && handPoint ? '#00f0ff' : 'rgba(226, 232, 240, 0.72)';
  ctx.fillText('CENTER', (leftLine + rightLine) / 2, 28);
  ctx.fillStyle = activeZone === 'RIGHT' && handPoint ? '#00ff88' : 'rgba(226, 232, 240, 0.72)';
  ctx.fillText('RIGHT →', rightLine + (w - rightLine) / 2, 28);
  ctx.restore();

  // Draw hand skeleton if landmarks are available
  if (mirroredLandmarks) {
    drawCyberHandSkeleton(ctx, mirroredLandmarks, w, h);
  }

  // Tracked palm marker
  if (handPoint) {
    const px = handPoint.x * w;
    const py = handPoint.y * h;

    ctx.save();
    ctx.beginPath();
    ctx.arc(px, py, 18, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0, 255, 136, 0.22)';
    ctx.fill();
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = '#00ff88';
    ctx.shadowColor = '#00ff88';
    ctx.shadowBlur = 10;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    ctx.font = '700 11px JetBrains Mono, monospace';
    ctx.fillStyle = '#00ff88';
    ctx.fillText(`${TARGETS[activeZone].name.toUpperCase()} (${TARGETS[activeZone].angle}°)`, px + 24, py + 4);
    ctx.restore();
  }
}

function drawCyberHandSkeleton(ctx, lm, w, h) {
  const connections = [
    [0, 1], [1, 2], [2, 3], [3, 4],
    [0, 5], [5, 6], [6, 7], [7, 8],
    [5, 9], [9, 10], [10, 11], [11, 12],
    [9, 13], [13, 14], [14, 15], [15, 16],
    [13, 17], [17, 18], [18, 19], [19, 20],
    [0, 17]
  ];

  ctx.save();
  ctx.lineWidth = 2;
  ctx.strokeStyle = '#00f0ff';
  ctx.shadowColor = '#00f0ff';
  ctx.shadowBlur = 6;

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
    ctx.fillStyle = i === 4 || i === 8 || i === 12 || i === 16 || i === 20 ? '#ff0055' : '#00f0ff';
    ctx.fill();
  });
  ctx.restore();
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
        if (dom.cameraGuide) dom.cameraGuide.classList.add('hidden');

        const lm = result.landmarks[0];
        // Mirror X to match selfie-view video
        const mirroredLandmarks = lm.map((pt) => ({
          x: 1 - pt.x,
          y: pt.y,
          z: pt.z
        }));

        // Landmark 9 (middle finger MCP) is centered on the palm
        const palm = lm[9];
        const visualX = 1 - palm.x;
        const controlX = state.hand.mirrorX ? visualX : palm.x;
        const zone = zoneFromMirroredX(controlX);

        moveTo(zone);
        drawZoneOverlay(ctx, w, h, { x: visualX, y: palm.y }, mirroredLandmarks, zone);

        const zoneLabel = TARGETS[zone].name.toUpperCase();
        updateTrackingBadge(`TRACKING: ${zoneLabel}`, 'tracking');
        if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${visualX.toFixed(2)} (${zoneLabel})`;
        if (dom.handValGesture) dom.handValGesture.textContent = `HAND → ${zoneLabel}`;
        if (dom.chipTrackState) dom.chipTrackState.textContent = `FOLLOWING → ${zoneLabel}`;
      } else {
        state.hand.hasHand = false;
        if (dom.cameraGuide) dom.cameraGuide.classList.remove('hidden');
        drawZoneOverlay(ctx, w, h, null, null, state.currentZone);
        updateTrackingBadge('SEARCHING HAND', 'searching');
        if (dom.handValGesture) dom.handValGesture.textContent = 'SHOW HAND...';
        if (dom.chipTrackState) dom.chipTrackState.textContent = 'SEARCHING...';
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

    if (totalDiff > 1200) {
      const visualX = sumX / totalDiff / offW;
      const visualY = sumY / totalDiff / offH;
      const controlX = state.hand.mirrorX ? visualX : 1 - visualX;
      const zone = zoneFromMirroredX(controlX);

      state.hand.hasHand = true;
      if (dom.cameraGuide) dom.cameraGuide.classList.add('hidden');
      moveTo(zone);
      drawZoneOverlay(ctx, w, h, { x: visualX, y: visualY }, null, zone);

      const zoneLabel = TARGETS[zone].name.toUpperCase();
      updateTrackingBadge(`MOTION: ${zoneLabel}`, 'tracking');
      if (dom.handMetricCoords) dom.handMetricCoords.textContent = `X: ${visualX.toFixed(2)} (${zoneLabel})`;
      if (dom.handValGesture) dom.handValGesture.textContent = `MOTION → ${zoneLabel}`;
      if (dom.chipTrackState) dom.chipTrackState.textContent = `FOLLOWING → ${zoneLabel}`;
    } else {
      state.hand.hasHand = false;
      drawZoneOverlay(ctx, w, h, null, null, state.currentZone);
      updateTrackingBadge('SEARCHING HAND', 'searching');
    }
  }

  state.hand.prevFrameData = curImgData;
}

// ============================================================================
// Smart USB Camera Manager (Auto-Enable Robot USB Cam & Disable Laptop Cam)
// ============================================================================

const BUILTIN_CAM_PATTERNS = [
  'integrated', 'built-in', 'builtin', 'internal', 'laptop',
  'facetime', 'truevision', 'user facing', 'hd user', 'front',
  'acer', 'lenovo', 'dell', 'asus', 'hp ', 'surface',
  'ir camera', 'windows hello', 'chicony', 'realtek', 'bison', 'quanta'
];

const EXTERNAL_USB_PATTERNS = [
  'usb', 'uvc', 'external', 'robot', 'pc camera', 'hd camera',
  'webcam', 'logitech', 'c270', 'c920', 'arducam', 'microdia',
  'sonix', 'general', 'capture', '2.0', '3.0'
];

function isBuiltInLaptopCamera(label) {
  const lower = (label || '').toLowerCase();
  if (!lower) return false;
  return BUILTIN_CAM_PATTERNS.some((kw) => lower.includes(kw));
}

function isExternalUsbCamera(label) {
  const lower = (label || '').toLowerCase();
  if (!lower) return false;
  return EXTERNAL_USB_PATTERNS.some((kw) => lower.includes(kw)) && !isBuiltInLaptopCamera(lower);
}

/**
 * Picks the best camera device:
 * - Whenever another USB camera is connected (2+ cameras, or an external USB camera),
 *   the built-in laptop camera is strictly DISABLED and the Robot USB camera is selected.
 * - Falls back to the single available camera only if no external USB camera is attached.
 */
function chooseBestCamera(videoDevices) {
  if (!videoDevices || videoDevices.length === 0) {
    return { device: null, isRobotCam: false, hasExternal: false };
  }

  // Only 1 camera connected on the system
  if (videoDevices.length === 1) {
    const onlyCam = videoDevices[0];
    const isExt = isExternalUsbCamera(onlyCam.label);
    return { device: onlyCam, isRobotCam: isExt, hasExternal: isExt };
  }

  // 2+ cameras detected -> External USB Robot Camera is connected!
  // Built-in laptop camera MUST be disabled and never chosen.
  const nonBuiltInCams = videoDevices.filter((d) => !isBuiltInLaptopCamera(d.label));
  const explicitUsbCams = videoDevices.filter((d) => isExternalUsbCamera(d.label));

  // If user manually chose one of the external USB cameras and it is still plugged in
  if (state.camera.preferredExternalId) {
    const preferred = (nonBuiltInCams.length > 0 ? nonBuiltInCams : videoDevices).find(
      (d) => d.deviceId === state.camera.preferredExternalId
    );
    if (preferred) {
      return { device: preferred, isRobotCam: true, hasExternal: true };
    }
  }

  if (explicitUsbCams.length > 0) {
    return { device: explicitUsbCams[0], isRobotCam: true, hasExternal: true };
  }
  if (nonBuiltInCams.length > 0) {
    return { device: nonBuiltInCams[0], isRobotCam: true, hasExternal: true };
  }

  // Fallback when OS hides labels or gives identical generic names:
  // Windows lists built-in laptop webcam at index 0 and external USB cameras after it.
  const externalFallback = videoDevices[videoDevices.length - 1];
  return { device: externalFallback, isRobotCam: true, hasExternal: true };
}

function updateCameraSourceUI(videoDevices, selectedDevice, isRobotCam, hasExternal) {
  if (dom.camSourceBadge) {
    if (!selectedDevice) {
      dom.camSourceBadge.textContent = '❌ NO CAMERA';
      dom.camSourceBadge.className = 'studio-pill cam-pill';
      dom.camSourceBadge.title = 'No camera detected';
    } else if (isRobotCam) {
      dom.camSourceBadge.textContent = '🤖 ROBOT USB CAM';
      dom.camSourceBadge.className = 'studio-pill cam-pill robot-cam';
      dom.camSourceBadge.title = `Active: ${selectedDevice.label || 'USB Camera'} (Laptop camera disabled)`;
    } else {
      dom.camSourceBadge.textContent = '💻 LAPTOP CAM';
      dom.camSourceBadge.className = 'studio-pill cam-pill laptop-cam';
      dom.camSourceBadge.title = `Active: ${selectedDevice.label || 'Built-in Camera'} — Connect Robot USB Camera to auto-switch`;
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
      const isBuiltIn = isBuiltInLaptopCamera(rawName) || (hasExternal && videoDevices.length >= 2 && idx === 0 && dev.deviceId !== selectedDevice?.deviceId && !isExternalUsbCamera(rawName));

      if (hasExternal && isBuiltIn) {
        opt.disabled = true;
        opt.textContent = `🚫 ${rawName} (Disabled)`;
      } else if (hasExternal) {
        opt.textContent = `🤖 ${rawName}`;
      } else {
        opt.textContent = `💻 ${rawName}`;
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
    state.hand.stream.getTracks().forEach((t) => t.stop());
    state.hand.stream = null;
  }
  if (dom.handWebcam) {
    dom.handWebcam.srcObject = null;
  }
}

async function openCameraStreamForDevice(targetDevice) {
  // Always stop previous stream tracks first so laptop camera LED turns OFF immediately
  stopCameraStreamOnly();

  const videoConstraints =
    targetDevice && targetDevice.deviceId
      ? { deviceId: { exact: targetDevice.deviceId }, width: { ideal: 640 }, height: { ideal: 480 } }
      : { width: { ideal: 640 }, height: { ideal: 480 }, facingMode: 'user' };

  let stream;
  try {
    stream = await navigator.mediaDevices.getUserMedia({ video: videoConstraints, audio: false });
  } catch (err) {
    // Fallback without strict resolution constraints if the USB camera uses a different native mode
    if (targetDevice && targetDevice.deviceId) {
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

      // Re-enumerate now that permission is granted and exact USB device labels are visible
      devices = await navigator.mediaDevices.enumerateDevices();
      videoDevices = devices.filter((d) => d.kind === 'videoinput');
    }

    const best = chooseBestCamera(videoDevices);
    state.camera.isRobotCam = best.isRobotCam;
    state.camera.hasExternalCam = best.hasExternal;

    const needsSwitch =
      best.device &&
      (best.device.deviceId !== state.camera.activeDeviceId || !state.hand.stream || forceRescan);

    if (needsSwitch && best.device) {
      if (best.device.deviceId !== state.camera.activeDeviceId || !state.hand.stream) {
        console.log(
          `[CAMERA] Switching to ${best.isRobotCam ? 'ROBOT USB CAMERA' : 'LAPTOP CAMERA'}: ${best.device.label || best.device.deviceId}`
        );
        await openCameraStreamForDevice(best.device);
      }
    }

    updateCameraSourceUI(videoDevices, best.device, best.isRobotCam, best.hasExternal);
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

    if (dom.guideText) dom.guideText.textContent = 'MOVE HAND LEFT OR RIGHT TO TURN HEAD';
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
  if (dom.selCameraSource) {
    dom.selCameraSource.addEventListener('change', (e) => {
      const chosenId = e.target.value;
      if (chosenId) {
        state.camera.preferredExternalId = chosenId;
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
