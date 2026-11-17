/**
 * RoboHead Hand Tracking Studio (2-Column Vision Control)
 * Column 1: 3D Robot Head Digital Twin (Three.js)
 * Column 2: Camera Hand Tracking Feed (MediaPipe Hands + Fallback)
 * With Web Serial API integration (9600 Baud to Arduino / ESP32)
 */

// ============================================================================
// State Management
// ============================================================================
const state = {
  pan: 90,
  tilt: 90,
  panMin: 20,
  panMax: 160,
  tiltMin: 40,
  tiltMax: 145,

  // Web Serial Port
  isConnected: false,
  serialPort: null,
  serialWriter: null,
  serialReader: null,
  baudRate: 9600,

  // Hand Tracker
  hand: {
    active: false,
    stream: null,
    camera: null,
    hands: null,
    hasHand: false,
    usingMediaPipe: false,
    animationId: null,
    mirrorX: true,
    invertY: false,
    smoothing: 0.35, // Lerp smoothing factor
    targetPan: 90,
    targetTilt: 90,
    smoothedPan: 90,
    smoothedTilt: 90,
    isFist: false,
    gesture: 'SEARCHING...',
    lastSerialDispatch: 0,
    // Offline canvas motion fallback
    prevFrameData: null,
    offCanvas: null,
    offCtx: null
  }
};

// Sequential serial write queue
let serialWriteChain = Promise.resolve();

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

  // Column 1: Face
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
  handStatusBadge: document.getElementById('handStatusBadge'),
  cameraGuide: document.getElementById('cameraGuide'),
  guideText: document.getElementById('guideText'),
  handMetricCoords: document.getElementById('handMetricCoords'),
  handValGesture: document.getElementById('handValGesture'),
  serialDispatchRate: document.getElementById('serialDispatchRate'),
  chkHandMirror: document.getElementById('chkHandMirror'),
  chkHandInvertY: document.getElementById('chkHandInvertY'),
  rngSmoothing: document.getElementById('rngSmoothing'),
  btnRestartCamera: document.getElementById('btnRestartCamera')
};

// ============================================================================
// Web Serial Communication (Arduino / ESP32 at 9600 Baud)
// ============================================================================

async function autoReconnectSerial(retryCount = 0) {
  if (!("serial" in navigator)) return;
  try {
    const ports = await navigator.serial.getPorts();
    if (ports.length > 0) {
      console.log(`[SERIAL] Paired USB port detected (attempt ${retryCount + 1}). Auto-connecting...`);
      state.serialPort = ports[0];
      await state.serialPort.open({ baudRate: state.baudRate });
      state.isConnected = true;
      updateConnectionUI(true);
      console.log('[SERIAL] Connected to Arduino Uno successfully!');
      setTimeout(() => {
        sendSerialData('HOME\n');
      }, 600);
      startSerialRead();
      return;
    }
  } catch (err) {
    console.warn(`[SERIAL AUTO-CONNECT] Attempt ${retryCount + 1}:`, err.message);
    if (retryCount < 3) {
      setTimeout(() => autoReconnectSerial(retryCount + 1), 400);
    }
  }
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    alert("Web Serial API is available in Google Chrome, Microsoft Edge, and Opera.\nPlease open this page in Chrome or Edge to connect your robot.");
    return;
  }

  if (state.isConnected && state.serialPort) {
    // Disconnect
    try {
      if (state.serialReader) await state.serialReader.cancel();
      if (state.serialPort) await state.serialPort.close();
    } catch (e) {}
    state.isConnected = false;
    state.serialPort = null;
    updateConnectionUI(false);
    return;
  }

  try {
    state.serialPort = await navigator.serial.requestPort();
    await state.serialPort.open({ baudRate: state.baudRate });
    state.isConnected = true;
    updateConnectionUI(true);

    setTimeout(() => {
      sendSerialData('HOME\n');
    }, 1000);

    startSerialRead();
  } catch (err) {
    console.error('[SERIAL CONNECT ERROR]', err);
    if (err.name === 'NetworkError' || (err.message && err.message.toLowerCase().includes('open'))) {
      alert("⚠️ COM Port is Busy!\n\n1. If Arduino IDE Serial Monitor is open, CLOSE IT.\n2. Close duplicate tabs.\n3. Unplug and replug the USB cable, then click CONNECT USB again.");
    }
    state.isConnected = false;
    state.serialPort = null;
    updateConnectionUI(false);
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
    if (dom.serialDispatchRate) {
      dom.serialDispatchRate.textContent = '20 Hz ONLINE';
      dom.serialDispatchRate.style.color = '#00ff88';
    }
    if (dom.offlineNotice) {
      dom.offlineNotice.style.display = 'none';
    }
  } else {
    dom.connIndicator.classList.remove('online');
    if (dom.connStatusText) dom.connStatusText.textContent = 'DISCONNECTED';
    if (dom.btnQuickConnect) {
      dom.btnQuickConnect.classList.remove('connected');
      dom.btnQuickConnect.innerHTML = '<span class="btn-icon">⚡</span> CONNECT USB';
    }
    if (dom.serialDispatchRate) {
      dom.serialDispatchRate.textContent = 'OFFLINE (CONNECT USB)';
      dom.serialDispatchRate.style.color = '#ff3b5c';
    }
    if (dom.offlineNotice) {
      dom.offlineNotice.style.display = 'flex';
    }
  }
}

function sendSerialData(text) {
  if (!state.serialPort || !state.isConnected) return;
  const dataToSend = text.endsWith('\n') ? text : (text + '\n');

  serialWriteChain = serialWriteChain.then(async () => {
    if (!state.serialPort || !state.serialPort.writable) return;
    try {
      const encoder = new TextEncoder();
      const writer = state.serialPort.writable.getWriter();
      await writer.write(encoder.encode(dataToSend));
      writer.releaseLock();
    } catch (err) {
      console.warn('[SERIAL WRITE ERR]', err);
    }
  });
}

async function startSerialRead() {
  while (state.serialPort && state.serialPort.readable && state.isConnected) {
    try {
      const textDecoder = new TextDecoderStream();
      state.serialPort.readable.pipeTo(textDecoder.writable);
      state.serialReader = textDecoder.readable.getReader();

      while (true) {
        const { value, done } = await state.serialReader.read();
        if (done) break;
      }
    } catch (err) {
      break;
    }
  }
}

// ============================================================================
// Column 1: Three.js 3D Robot Head Digital Twin Visualizer
// ============================================================================

let scene, camera, renderer;
let robotHeadGroup, neckPivot, upperFaceMesh, jawPivot;
let targetEulerYaw = 0;
let targetEulerPitch = 0;
let currentEulerYaw = 0;
let currentEulerPitch = 0;

function initThreeJS() {
  const container = dom.threeContainer;
  const width = container.clientWidth || 600;
  const height = container.clientHeight || 500;

  // Scene
  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x04060a, 0.035);

  // Camera
  camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  camera.position.set(0, 0.45, 4.4);
  camera.lookAt(0, 0.25, 0);

  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  // Lights
  const ambientLight = new THREE.AmbientLight(0x667799, 1.4);
  scene.add(ambientLight);

  const keyLight = new THREE.DirectionalLight(0x00f0ff, 1.6);
  keyLight.position.set(3, 4, 3);
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0xff0055, 1.2);
  fillLight.position.set(-3, 2, -2);
  scene.add(fillLight);

  // Ground Grid
  const gridHelper = new THREE.GridHelper(6, 20, 0x00f0ff, 0x152238);
  gridHelper.position.y = -1.25;
  scene.add(gridHelper);

  // Build Head
  buildRobotModel();

  // Mouse orbit
  setupViewDragging(container);

  // Resize
  window.addEventListener('resize', onWindowResize);

  // Loop
  animate();
}

function buildRobotModel() {
  // Holographic Emitter Pedestal
  const pedestalGeo = new THREE.CylinderGeometry(0.85, 1.05, 0.1, 32);
  const pedestalMat = new THREE.MeshStandardMaterial({
    color: 0x0c1322,
    metalness: 0.85,
    roughness: 0.3
  });
  const pedestal = new THREE.Mesh(pedestalGeo, pedestalMat);
  pedestal.position.y = -1.2;
  scene.add(pedestal);

  // Glowing base ring
  const ringGeo = new THREE.TorusGeometry(0.95, 0.02, 16, 64);
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, transparent: true, opacity: 0.5 });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = Math.PI / 2;
  ring.position.y = -1.14;
  scene.add(ring);

  // Pan / Yaw Pivot Group
  neckPivot = new THREE.Group();
  scene.add(neckPivot);

  // Tilt / Pitch Group
  robotHeadGroup = new THREE.Group();
  neckPivot.add(robotHeadGroup);

  // Texture Loader for Head Face
  const textureLoader = new THREE.TextureLoader();
  const faceTextureSrc = (typeof HEAD_IMAGE_DATA !== 'undefined' && HEAD_IMAGE_DATA)
    ? HEAD_IMAGE_DATA
    : 'head_mesh.png';
  const faceTexture = textureLoader.load(faceTextureSrc);
  faceTexture.minFilter = THREE.LinearFilter;
  faceTexture.magFilter = THREE.LinearFilter;

  // Face Geometry
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

  upperFaceMesh = new THREE.Mesh(upperGeo, faceMat);
  upperFaceMesh.position.set(0, upperCenterY, 0);
  robotHeadGroup.add(upperFaceMesh);

  // Jaw
  jawPivot = new THREE.Group();
  jawPivot.position.set(0, seamY, 0);
  robotHeadGroup.add(jawPivot);

  const jawGeo = new THREE.PlaneGeometry(faceWidth, jawHeight, 48, 16);
  const jawPos = jawGeo.attributes.position;
  const jawUv = jawGeo.attributes.uv;

  for (let i = 0; i < jawPos.count; i++) {
    const x = jawPos.getX(i);
    const localY = jawPos.getY(i);
    const worldY = localY + jawCenterY + seamY;
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

  // Glowing Visor Light behind face
  const visorLight = new THREE.PointLight(0x00f0ff, 1.2, 2.5);
  visorLight.position.set(0, 0.35, 0.1);
  robotHeadGroup.add(visorLight);
}

function update3DHeadAngles(panAngle, tiltAngle) {
  // Pan 90° is neutral (0 rad). 0° is -45°, 180° is +45°
  targetEulerYaw = -((panAngle - 90) * (Math.PI / 180));
  // Tilt 90° is neutral. 140° is look up (+tilt), 65° is look down (-tilt)
  targetEulerPitch = ((tiltAngle - 90) * (Math.PI / 180));

  // Update DOM HUD readouts
  const relYaw = (panAngle - 90).toFixed(1);
  const relPitch = (tiltAngle - 90).toFixed(1);

  if (dom.hudYaw) dom.hudYaw.textContent = `${relYaw > 0 ? '+' : ''}${relYaw}°`;
  if (dom.hudPitch) dom.hudPitch.textContent = `${relPitch > 0 ? '+' : ''}${relPitch}°`;
  if (dom.servoPanVal) dom.servoPanVal.textContent = `${panAngle}°`;
  if (dom.servoTiltVal) dom.servoTiltVal.textContent = `${tiltAngle}°`;
  if (dom.chipPanVal) dom.chipPanVal.textContent = `${panAngle}°`;
  if (dom.chipTiltVal) dom.chipTiltVal.textContent = `${tiltAngle}°`;
}

function animate() {
  requestAnimationFrame(animate);

  // Smooth interpolation toward target Euler angles
  currentEulerYaw += (targetEulerYaw - currentEulerYaw) * 0.18;
  currentEulerPitch += (targetEulerPitch - currentEulerPitch) * 0.18;

  if (neckPivot) neckPivot.rotation.y = currentEulerYaw;
  if (robotHeadGroup) robotHeadGroup.rotation.x = currentEulerPitch;

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

  window.addEventListener('mouseup', () => { isDragging = false; });

  window.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    const deltaX = e.clientX - prevMouseX;
    const deltaY = e.clientY - prevMouseY;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;

    if (camera) {
      camera.position.x += deltaX * 0.005;
      camera.position.y -= deltaY * 0.005;
      camera.position.x = Math.max(-2, Math.min(2, camera.position.x));
      camera.position.y = Math.max(-0.5, Math.min(1.5, camera.position.y));
      camera.lookAt(0, 0.25, 0);
    }
  });
}

// ============================================================================
// Column 2: Camera Hand Tracking Vision Engine
// ============================================================================

async function startHandTracking() {
  if (state.hand.active) return;

  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    alert("⚠️ Camera access not supported. Please open this page in Google Chrome or Microsoft Edge.");
    return;
  }

  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        width: { ideal: 640 },
        height: { ideal: 480 },
        facingMode: 'user'
      },
      audio: false
    });

    state.hand.stream = stream;
    state.hand.active = true;

    if (dom.handWebcam) {
      dom.handWebcam.srcObject = stream;
      await dom.handWebcam.play();
    }

    if (dom.cameraGuide) dom.cameraGuide.classList.remove('hidden');
    updateTrackingBadge('SEARCHING', 'searching');

    // Attempt MediaPipe Hands
    if (typeof window.Hands !== 'undefined') {
      setupMediaPipeHands();
    } else {
      startMotionTrackerFallback();
      let checkCount = 0;
      const checkTimer = setInterval(() => {
        checkCount++;
        if (typeof window.Hands !== 'undefined') {
          clearInterval(checkTimer);
          setupMediaPipeHands();
        } else if (checkCount > 25) {
          clearInterval(checkTimer);
        }
      }, 400);
    }
  } catch (err) {
    console.error('[CAMERA ERROR]', err);
    alert(`⚠️ Could not access webcam: ${err.message}\nPlease check camera permissions.`);
  }
}

function setupMediaPipeHands() {
  try {
    if (state.hand.hands) return;

    const hands = new Hands({
      locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/hands/${file}`
    });

    hands.setOptions({
      maxNumHands: 1,
      modelComplexity: 1,
      minDetectionConfidence: 0.5,
      minTrackingConfidence: 0.5
    });

    hands.onResults(onMediaPipeHandResults);
    state.hand.hands = hands;
    state.hand.usingMediaPipe = true;

    if (typeof Camera !== 'undefined' && dom.handWebcam) {
      state.hand.camera = new Camera(dom.handWebcam, {
        onFrame: async () => {
          if (state.hand.active && dom.handWebcam && dom.handWebcam.readyState >= 2) {
            await hands.send({ image: dom.handWebcam });
          }
        },
        width: 640,
        height: 480
      });
      state.hand.camera.start();
    } else {
      startManualVideoSendLoop();
    }
  } catch (e) {
    console.warn('[HAND TRACK] Using motion fallback:', e);
    startMotionTrackerFallback();
  }
}

function startManualVideoSendLoop() {
  async function loop() {
    if (!state.hand.active) return;
    if (dom.handWebcam && dom.handWebcam.readyState >= 2 && state.hand.hands) {
      try {
        await state.hand.hands.send({ image: dom.handWebcam });
      } catch (e) {}
    }
    state.hand.animationId = requestAnimationFrame(loop);
  }
  state.hand.animationId = requestAnimationFrame(loop);
}

function onMediaPipeHandResults(results) {
  if (!state.hand.active) return;
  const canvas = dom.handCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  ctx.save();
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Draw mirrored video
  ctx.translate(canvas.width, 0);
  ctx.scale(-1, 1);
  ctx.drawImage(results.image, 0, 0, canvas.width, canvas.height);
  ctx.restore();

  // Sci-fi dark overlay
  ctx.fillStyle = 'rgba(7, 10, 18, 0.28)';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
    state.hand.hasHand = true;
    if (dom.cameraGuide) dom.cameraGuide.classList.add('hidden');

    const rawLandmarks = results.multiHandLandmarks[0];
    const landmarks = rawLandmarks.map(pt => ({
      x: 1.0 - pt.x,
      y: pt.y,
      z: pt.z
    }));

    // Draw Cybernetic Skeleton
    drawCyberHandSkeleton(ctx, landmarks, canvas.width, canvas.height);

    // Palm midpoint
    const palmX = (landmarks[9].x * 0.7) + (landmarks[0].x * 0.3);
    const palmY = (landmarks[9].y * 0.7) + (landmarks[0].y * 0.3);

    // Analyze gesture
    const gesture = analyzeHandGesture(landmarks);
    state.hand.isFist = gesture.isFist;
    state.hand.gesture = gesture.name;

    // Draw Reticle
    drawTargetReticle(ctx, palmX * canvas.width, palmY * canvas.height, gesture.name);

    if (dom.handMetricCoords) {
      dom.handMetricCoords.textContent = `X: ${palmX.toFixed(2)} | Y: ${palmY.toFixed(2)}`;
    }
    if (dom.handValGesture) {
      dom.handValGesture.textContent = gesture.name;
    }

    if (gesture.isFist) {
      updateTrackingBadge('FIST // LOCKED', 'locked');
      if (dom.chipTrackState) dom.chipTrackState.textContent = 'LOCKED (FIST)';
    } else {
      updateTrackingBadge('TRACKING ACTIVE', 'tracking');
      if (dom.chipTrackState) dom.chipTrackState.textContent = 'FOLLOWING HAND';

      if (gesture.isPeace) {
        centerHead();
      } else {
        computeAndApplyKinematics(palmX, palmY);
      }
    }
  } else {
    state.hand.hasHand = false;
    if (dom.cameraGuide) dom.cameraGuide.classList.remove('hidden');
    updateTrackingBadge('SEARCHING HAND', 'searching');
    if (dom.handValGesture) dom.handValGesture.textContent = 'NO HAND DETECTED';
    if (dom.chipTrackState) dom.chipTrackState.textContent = 'SEARCHING...';
    drawSearchingCrosshairs(ctx, canvas.width, canvas.height);
  }
}

function analyzeHandGesture(lm) {
  const indexExt = lm[8].y < lm[6].y;
  const middleExt = lm[12].y < lm[10].y;
  const ringExt = lm[16].y < lm[14].y;
  const pinkyExt = lm[20].y < lm[18].y;
  const pinchDist = Math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y);

  if (!indexExt && !middleExt && !ringExt && !pinkyExt) {
    return { isFist: true, name: '✊ FIST (HOLD)' };
  }
  if (pinchDist < 0.055) {
    return { isFist: false, isPinch: true, name: '👌 PINCH' };
  }
  if (indexExt && middleExt && !ringExt && !pinkyExt) {
    return { isFist: false, isPeace: true, name: '✌️ PEACE (CENTER)' };
  }
  return { isFist: false, name: '🖐️ OPEN PALM' };
}

function computeAndApplyKinematics(visualX, visualY) {
  // Comfortable 15% margin on camera frame
  const normX = Math.max(0, Math.min(1, (visualX - 0.15) / 0.70));
  const normY = Math.max(0, Math.min(1, (visualY - 0.15) / 0.70));

  // Horizontal Pan: Hand Left (normX near 0) -> Pan Left (140°). Hand Right -> Pan Right (50°)
  const panRatio = state.hand.mirrorX ? (1.0 - normX) : normX;
  const targetPan = Math.round(state.panMin + panRatio * (state.panMax - state.panMin));

  // Vertical Tilt: Hand Up (normY near 0) -> Tilt Up (135°). Hand Down -> Tilt Down (60°)
  const tiltRatio = state.hand.invertY ? normY : (1.0 - normY);
  const targetTilt = Math.round(state.tiltMin + tiltRatio * (state.tiltMax - state.tiltMin));

  // Lerp smoothing
  state.hand.smoothedPan += (targetPan - state.hand.smoothedPan) * state.hand.smoothing;
  state.hand.smoothedTilt += (targetTilt - state.hand.smoothedTilt) * state.hand.smoothing;

  state.pan = Math.round(state.hand.smoothedPan);
  state.tilt = Math.round(state.hand.smoothedTilt);

  // Update 3D Face Viewport (Column 1)
  update3DHeadAngles(state.pan, state.tilt);

  // Throttled Serial Transmission to Arduino (Pin 6 Pan & Pin 9 Tilt)
  const now = performance.now();
  if (now - state.hand.lastSerialDispatch >= 50) { // 20 Hz stable rate
    state.hand.lastSerialDispatch = now;
    if (state.isConnected) {
      sendSerialData(`TILT ${state.tilt}\nPAN ${state.pan}\n`);
    }
  }
}

function centerHead() {
  state.hand.smoothedPan = 90;
  state.hand.smoothedTilt = 90;
  state.pan = 90;
  state.tilt = 90;
  update3DHeadAngles(90, 90);
  if (state.isConnected) {
    sendSerialData('HOME\n');
  }
}

// ============================================================================
// Fallback Motion & Centroid Tracker (Zero-Dependency Offline)
// ============================================================================

function startMotionTrackerFallback() {
  if (state.hand.usingMediaPipe) return;
  const canvas = dom.handCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  if (!state.hand.offCanvas) {
    state.hand.offCanvas = document.createElement('canvas');
    state.hand.offCanvas.width = 80;
    state.hand.offCanvas.height = 60;
    state.hand.offCtx = state.hand.offCanvas.getContext('2d', { willReadFrequently: true });
  }

  function frameLoop() {
    if (!state.hand.active || state.hand.usingMediaPipe) return;

    if (dom.handWebcam && dom.handWebcam.readyState >= 2) {
      ctx.save();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.translate(canvas.width, 0);
      ctx.scale(-1, 1);
      ctx.drawImage(dom.handWebcam, 0, 0, canvas.width, canvas.height);
      ctx.restore();

      ctx.fillStyle = 'rgba(7, 10, 18, 0.28)';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      const offW = 80, offH = 60;
      state.hand.offCtx.drawImage(dom.handWebcam, 0, 0, offW, offH);
      const curImgData = state.hand.offCtx.getImageData(0, 0, offW, offH);
      const curPixels = curImgData.data;

      if (state.hand.prevFrameData) {
        let sumX = 0, sumY = 0, totalDiff = 0;
        const prevPixels = state.hand.prevFrameData.data;

        for (let y = 0; y < offH; y++) {
          for (let x = 0; x < offW; x++) {
            const idx = (y * offW + x) * 4;
            const diffR = Math.abs(curPixels[idx] - prevPixels[idx]);
            const diffG = Math.abs(curPixels[idx + 1] - prevPixels[idx + 1]);
            const diffB = Math.abs(curPixels[idx + 2] - prevPixels[idx + 2]);
            const diff = (diffR + diffG + diffB) / 3;

            if (diff > 25) {
              const mx = offW - 1 - x;
              sumX += mx * diff;
              sumY += y * diff;
              totalDiff += diff;
            }
          }
        }

        if (totalDiff > 1200) {
          const centroidX = (sumX / totalDiff) / offW;
          const centroidY = (sumY / totalDiff) / offH;

          state.hand.hasHand = true;
          if (dom.cameraGuide) dom.cameraGuide.classList.add('hidden');
          updateTrackingBadge('MOTION TRACKING', 'tracking');
          if (dom.handValGesture) dom.handValGesture.textContent = '🖐️ MOTION FOLLOW';
          if (dom.chipTrackState) dom.chipTrackState.textContent = 'FOLLOWING HAND';

          drawTargetReticle(ctx, centroidX * canvas.width, centroidY * canvas.height, 'MOTION');
          computeAndApplyKinematics(centroidX, centroidY);
        } else {
          state.hand.hasHand = false;
          if (dom.cameraGuide) dom.cameraGuide.classList.remove('hidden');
          updateTrackingBadge('SEARCHING HAND', 'searching');
          if (dom.handValGesture) dom.handValGesture.textContent = 'WAVE HAND TO TRACK';
          if (dom.chipTrackState) dom.chipTrackState.textContent = 'SEARCHING...';
          drawSearchingCrosshairs(ctx, canvas.width, canvas.height);
        }
      }

      state.hand.prevFrameData = curImgData;
    }

    state.hand.animationId = requestAnimationFrame(frameLoop);
  }

  state.hand.animationId = requestAnimationFrame(frameLoop);
}

// ============================================================================
// Cyberpunk HUD Drawing Helpers
// ============================================================================

function drawCyberHandSkeleton(ctx, lm, w, h) {
  const connections = [
    [0, 1], [1, 2], [2, 3], [3, 4],
    [0, 5], [5, 6], [6, 7], [7, 8],
    [5, 9], [9, 10], [10, 11], [11, 12],
    [9, 13], [13, 14], [14, 15], [15, 16],
    [13, 17], [17, 18], [18, 19], [19, 20],
    [0, 17]
  ];

  ctx.lineWidth = 2.5;
  ctx.strokeStyle = '#00f0ff';
  ctx.shadowColor = '#00f0ff';
  ctx.shadowBlur = 8;

  connections.forEach(([i, j]) => {
    ctx.beginPath();
    ctx.moveTo(lm[i].x * w, lm[i].y * h);
    ctx.lineTo(lm[j].x * w, lm[j].y * h);
    ctx.stroke();
  });

  ctx.shadowBlur = 0;
  lm.forEach((pt, i) => {
    ctx.beginPath();
    ctx.arc(pt.x * w, pt.y * h, (i === 4 || i === 8 || i === 12 || i === 16 || i === 20) ? 5 : 3, 0, Math.PI * 2);
    ctx.fillStyle = (i === 4 || i === 8 || i === 12 || i === 16 || i === 20) ? '#ff0055' : '#00f0ff';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.stroke();
  });
}

function drawTargetReticle(ctx, x, y, label) {
  ctx.save();
  ctx.strokeStyle = '#00ff88';
  ctx.fillStyle = '#00ff88';
  ctx.shadowColor = '#00ff88';
  ctx.shadowBlur = 10;
  ctx.lineWidth = 2;

  ctx.beginPath();
  ctx.arc(x, y, 24, 0, Math.PI * 2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x - 34, y); ctx.lineTo(x - 12, y);
  ctx.moveTo(x + 12, y); ctx.lineTo(x + 34, y);
  ctx.moveTo(x, y - 34); ctx.lineTo(x, y - 12);
  ctx.moveTo(x, y + 12); ctx.lineTo(x, y + 34);
  ctx.stroke();

  const cx = ctx.canvas.width / 2;
  const cy = ctx.canvas.height / 2;
  ctx.beginPath();
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.45)';
  ctx.lineWidth = 1.5;
  ctx.moveTo(cx, cy);
  ctx.lineTo(x, y);
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.font = '11px JetBrains Mono, monospace';
  ctx.fillStyle = '#00ff88';
  ctx.fillText(label, x + 28, y - 10);

  ctx.restore();
}

function drawSearchingCrosshairs(ctx, w, h) {
  const cx = w / 2;
  const cy = h / 2;
  ctx.save();
  ctx.strokeStyle = 'rgba(255, 170, 0, 0.4)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 6]);

  ctx.beginPath();
  ctx.arc(cx, cy, 38, 0, Math.PI * 2);
  ctx.stroke();

  ctx.restore();
}

function updateTrackingBadge(text, className) {
  if (dom.handStatusBadge) {
    dom.handStatusBadge.textContent = text;
    dom.handStatusBadge.className = `hand-badge ${className}`;
  }
}

// ============================================================================
// Initialization on Page Load
// ============================================================================

window.addEventListener('DOMContentLoaded', () => {
  console.log('[HAND STUDIO] Initializing 2-Column Vision Studio...');

  // 1. Initialize 3D Robot Head (Column 1)
  initThreeJS();
  update3DHeadAngles(90, 90);

  // 2. Setup Event Listeners
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

  if (dom.chkHandInvertY) {
    dom.chkHandInvertY.addEventListener('change', (e) => {
      state.hand.invertY = e.target.checked;
    });
  }

  if (dom.rngSmoothing) {
    dom.rngSmoothing.addEventListener('input', (e) => {
      state.hand.smoothing = parseInt(e.target.value, 10) * 0.08;
    });
  }

  if (dom.btnRestartCamera) {
    dom.btnRestartCamera.addEventListener('click', () => {
      if (state.hand.stream) {
        state.hand.stream.getTracks().forEach(t => t.stop());
        state.hand.stream = null;
        state.hand.active = false;
      }
      startHandTracking();
    });
  }

  // Return cleanly to Cockpit, closing port so index.html can open it immediately
  const btnBack = document.querySelector('.btn-back-cockpit');
  if (btnBack) {
    btnBack.addEventListener('click', async (e) => {
      e.preventDefault();
      try {
        if (state.serialReader) await state.serialReader.cancel();
        if (state.serialPort) await state.serialPort.close();
      } catch (err) {}
      window.location.href = 'index.html';
    });
  }

  // 3. Auto-reconnect Web Serial if already paired in main dashboard
  autoReconnectSerial();

  // 4. Automatically start Webcam Hand Tracking (Column 2)
  startHandTracking();
});

