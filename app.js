/**
 * RoboHead Control System - Mission Control Client
 * Complete directional control (Up, Down, Left, Right, Center)
 * With 3D Head Visualizer, WebSocket/ROS/HTTP communication engine.
 */

// ============================================================================
// State Management
// ============================================================================
const state = {
  // Head servo angles (0° to 180°, neutral is 90°)
  pan: 90,    // Yaw (Left / Right)
  tilt: 90,   // Pitch (Up / Down)
  
  // Settings & Calibration
  panMin: 0,
  panMax: 180,
  tiltMin: 0,
  tiltMax: 180,
  stepSize: 5,
  speed: 5,
  continuous: true,
  invertPitch: false,
  invertYaw: false,

  // Protocol configuration
  protocol: 'serial', // 'serial' | 'websocket' | 'ros' | 'http' | 'simulation'
  baudRate: 9600,
  serialPort: null,
  serialReader: null,
  serialReadableStreamClosed: null,
  wsUrl: 'ws://192.168.4.1:81',
  rosWsUrl: 'ws://localhost:9090',
  rosTopic: '/robot/head_controller/command',
  httpUrl: 'http://192.168.4.1/look?pan={pan}&tilt={tilt}&dir={dir}',
  httpMethod: 'GET',
  camUrl: '',

  // Runtime connection state
  isConnected: false,
  isConnecting: false,
  socket: null,
  pingInterval: null,
  lastPingTime: 0,
  latency: 0,

  // Stats
  stats: {
    sent: 0,
    recv: 0,
    dropped: 0
  },

  // Standing pulse loop
  pulseLoopActive: false,
  pulseIntervalId: null,
  pulseFrequency: 1.0,
  pulseSpeed: parseInt(localStorage.getItem('robo_pulse_speed') || '20', 10),

  // Robotic Mouth Actuator (Pin 3)
  mouthActive: false,

  // Food Pipe Actuator
  foodPipeOpen: false,

  // Air Blow Actuator & Relay Cycle Configuration (Pin 4 Active LOW)
  blowActive: false,
  blowInterval: parseFloat(localStorage.getItem('robo_blow_interval') || '2.0'),

  // Animation / Patrol state
  activeAnimation: null
};

// Continuous press timer
let holdInterval = null;
let activePressedDir = null;

// ============================================================================
// DOM Elements
// ============================================================================
const dom = {
  // D-Pad buttons
  btnUp: document.getElementById('btnUp'),
  btnDown: document.getElementById('btnDown'),
  btnLeft: document.getElementById('btnLeft'),
  btnRight: document.getElementById('btnRight'),
  btnCenter: document.getElementById('btnCenter'),
  
  // Sliders
  sliderPan: document.getElementById('sliderPan'),
  sliderTilt: document.getElementById('sliderTilt'),
  sliderPanLabel: document.getElementById('sliderPanLabel'),
  sliderTiltLabel: document.getElementById('sliderTiltLabel'),
  speedRange: document.getElementById('speedRange'),
  speedValue: document.getElementById('speedValue'),
  stepSelector: document.getElementById('stepSelector'),
  chkContinuous: document.getElementById('chkContinuous'),
  chkInvertPitch: document.getElementById('chkInvertPitch'),
  chkInvertYaw: document.getElementById('chkInvertYaw'),

  // HUD Readouts
  hudYaw: document.getElementById('hudYaw'),
  hudPitch: document.getElementById('hudPitch'),
  servoPanVal: document.getElementById('servoPanVal'),
  servoTiltVal: document.getElementById('servoTiltVal'),
  fpsBadge: document.getElementById('fpsBadge'),
  latencyBadge: document.getElementById('latencyBadge'),
  protocolBadge: document.getElementById('protocolBadge'),

  // Connection
  connIndicator: document.getElementById('connIndicator'),
  connStatusText: document.getElementById('connStatusText'),
  btnQuickConnect: document.getElementById('btnQuickConnect'),
  btnOpenSettings: document.getElementById('btnOpenSettings'),
  btnOpenLogs: document.getElementById('btnOpenLogs'),
  activeProtoLabel: document.getElementById('activeProtoLabel'),
  activeUrlDisplay: document.getElementById('activeUrlDisplay'),
  statSent: document.getElementById('statSent'),
  statRecv: document.getElementById('statRecv'),
  statDropped: document.getElementById('statDropped'),
  terminalLog: document.getElementById('terminalLog'),
  btnClearLog: document.getElementById('btnClearLog'),
  txStatus: document.getElementById('txStatus'),
  customCmdInput: document.getElementById('customCmdInput'),
  btnSendCustom: document.getElementById('btnSendCustom'),

  // Logs Modal
  logsModal: document.getElementById('logsModal'),
  btnCloseLogs: document.getElementById('btnCloseLogs'),

  // Header Logo & Admin Modal
  headerLogoBadge: document.getElementById('headerLogoBadge'),
  headerPulseRing: document.getElementById('headerPulseRing'),
  adminModal: document.getElementById('adminModal'),
  btnCloseAdmin: document.getElementById('btnCloseAdmin'),
  btnBackFromAdmin: document.getElementById('btnBackFromAdmin'),
  btnCloseAdminFooter: document.getElementById('btnCloseAdminFooter'),
  adminCmdInput: document.getElementById('adminCmdInput'),
  btnAdminSendCmd: document.getElementById('btnAdminSendCmd'),
  btnAdminOpenSettings: document.getElementById('btnAdminOpenSettings'),
  btnAdminOpenLogs: document.getElementById('btnAdminOpenLogs'),

  // Tabs & Viewports
  threeContainer: document.getElementById('threeContainer'),

  // Presets & Actuators
  btnMouthOn: document.getElementById('btnMouthOn'),
  btnMouthOff: document.getElementById('btnMouthOff'),
  btnFoodPipe: document.getElementById('btnFoodPipe'),
  foodPipeDot: document.getElementById('foodPipeDot'),
  lblFoodPipeText: document.getElementById('lblFoodPipeText'),
  btnQuickMouthOn: document.getElementById('btnQuickMouthOn'),
  btnQuickMouthClose: document.getElementById('btnQuickMouthClose'),
  btnPatrol: document.getElementById('btnPatrol'),
  btnPatrolTilt: document.getElementById('btnPatrolTilt'),
  btnNeckPulse: document.getElementById('btnNeckPulse'),
  btnPresetCenter: document.getElementById('btnPresetCenter'),
  btnEStop: document.getElementById('btnEStop'),

  // Right Column Controls
  btnPulseToggle: document.getElementById('btnPulseToggle'),
  pulseStatusBadge: document.getElementById('pulseStatusBadge'),
  pulseToggleTitle: document.getElementById('pulseToggleTitle'),
  pulseToggleSub: document.getElementById('pulseToggleSub'),
  lblPulseSpeed: document.getElementById('lblPulseSpeed'),
  pillsPulseSpeed: document.querySelectorAll('.pill-pulse-speed'),
  btnMouthToggle: document.getElementById('btnMouthToggle'),
  mouthStatusBadge: document.getElementById('mouthStatusBadge'),
  mouthToggleTitle: document.getElementById('mouthToggleTitle'),
  mouthToggleSub: document.getElementById('mouthToggleSub'),
  btnBlow: document.getElementById('btnBlow'),
  btnBlowOn: document.getElementById('btnBlowOn'),
  btnBlowOff: document.getElementById('btnBlowOff'),
  btnPresetBlow: document.getElementById('btnPresetBlow'),
  lblBlowDuration: document.getElementById('lblBlowDuration'),
  lblLungsStep: document.getElementById('lblLungsStep'),
  sliderBlowInterval: document.getElementById('sliderBlowInterval'),
  numBlowInterval: document.getElementById('numBlowInterval'),
  btnBlowDec: document.getElementById('btnBlowDec'),
  btnBlowInc: document.getElementById('btnBlowInc'),
  pillsBlowPreset: document.querySelectorAll('.pill-blow-preset'),
  pillsLungsStep: document.querySelectorAll('.pill-lungs-step'),
  blowRelayDot: document.getElementById('blowRelayDot'),
  blowCycleText: document.getElementById('blowCycleText'),
  blowCycleProgress: document.getElementById('blowCycleProgress'),
  speedPresets: document.querySelectorAll('.pill-speed'),

  // Customized Robot Q&A Elements
  qaButtons: document.querySelectorAll('.qa-btn'),
  qaDisplayPrompt: document.getElementById('qaDisplayPrompt'),
  qaAnswerText: document.getElementById('qaAnswerText'),
  qaStatusTag: document.getElementById('qaStatusTag'),
  qaVoiceBtn: document.getElementById('qaVoiceBtn'),

  // Voice Interaction Elements
  btnHeaderMic: document.getElementById('btnHeaderMic'),
  btnVoiceMic: document.getElementById('btnVoiceMic'),
  voiceMicTitle: document.getElementById('voiceMicTitle'),
  voiceMicSub: document.getElementById('voiceMicSub'),
  voiceMicBadge: document.getElementById('voiceMicBadge'),
  voiceHeardBanner: document.getElementById('voiceHeardBanner'),
  voiceHeardText: document.getElementById('voiceHeardText'),

  // Settings Modal
  settingsModal: document.getElementById('settingsModal'),
  btnCloseSettings: document.getElementById('btnCloseSettings'),
  btnCancelSettings: document.getElementById('btnCancelSettings'),
  btnSaveSettings: document.getElementById('btnSaveSettings'),
  serialGroup: document.getElementById('serialGroup'),
  selBaudRate: document.getElementById('selBaudRate'),
  wsGroup: document.getElementById('wsGroup'),
  rosGroup: document.getElementById('rosGroup'),
  httpGroup: document.getElementById('httpGroup'),
  txtWsUrl: document.getElementById('txtWsUrl'),
  txtRosTopic: document.getElementById('txtRosTopic'),
  txtRosWsUrl: document.getElementById('txtRosWsUrl'),
  txtHttpUrl: document.getElementById('txtHttpUrl'),
  selHttpMethod: document.getElementById('selHttpMethod'),
  txtCamUrl: document.getElementById('txtCamUrl'),
  numPanMin: document.getElementById('numPanMin'),
  numPanMax: document.getElementById('numPanMax'),
  numTiltMin: document.getElementById('numTiltMin'),
  numTiltMax: document.getElementById('numTiltMax'),

  // Hand Tracking Elements
  btnHeaderHand: document.getElementById('btnHeaderHand'),
  btnPresetHand: document.getElementById('btnPresetHand'),
  handHudOverlay: document.getElementById('handHudOverlay'),
  handVideoContainer: document.getElementById('handVideoContainer'),
  handWebcam: document.getElementById('handWebcam'),
  handCanvas: document.getElementById('handCanvas'),
  handStatusBadge: document.getElementById('handStatusBadge'),
  btnMinimizeHandHud: document.getElementById('btnMinimizeHandHud'),
  btnCloseHandHud: document.getElementById('btnCloseHandHud'),
  handValPan: document.getElementById('handValPan'),
  handValTilt: document.getElementById('handValTilt'),
  handValGesture: document.getElementById('handValGesture'),
  chkHandMirror: document.getElementById('chkHandMirror'),
  chkHandInvertY: document.getElementById('chkHandInvertY')
};

// ============================================================================
// Directional Control Logic (Up, Down, Left, Right, Center)
// ============================================================================

/**
 * Executes a directional movement step.
 * @param {'UP'|'DOWN'|'LEFT'|'RIGHT'|'CENTER'} dir
 */
function moveDirection(dir) {
  // Cancel autonomous animation if user manual control takes over
  if (state.activeAnimation && dir !== 'PATROL') {
    stopAnimation();
  }

  let deltaPan = 0;
  let deltaTilt = 0;
  const step = state.stepSize;

  switch (dir) {
    case 'UP':
      // Tilt increases or decreases depending on inversion
      deltaTilt = state.invertPitch ? -step : step;
      break;
    case 'DOWN':
      deltaTilt = state.invertPitch ? step : -step;
      break;
    case 'LEFT':
      deltaPan = state.invertYaw ? step : -step;
      break;
    case 'RIGHT':
      deltaPan = state.invertYaw ? -step : step;
      break;
    case 'CENTER':
      setHeadPosition(90, 90, 'CENTER');
      return;
    default:
      return;
  }

  const newPan = clamp(state.pan + deltaPan, state.panMin, state.panMax);
  const newTilt = clamp(state.tilt + deltaTilt, state.tiltMin, state.tiltMax);

  setHeadPosition(newPan, newTilt, dir);
}

/**
 * Updates internal head angles, UI sliders, 3D visualizer, and dispatches command.
 */
function setHeadPosition(newPan, newTilt, actionDir = 'MOVE') {
  const panChanged = state.pan !== newPan;
  const tiltChanged = state.tilt !== newTilt;

  state.pan = Math.round(newPan);
  state.tilt = Math.round(newTilt);

  // Update DOM sliders & HUD
  if (dom.sliderPan) dom.sliderPan.value = state.pan;
  if (dom.sliderTilt) dom.sliderTilt.value = state.tilt;
  if (dom.sliderPanLabel) dom.sliderPanLabel.textContent = `${state.pan}°`;
  if (dom.sliderTiltLabel) dom.sliderTiltLabel.textContent = `${state.tilt}°`;
  if (dom.servoPanVal) dom.servoPanVal.textContent = `${state.pan}°`;
  if (dom.servoTiltVal) dom.servoTiltVal.textContent = `${state.tilt}°`;

  // Calculated relative degrees from center (-90 to +90)
  const relYaw = (state.pan - 90).toFixed(1);
  const relPitch = (state.tilt - 90).toFixed(1);
  if (dom.hudYaw) dom.hudYaw.textContent = `${relYaw > 0 ? '+' : ''}${relYaw}°`;
  if (dom.hudPitch) dom.hudPitch.textContent = `${relPitch > 0 ? '+' : ''}${relPitch}°`;

  // Dispatch network command
  sendCommand({
    cmd: 'LOOK',
    action: actionDir,
    pan: state.pan,
    tilt: state.tilt,
    relYaw: parseFloat(relYaw),
    relPitch: parseFloat(relPitch),
    timestamp: Date.now()
  });
}

/**
 * Starts continuous movement when button/key is held down.
 */
function startContinuousMovement(dir) {
  if (activePressedDir === dir) return;
  activePressedDir = dir;
  highlightButton(dir, true);

  // Immediately execute first step
  moveDirection(dir);

  if (state.continuous && dir !== 'CENTER') {
    clearInterval(holdInterval);
    // Interval rate inversely proportional to speed (1 = 200ms, 10 = 35ms)
    const intervalMs = Math.max(30, 220 - (state.speed * 18));
    holdInterval = setInterval(() => {
      moveDirection(dir);
    }, intervalMs);
  }
}

/**
 * Stops continuous movement.
 */
function stopContinuousMovement() {
  if (activePressedDir) {
    highlightButton(activePressedDir, false);
    activePressedDir = null;
  }
  if (holdInterval) {
    clearInterval(holdInterval);
    holdInterval = null;
  }
}

function highlightButton(dir, isActive) {
  const btnMap = {
    UP: dom.btnUp,
    DOWN: dom.btnDown,
    LEFT: dom.btnLeft,
    RIGHT: dom.btnRight,
    CENTER: dom.btnCenter
  };
  const btn = btnMap[dir];
  if (btn) {
    btn.classList.toggle('active-key', isActive);
  }
}

// ============================================================================
// Keyboard Navigation (Arrow Keys + WASD + Space)
// ============================================================================
const keyMap = {
  ArrowUp: 'UP',
  KeyW: 'UP',
  w: 'UP',
  ArrowDown: 'DOWN',
  KeyS: 'DOWN',
  s: 'DOWN',
  ArrowLeft: 'LEFT',
  KeyA: 'LEFT',
  a: 'LEFT',
  ArrowRight: 'RIGHT',
  KeyD: 'RIGHT',
  d: 'RIGHT',
  Space: 'CENTER',
  ' ': 'CENTER'
};

const activeKeys = new Set();

window.addEventListener('keydown', (e) => {
  // Ignore keys if typing in custom input or settings modal
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

  if (e.code === 'KeyM' || e.key === 'm' || e.key === 'M') {
    e.preventDefault();
    toggleMouth();
    return;
  }

  if (e.code === 'KeyP' || e.key === 'p' || e.key === 'P') {
    e.preventDefault();
    togglePulseLoop();
    return;
  }

  const dir = keyMap[e.code] || keyMap[e.key];
  if (dir && !activeKeys.has(e.code)) {
    e.preventDefault();
    activeKeys.add(e.code);
    startContinuousMovement(dir);
  }
});

window.addEventListener('keyup', (e) => {
  if (keyMap[e.code] || keyMap[e.key]) {
    activeKeys.delete(e.code);
    if (activeKeys.size === 0) {
      stopContinuousMovement();
    } else {
      // Pick the remaining active key
      const nextKey = [...activeKeys][activeKeys.size - 1];
      const nextDir = keyMap[nextKey];
      if (nextDir) startContinuousMovement(nextDir);
    }
  }
});

// Bind pointer events on D-Pad buttons
[
  { el: dom.btnUp, dir: 'UP' },
  { el: dom.btnDown, dir: 'DOWN' },
  { el: dom.btnLeft, dir: 'LEFT' },
  { el: dom.btnRight, dir: 'RIGHT' },
  { el: dom.btnCenter, dir: 'CENTER' }
].forEach(({ el, dir }) => {
  el.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    startContinuousMovement(dir);
  });
  el.addEventListener('pointerup', stopContinuousMovement);
  el.addEventListener('pointerleave', stopContinuousMovement);
  el.addEventListener('pointercancel', stopContinuousMovement);
});

// Sliders live input
dom.sliderPan.addEventListener('input', (e) => {
  setHeadPosition(parseInt(e.target.value, 10), state.tilt, 'PAN_SLIDER');
});

dom.sliderTilt.addEventListener('input', (e) => {
  setHeadPosition(state.pan, parseInt(e.target.value, 10), 'TILT_SLIDER');
});

if (dom.speedRange) {
  dom.speedRange.addEventListener('input', (e) => {
    state.speed = parseInt(e.target.value, 10);
    if (dom.speedValue) dom.speedValue.textContent = `${state.speed}x`;
    if (dom.speedPresets) {
      dom.speedPresets.forEach(p => {
        p.classList.toggle('active', parseInt(p.dataset.speed, 10) === state.speed);
      });
    }
  });
}

// Step selector pills
if (dom.stepSelector) {
  dom.stepSelector.addEventListener('click', (e) => {
    if (e.target.classList.contains('pill')) {
      dom.stepSelector.querySelectorAll('.pill').forEach(p => p.classList.remove('active'));
      e.target.classList.add('active');
      state.stepSize = parseInt(e.target.dataset.step, 10);
      log(`[INFO] Step size set to ${state.stepSize}°`, 'info');
    }
  });
}

// Options toggles
if (dom.chkContinuous) {
  dom.chkContinuous.addEventListener('change', (e) => {
    state.continuous = e.target.checked;
  });
}
if (dom.chkInvertPitch) {
  dom.chkInvertPitch.addEventListener('change', (e) => {
    state.invertPitch = e.target.checked;
  });
}
if (dom.chkInvertYaw) {
  dom.chkInvertYaw.addEventListener('change', (e) => {
    state.invertYaw = e.target.checked;
  });
}

// ============================================================================
// Autonomous Presets & Sequences
// ============================================================================

[dom.btnMouthOn, dom.btnQuickMouthOn].forEach(btn => {
  if (btn) {
    btn.addEventListener('click', () => {
      toggleMouth(true);
    });
  }
});

[dom.btnMouthOff, dom.btnQuickMouthClose].forEach(btn => {
  if (btn) {
    btn.addEventListener('click', () => {
      toggleMouth(false);
    });
  }
});

if (dom.btnFoodPipe) {
  dom.btnFoodPipe.addEventListener('click', () => {
    toggleFoodPipe();
  });
}

if (dom.btnPatrolTilt) {
  dom.btnPatrolTilt.addEventListener('click', () => {
    toggleTiltPatrolScan();
  });
}

if (dom.btnPatrol) {
  dom.btnPatrol.addEventListener('click', () => {
    togglePatrolScan();
  });
}

if (dom.btnPresetCenter) {
  dom.btnPresetCenter.addEventListener('click', () => {
    setHeadPosition(90, 90, 'CENTER');
  });
}

if (dom.btnNeckPulse) {
  dom.btnNeckPulse.addEventListener('click', () => {
    triggerNeckPulse();
  });
}

if (dom.btnEStop) {
  dom.btnEStop.addEventListener('click', () => {
    emergencyStop();
  });
}

function stopAnimation() {
  if (state.activeAnimation) {
    clearInterval(state.activeAnimation);
    state.activeAnimation = null;
  }
  if (dom.btnPatrol) {
    dom.btnPatrol.classList.remove('active');
    dom.btnPatrol.textContent = '↔️ Sweep Left/Right';
  }
  if (dom.btnPatrolTilt) {
    dom.btnPatrolTilt.classList.remove('active');
    dom.btnPatrolTilt.textContent = '↕️ Sweep Up/Down';
  }
}

function playNodSequence() {
  stopAnimation();
  log('[PRESET] Executing Nod "Yes" sequence...', 'info');
  const basePan = state.pan;
  const originalTilt = state.tilt;
  const frames = [
    { tilt: Math.min(140, originalTilt + 25), dur: 220 },
    { tilt: Math.max(45, originalTilt - 25), dur: 240 },
    { tilt: Math.min(135, originalTilt + 20), dur: 220 },
    { tilt: Math.max(50, originalTilt - 20), dur: 240 },
    { tilt: originalTilt, dur: 200 }
  ];

  let i = 0;
  function runStep() {
    if (i < frames.length) {
      setHeadPosition(basePan, frames[i].tilt, 'NOD');
      setTimeout(runStep, frames[i].dur);
      i++;
    }
  }
  runStep();
}

function playShakeSequence() {
  stopAnimation();
  log('[PRESET] Executing Shake "No" sequence...', 'info');
  const originalPan = state.pan;
  const baseTilt = state.tilt;
  const frames = [
    { pan: Math.max(40, originalPan - 35), dur: 200 },
    { pan: Math.min(140, originalPan + 35), dur: 240 },
    { pan: Math.max(50, originalPan - 25), dur: 220 },
    { pan: Math.min(130, originalPan + 25), dur: 220 },
    { pan: originalPan, dur: 200 }
  ];

  let i = 0;
  function runStep() {
    if (i < frames.length) {
      setHeadPosition(frames[i].pan, baseTilt, 'SHAKE');
      setTimeout(runStep, frames[i].dur);
      i++;
    }
  }
  runStep();
}

let tiltPatrolDirection = 1;
function toggleTiltPatrolScan() {
  if (state.activeAnimation) {
    const wasRunningThis = dom.btnPatrolTilt && dom.btnPatrolTilt.classList.contains('active');
    stopAnimation();
    if (wasRunningThis) {
      log('[PRESET] Up/Down Sweep scan stopped.', 'warn');
      return;
    }
  }

  log('[PRESET] ↕️ Up/Down Sweep Scan started.', 'info');
  if (dom.btnPatrolTilt) {
    dom.btnPatrolTilt.classList.add('active');
    dom.btnPatrolTilt.textContent = '⏹ Stop Up/Down';
  }

  const step = Math.max(1, Math.round(state.speed / 2));
  state.activeAnimation = setInterval(() => {
    let newTilt = state.tilt + (tiltPatrolDirection * step);
    if (newTilt >= 145) {
      newTilt = 145;
      tiltPatrolDirection = -1;
    } else if (newTilt <= 35) {
      newTilt = 35;
      tiltPatrolDirection = 1;
    }
    setHeadPosition(state.pan, newTilt, 'SWEEP_TILT');
  }, 45);
}

let patrolDirection = 1;
function togglePatrolScan() {
  if (state.activeAnimation) {
    const wasRunningThis = dom.btnPatrol && dom.btnPatrol.classList.contains('active');
    stopAnimation();
    if (wasRunningThis) {
      log('[PRESET] Left/Right Sweep scan stopped.', 'warn');
      return;
    }
  }

  log('[PRESET] ↔️ Left/Right Sweep Scan started.', 'info');
  if (dom.btnPatrol) {
    dom.btnPatrol.classList.add('active');
    dom.btnPatrol.textContent = '⏹ Stop Left/Right';
  }

  const step = Math.max(1, Math.round(state.speed / 2));
  state.activeAnimation = setInterval(() => {
    let newPan = state.pan + (patrolDirection * step);
    if (newPan >= 160) {
      newPan = 160;
      patrolDirection = -1;
    } else if (newPan <= 20) {
      newPan = 20;
      patrolDirection = 1;
    }
    setHeadPosition(newPan, state.tilt, 'SWEEP_PAN');
  }, 45);
}

function emergencyStop() {
  stopAnimation();
  stopContinuousMovement();
  stopPulseLoop();
  if (state.mouthActive) toggleMouth(false);
  if (state.foodPipeOpen) toggleFoodPipe(false);
  sendCommand({
    cmd: 'ESTOP',
    action: 'EMERGENCY_STOP',
    timestamp: Date.now()
  });
  log('[ALERT] 🛑 EMERGENCY STOP TRIGGERED! Commands halted.', 'err');
}

function playPulseSound() {
  try {
    const AudioCtx = window.AudioContext || window.webkitAudioContext;
    if (!AudioCtx) return;
    const ctx = new AudioCtx();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = 'sawtooth';
    osc.frequency.setValueAtTime(140, ctx.currentTime);
    osc.frequency.exponentialRampToValueAtTime(540, ctx.currentTime + 0.18);
    osc.frequency.exponentialRampToValueAtTime(95, ctx.currentTime + 0.42);
    gain.gain.setValueAtTime(0.18, ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.01, ctx.currentTime + 0.48);
    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.5);
  } catch {}
}

function triggerNeckPulse() {
  neckPulseActive = true;
  neckPulseStartTime = performance.now();

  if (dom.btnNeckPulse) {
    dom.btnNeckPulse.classList.add('pulsing');
    setTimeout(() => {
      if (dom.btnNeckPulse) dom.btnNeckPulse.classList.remove('pulsing');
    }, NECK_PULSE_DURATION);
  }

  playPulseSound();

  sendCommand({
    cmd: 'PULSE',
    target: 'neck',
    action: 'NECK_PULSE',
    duration: NECK_PULSE_DURATION,
    pan: state.pan,
    tilt: state.tilt,
    timestamp: Date.now()
  });

  log('[ACTION] ⚡ Running pulse in the neck...', 'tx');
}

/**
 * Sets the Blow On/Off Cycle duration in seconds.
 * Synchronizes the UI slider, stepper, preset pills, and transmits INTERVAL <sec> to Arduino/ESP32.
 * @param {number|string} sec - Interval duration in seconds (e.g. 1.0, 2.0, 3.0)
 * @param {boolean} notifyHardware - Whether to transmit command over Serial
 */
function setBlowInterval(sec, notifyHardware = true) {
  let val = parseFloat(sec);
  if (isNaN(val) || val <= 0) val = 2.0;
  val = Math.max(0.5, Math.min(30.0, Math.round(val * 10) / 10)); // 0.5s to 30.0s clamp

  state.blowInterval = val;
  try {
    localStorage.setItem('robo_blow_interval', val.toString());
  } catch {}

  let stepLabel = 'STEP 2';
  if (Math.abs(val - 1.0) < 0.1) stepLabel = 'STEP 1';
  else if (Math.abs(val - 2.0) < 0.1) stepLabel = 'STEP 2';
  else if (Math.abs(val - 3.0) < 0.1) stepLabel = 'STEP 3';
  else stepLabel = `${val.toFixed(1)}s`;

  if (dom.lblLungsStep) {
    dom.lblLungsStep.textContent = stepLabel;
  }
  if (dom.lblBlowDuration) {
    dom.lblBlowDuration.textContent = val.toFixed(1) + 's';
  }
  if (dom.sliderBlowInterval && Math.abs(parseFloat(dom.sliderBlowInterval.value) - val) > 0.05) {
    dom.sliderBlowInterval.value = val;
  }
  if (dom.numBlowInterval && Math.abs(parseFloat(dom.numBlowInterval.value) - val) > 0.05) {
    dom.numBlowInterval.value = val;
  }

  // Update preset pills active highlight (both left panel and right panel)
  if (dom.pillsBlowPreset) {
    dom.pillsBlowPreset.forEach(pill => {
      const pSec = parseFloat(pill.dataset.sec);
      if (Math.abs(pSec - val) < 0.05) {
        pill.classList.add('active');
      } else {
        pill.classList.remove('active');
      }
    });
  }
  if (dom.pillsLungsStep) {
    dom.pillsLungsStep.forEach(pill => {
      const pSec = parseFloat(pill.dataset.sec);
      if (Math.abs(pSec - val) < 0.05) {
        pill.classList.add('active');
      } else {
        pill.classList.remove('active');
      }
    });
  }

  if (notifyHardware) {
    sendCommand({
      cmd: `INTERVAL ${val}`,
      action: 'SET_BLOW_INTERVAL',
      interval: val,
      timestamp: Date.now()
    });
    log(`[CONFIG] ⏱️ Blow cycle duration set to ${val.toFixed(1)}s`, 'info');
  }
}

/**
 * Starts continuous Air Blow / Wind Exhale action (BLOW ON).
 * Moves the head forward, shoots continuous 3D wind puff rings, plays whoosh audio,
 * and sends BLOW ON command to Arduino/ESP32.
 */
function startBlowAction() {
  if (state.activeAnimation && state.activeAnimation !== 'BLOW') {
    stopAnimation();
  }

  state.blowActive = true;
  blowActionActive = true;
  blowStartTime = performance.now();

  if (dom.btnBlowOn) dom.btnBlowOn.classList.add('active', 'blowing');
  if (dom.btnBlowOff) dom.btnBlowOff.classList.remove('active');
  if (dom.btnBlow) dom.btnBlow.classList.add('blowing');
  if (dom.btnPresetBlow) dom.btnPresetBlow.classList.add('blowing');

  if (dom.blowRelayDot) {
    dom.blowRelayDot.className = 'strip-dot relay-on';
  }
  if (dom.blowCycleText) {
    dom.blowCycleText.className = 'strip-label active-on';
    dom.blowCycleText.textContent = `BLOWING (ON) - ${(state.blowInterval || 2.0).toFixed(1)}s`;
  }

  playBlowSound();

  sendCommand({
    cmd: 'BLOW ON',
    action: 'BLOW_ON',
    interval: state.blowInterval || 2.0,
    pan: state.pan,
    tilt: state.tilt,
    timestamp: Date.now()
  });

  log(`[ACTION] 💨 Air Blow turned ON (Cycle: ${(state.blowInterval || 2.0).toFixed(1)}s)`, 'tx');
}

/**
 * Stops Air Blow action (BLOW OFF).
 * Resets 3D wind rings, returns head tilt to neutral,
 * and sends BLOW OFF command to Arduino/ESP32.
 */
function stopBlowAction() {
  state.blowActive = false;
  blowActionActive = false;

  if (dom.btnBlowOn) dom.btnBlowOn.classList.remove('active', 'blowing');
  if (dom.btnBlowOff) {
    dom.btnBlowOff.classList.add('active');
    setTimeout(() => {
      if (dom.btnBlowOff) dom.btnBlowOff.classList.remove('active');
    }, 600);
  }
  if (dom.btnBlow) dom.btnBlow.classList.remove('blowing');
  if (dom.btnPresetBlow) dom.btnPresetBlow.classList.remove('blowing');

  if (dom.blowRelayDot) {
    dom.blowRelayDot.className = 'strip-dot';
  }
  if (dom.blowCycleText) {
    dom.blowCycleText.className = 'strip-label';
    dom.blowCycleText.textContent = 'RELAY: IDLE (OFF)';
  }
  if (dom.blowCycleProgress) {
    dom.blowCycleProgress.style.width = '0%';
  }

  if (blowRings) {
    blowRings.forEach(r => {
      r.material.opacity = 0;
      r.scale.set(1, 1, 1);
      r.position.z = 0;
    });
  }

  if (robotHeadGroup) {
    robotHeadGroup.rotation.x = currentEulerPitch;
  }

  sendCommand({
    cmd: 'BLOW OFF',
    action: 'BLOW_OFF',
    pan: state.pan,
    tilt: state.tilt,
    timestamp: Date.now()
  });

  log('[ACTION] 🛑 Air Blow turned OFF', 'warn');
}

/**
 * Toggle / One-shot trigger compatibility
 */
function triggerBlowAction() {
  if (state.blowActive) {
    stopBlowAction();
  } else {
    startBlowAction();
  }
}

function playBlowSound() {
  try {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (!AudioContext) return;
    const ctx = new AudioContext();

    const bufferSize = Math.floor(ctx.sampleRate * 0.7);
    const buffer = ctx.createBuffer(1, bufferSize, ctx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < bufferSize; i++) {
      data[i] = (Math.random() * 2 - 1) * 0.7;
    }

    const noise = ctx.createBufferSource();
    noise.buffer = buffer;

    const filter = ctx.createBiquadFilter();
    filter.type = 'bandpass';
    filter.frequency.setValueAtTime(800, ctx.currentTime);
    filter.frequency.exponentialRampToValueAtTime(320, ctx.currentTime + 0.6);
    filter.Q.value = 2.8;

    const gain = ctx.createGain();
    gain.gain.setValueAtTime(0.001, ctx.currentTime);
    gain.gain.linearRampToValueAtTime(0.35, ctx.currentTime + 0.12);
    gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.65);

    noise.connect(filter);
    filter.connect(gain);
    gain.connect(ctx.destination);

    noise.start();
    noise.stop(ctx.currentTime + 0.7);
  } catch (e) {}
}

// Master Pulse ON / OFF Loop Handler
function togglePulseLoop(forceState = null) {
  const shouldBeActive = forceState !== null ? forceState : !state.pulseLoopActive;
  state.pulseLoopActive = shouldBeActive;

  if (state.pulseLoopActive) {
    if (dom.btnPulseToggle) {
      dom.btnPulseToggle.classList.add('active-on');
      dom.btnPulseToggle.dataset.active = 'true';
    }
    if (dom.pulseStatusBadge) {
      dom.pulseStatusBadge.textContent = 'PULSE: ON';
      dom.pulseStatusBadge.classList.add('active');
    }
    if (dom.pulseToggleTitle) dom.pulseToggleTitle.textContent = 'PULSE: ON (ACTIVE)';
    if (dom.pulseToggleSub) dom.pulseToggleSub.textContent = 'CLICK TO TURN OFF';

    startPulseInterval();
    log(`[PULSE] ⚡ Continuous pulse activated`, 'tx');

    sendCommand({
      cmd: 'PULSE_LOOP',
      state: 'ON',
      freq: state.pulseFrequency,
      timestamp: Date.now()
    });
  } else {
    stopPulseLoop();
    log('[PULSE] Neck pulse turned OFF', 'warn');

    sendCommand({
      cmd: 'PULSE_LOOP',
      state: 'OFF',
      timestamp: Date.now()
    });
  }
}

function startPulseInterval() {
  if (state.pulseIntervalId) clearInterval(state.pulseIntervalId);
  triggerNeckPulse();
  const intervalMs = Math.max(350, Math.round(1000 / state.pulseFrequency));
  state.pulseIntervalId = setInterval(() => {
    if (state.pulseLoopActive) {
      triggerNeckPulse();
    }
  }, intervalMs);
}

function stopPulseLoop() {
  state.pulseLoopActive = false;
  if (state.pulseIntervalId) {
    clearInterval(state.pulseIntervalId);
    state.pulseIntervalId = null;
  }
  if (dom.btnPulseToggle) {
    dom.btnPulseToggle.classList.remove('active-on');
    dom.btnPulseToggle.dataset.active = 'false';
  }
  if (dom.pulseStatusBadge) {
    dom.pulseStatusBadge.textContent = 'PULSE: OFF';
    dom.pulseStatusBadge.classList.remove('active');
  }
  if (dom.pulseToggleTitle) dom.pulseToggleTitle.textContent = 'PULSE: OFF';
  if (dom.pulseToggleSub) dom.pulseToggleSub.textContent = 'CLICK TO TURN ON';
}

/**
 * Sets Pulse Servo Speed (1: HIGH, 20: MEDIUM, 35: LOW).
 * Synchronizes UI pills and transmits PULSE SPEED <val> to Arduino/ESP32.
 */
function setPulseSpeed(speedVal, notifyHardware = true) {
  let val = parseInt(speedVal, 10);
  if (isNaN(val) || val < 1) val = 20;

  state.pulseSpeed = val;
  try {
    localStorage.setItem('robo_pulse_speed', val.toString());
  } catch {}

  let labelText = 'MEDIUM';
  if (val === 1) labelText = 'HIGH';
  else if (val === 35) labelText = 'LOW';
  else labelText = `${val} ms`;

  if (dom.lblPulseSpeed) {
    dom.lblPulseSpeed.textContent = labelText;
  }

  if (dom.pillsPulseSpeed) {
    dom.pillsPulseSpeed.forEach(pill => {
      const pSpeed = parseInt(pill.dataset.speed, 10);
      if (pSpeed === val) {
        pill.classList.add('active');
      } else {
        pill.classList.remove('active');
      }
    });
  }

  // Adjust live interval if loop active
  if (state.pulseLoopActive) {
    startPulseInterval();
  }

  if (notifyHardware) {
    sendCommand({
      cmd: `PULSE SPEED ${val}`,
      action: 'SET_PULSE_SPEED',
      speed: val,
      timestamp: Date.now()
    });
    log(`[PULSE] 💓 Pulse speed set to ${labelText} (Command: PULSE SPEED ${val})`, 'info');
  }
}

// Master Mouth ON / OFF Actuator Handler (Pin 3 Servo)
function toggleMouth(forceState = null) {
  const shouldBeActive = forceState !== null ? forceState : !state.mouthActive;
  state.mouthActive = shouldBeActive;

  if (state.mouthActive) {
    if (dom.btnMouthToggle) {
      dom.btnMouthToggle.classList.add('active-on');
      dom.btnMouthToggle.dataset.active = 'true';
    }
    if (dom.mouthStatusBadge) {
      dom.mouthStatusBadge.textContent = 'MOUTH: OPEN';
      dom.mouthStatusBadge.classList.add('active');
    }
    if (dom.mouthToggleTitle) dom.mouthToggleTitle.textContent = 'MOUTH: OPEN';
    if (dom.mouthToggleSub) dom.mouthToggleSub.textContent = 'CLICK TO CLOSE (KEY: M)';

    [dom.btnMouthOn, dom.btnQuickMouthOn].forEach(b => {
      if (b) b.classList.add('active');
    });
    [dom.btnMouthOff, dom.btnQuickMouthClose].forEach(b => {
      if (b) b.classList.remove('active');
    });

    playMouthSound(true);
    log('[MOUTH] 👄 Robotic mouth OPENED (Pin 3 open)', 'tx');

    sendCommand({
      cmd: 'MOUTH',
      state: 'ON',
      timestamp: Date.now()
    });
  } else {
    if (dom.btnMouthToggle) {
      dom.btnMouthToggle.classList.remove('active-on');
      dom.btnMouthToggle.dataset.active = 'false';
    }
    if (dom.mouthStatusBadge) {
      dom.mouthStatusBadge.textContent = 'MOUTH: CLOSED';
      dom.mouthStatusBadge.classList.remove('active');
    }
    if (dom.mouthToggleTitle) dom.mouthToggleTitle.textContent = 'MOUTH: CLOSED';
    if (dom.mouthToggleSub) dom.mouthToggleSub.textContent = 'CLICK TO OPEN (MOUTH ON)';

    [dom.btnMouthOn, dom.btnQuickMouthOn].forEach(b => {
      if (b) b.classList.remove('active');
    });
    [dom.btnMouthOff, dom.btnQuickMouthClose].forEach(b => {
      if (b) b.classList.add('active');
    });

    playMouthSound(false);
    log('[MOUTH] 🤐 Robotic mouth CLOSED (Pin 3 closed)', 'warn');

    sendCommand({
      cmd: 'MOUTH',
      state: 'OFF',
      timestamp: Date.now()
    });
  }
}

function playMouthSound(isOpen) {
  try {
    const AudioContext = window.AudioContext || window.webkitAudioContext;
    if (!AudioContext) return;
    const ctx = new AudioContext();
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();

    osc.type = 'sine';
    if (isOpen) {
      osc.frequency.setValueAtTime(320, ctx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(560, ctx.currentTime + 0.08);
    } else {
      osc.frequency.setValueAtTime(480, ctx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(180, ctx.currentTime + 0.08);
    }

    gain.gain.setValueAtTime(0.25, ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.09);

    osc.connect(gain);
    gain.connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.1);
  } catch (e) {}
}

/**
 * Master Food Pipe Actuator Handler.
 * Toggles Food Pipe Open / Closed without affecting the 3D middle face.
 * Sends "FOOD PIPE OPEN" / "FOOD PIPE CLOSE" over Serial.
 * @param {boolean|null} forceState - Optional explicit state (true: open, false: close)
 */
function toggleFoodPipe(forceState = null) {
  const shouldOpen = forceState !== null ? forceState : !state.foodPipeOpen;
  state.foodPipeOpen = shouldOpen;

  if (dom.btnFoodPipe) {
    dom.btnFoodPipe.classList.toggle('active', state.foodPipeOpen);
  }
  if (dom.lblFoodPipeText) {
    dom.lblFoodPipeText.textContent = state.foodPipeOpen ? 'FOOD PIPE: OPEN' : 'FOOD PIPE: CLOSED';
  } else if (dom.btnFoodPipe) {
    dom.btnFoodPipe.textContent = state.foodPipeOpen ? 'FOOD PIPE: OPEN' : 'FOOD PIPE: CLOSED';
  }

  const cmd = state.foodPipeOpen ? 'FOOD PIPE OPEN' : 'FOOD PIPE CLOSE';
  sendSerialData(cmd + '\n');
  sendCommand({
    cmd: 'FOOD_PIPE',
    state: state.foodPipeOpen ? 'OPEN' : 'CLOSE',
    timestamp: Date.now()
  });

  if (state.foodPipeOpen) {
    playMouthSound(true);
    log('[ACTION] 🫁 Food pipe OPENED (Command: FOOD PIPE OPEN)', 'tx');
  } else {
    playMouthSound(false);
    log('[ACTION] 🫁 Food pipe CLOSED (Command: FOOD PIPE CLOSE)', 'warn');
  }
}

// Right Column Pulse & Speed Listeners
if (dom.btnPulseToggle) {
  dom.btnPulseToggle.addEventListener('click', () => {
    togglePulseLoop();
  });
}

if (dom.btnMouthToggle) {
  dom.btnMouthToggle.addEventListener('click', () => {
    toggleMouth();
  });
}

if (dom.btnBlowOn) {
  dom.btnBlowOn.addEventListener('click', () => {
    startBlowAction();
  });
}

if (dom.btnBlowOff) {
  dom.btnBlowOff.addEventListener('click', () => {
    stopBlowAction();
  });
}

// Configurable Blow Interval Controls & Presets
if (dom.sliderBlowInterval) {
  dom.sliderBlowInterval.addEventListener('input', (e) => {
    setBlowInterval(e.target.value, true);
  });
}

if (dom.numBlowInterval) {
  dom.numBlowInterval.addEventListener('change', (e) => {
    setBlowInterval(e.target.value, true);
  });
}

if (dom.btnBlowDec) {
  dom.btnBlowDec.addEventListener('click', () => {
    setBlowInterval((state.blowInterval || 2.0) - 0.5, true);
  });
}

if (dom.btnBlowInc) {
  dom.btnBlowInc.addEventListener('click', () => {
    setBlowInterval((state.blowInterval || 2.0) + 0.5, true);
  });
}

if (dom.pillsBlowPreset) {
  dom.pillsBlowPreset.forEach(pill => {
    pill.addEventListener('click', () => {
      const sec = parseFloat(pill.dataset.sec);
      setBlowInterval(sec, true);
    });
  });
}

if (dom.pillsLungsStep) {
  dom.pillsLungsStep.forEach(pill => {
    pill.addEventListener('click', () => {
      const sec = parseFloat(pill.dataset.sec);
      setBlowInterval(sec, true);
    });
  });
}

// Initialize Blow Interval UI to current saved state
setBlowInterval(state.blowInterval || 2.0, false);

// Pulse Speed Preset Pills Handler (HIGH: 1ms, MEDIUM: 20ms, LOW: 35ms)
if (dom.pillsPulseSpeed) {
  dom.pillsPulseSpeed.forEach(pill => {
    pill.addEventListener('click', () => {
      const spd = parseInt(pill.dataset.speed, 10);
      setPulseSpeed(spd, true);
    });
  });
}

// Initialize Pulse Speed UI to current saved state
setPulseSpeed(state.pulseSpeed || 20, false);

[dom.btnBlow, dom.btnPresetBlow].forEach(btn => {
  if (btn && btn !== dom.btnBlowOn && btn !== dom.btnBlowOff) {
    btn.addEventListener('click', () => {
      triggerBlowAction();
    });
  }
});

if (dom.speedPresets) {
  dom.speedPresets.forEach(pill => {
    pill.addEventListener('click', () => {
      dom.speedPresets.forEach(p => p.classList.remove('active'));
      pill.classList.add('active');
      const speedVal = parseInt(pill.dataset.speed, 10);
      state.speed = speedVal;
      if (dom.speedRange) dom.speedRange.value = speedVal;
      if (dom.speedValue) dom.speedValue.textContent = `${speedVal}x`;
      log(`[CONFIG] Speed set to ${speedVal}x`, 'info');
    });
  });
}

// ============================================================================
// Communication Engine (WebSocket / ROS Bridge / HTTP REST / Sim)
// ============================================================================

dom.btnQuickConnect.addEventListener('click', () => {
  if (state.isConnected || state.isConnecting) {
    disconnect();
  } else {
    connect();
  }
});

// ============================================================================
// Web Serial API (Direct USB Cable to Arduino / ESP32)
// ============================================================================

// Toast notification when user clicks controls while not connected
let toastTimer = null;
function showNotConnectedNotification() {
  if (dom.btnQuickConnect) {
    dom.btnQuickConnect.classList.add('pulse-highlight');
    setTimeout(() => {
      if (dom.btnQuickConnect) dom.btnQuickConnect.classList.remove('pulse-highlight');
    }, 2500);
  }

  let toast = document.getElementById('connToast');
  if (!toast) {
    toast = document.createElement('div');
    toast.id = 'connToast';
    toast.className = 'conn-toast';
    document.body.appendChild(toast);
  }

  toast.innerHTML = `
    <div class="toast-content">
      <span class="toast-icon">⚠️</span>
      <div class="toast-msg">
        <strong>USB MOTOR NOT CONNECTED</strong>
        <span>Click <b>[⚡ CONNECT]</b> in the top right header to connect your Arduino / ESP32!</span>
      </div>
      <button class="toast-btn" id="toastConnectBtn">CONNECT NOW</button>
    </div>
  `;
  toast.classList.add('visible');

  const btn = document.getElementById('toastConnectBtn');
  if (btn) {
    btn.onclick = () => {
      toast.classList.remove('visible');
      connect();
    };
  }

  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    if (toast) toast.classList.remove('visible');
  }, 4500);
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    alert("Web Serial API is available in Google Chrome, Microsoft Edge, and Opera.\nIf you are using another browser, please open this page in Chrome or Edge, or switch to WebSocket in Settings.");
    disconnect();
    return;
  }

  updateConnectionUI('connecting', 'SELECT COM PORT...');
  log(`[SERIAL] Requesting USB Serial Port (${state.baudRate} Baud)...`, 'info');

  try {
    state.serialPort = await navigator.serial.requestPort();
    await state.serialPort.open({ baudRate: state.baudRate });
    state.isConnected = true;
    state.isConnecting = false;
    updateConnectionUI('online', 'USB CONNECTED');
    if (dom.latencyBadge) dom.latencyBadge.textContent = 'USB READY';
    log(`[SERIAL] Connected to USB COM Port at ${state.baudRate} baud`, 'rx');

    const toast = document.getElementById('connToast');
    if (toast) toast.classList.remove('visible');

    // Wait 1.2s for board bootloader reset to finish before first sync
    setTimeout(() => {
      sendSerialData('HOME\n');
      sendSerialData(`PULSE SPEED ${state.pulseSpeed || 20}\n`);
    }, 1200);

    startSerialRead();
  } catch (err) {
    if (err.name === 'NetworkError' || (err.message && err.message.toLowerCase().includes('open'))) {
      alert("⚠️ COM Port is Busy!\n\n1. If the Arduino IDE Serial Monitor is open, CLOSE IT.\n2. Close any duplicate tabs of this page.\n3. If still busy, unplug and replug the USB cable to reset the port, then click CONNECT again.");
    }
    log(`[SERIAL] ${err.message || 'Port selection cancelled'}`, 'err');
    disconnect();
  }
}

// Sequential write chain ensures writes never lock or collide
let serialWriteChain = Promise.resolve();

function sendSerialData(text) {
  if (!state.serialPort || !state.isConnected) {
    log(`[SERIAL OFFLINE] ${text.trim()} (Click CONNECT in top right)`, 'warn');
    showNotConnectedNotification();
    return;
  }

  const dataToSend = text.endsWith('\n') ? text : (text + '\n');

  serialWriteChain = serialWriteChain.then(async () => {
    if (!state.serialPort || !state.serialPort.writable) return;
    try {
      const encoder = new TextEncoder();
      const writer = state.serialPort.writable.getWriter();
      await writer.write(encoder.encode(dataToSend));
      writer.releaseLock();
      log(`[SERIAL TX] ${dataToSend.trim()}`, 'tx');
    } catch (err) {
      log(`[SERIAL TX ERR] ${err.message}`, 'err');
    }
  });
}

async function startSerialRead() {
  while (state.serialPort && state.serialPort.readable && state.isConnected) {
    try {
      const textDecoder = new TextDecoderStream();
      state.serialReadableStreamClosed = state.serialPort.readable.pipeTo(textDecoder.writable);
      state.serialReader = textDecoder.readable.getReader();

      while (true) {
        const { value, done } = await state.serialReader.read();
        if (done) break;
        if (value && value.trim()) {
          state.stats.recv++;
          if (dom.statRecv) dom.statRecv.textContent = state.stats.recv;
          log(`[ARDUINO] ${value.trim()}`, 'rx');

          const upper = value.toUpperCase();
          if (upper.includes('RELAY ON')) {
            if (dom.blowRelayDot) dom.blowRelayDot.className = 'strip-dot relay-on';
            if (dom.blowCycleText && state.blowActive) {
              dom.blowCycleText.className = 'strip-label active-on';
            }
          } else if (upper.includes('RELAY OFF')) {
            if (dom.blowRelayDot) dom.blowRelayDot.className = 'strip-dot relay-off-cycle';
            if (dom.blowCycleText && state.blowActive) {
              dom.blowCycleText.className = 'strip-label active-off';
            }
          } else if (upper.includes('RELAY STOPPED')) {
            if (dom.blowRelayDot) dom.blowRelayDot.className = 'strip-dot';
            if (dom.blowCycleText) {
              dom.blowCycleText.className = 'strip-label';
              dom.blowCycleText.textContent = 'RELAY: IDLE (OFF)';
            }
            if (dom.blowCycleProgress) dom.blowCycleProgress.style.width = '0%';
          }
        }
      }
    } catch (err) {
      if (state.isConnected) {
        log(`[SERIAL READ] ${err.message}`, 'warn');
      }
      break;
    } finally {
      if (state.serialReader) {
        try { state.serialReader.releaseLock(); } catch {}
      }
    }
  }
}

async function disconnectSerial() {
  state.isConnected = false;
  state.isConnecting = false;
  if (state.serialReader) {
    try { await state.serialReader.cancel(); } catch {}
    state.serialReader = null;
  }
  if (state.serialReadableStreamClosed) {
    try { await state.serialReadableStreamClosed.catch(() => {}); } catch {}
    state.serialReadableStreamClosed = null;
  }
  if (state.serialPort) {
    try { await state.serialPort.close(); } catch {}
    state.serialPort = null;
  }
  updateConnectionUI('offline', 'DISCONNECTED');
  if (dom.latencyBadge) dom.latencyBadge.textContent = 'PING: -- ms';
}

function connect() {
  updateConnectionUI('connecting', 'CONNECTING...');

  if (state.protocol === 'serial') {
    connectSerial();
    return;
  }

  if (state.protocol === 'simulation') {
    setTimeout(() => {
      state.isConnected = true;
      updateConnectionUI('online', 'SIMULATION READY');
      log('[SYSTEM] Connected to Local Virtual Robot Simulator.', 'rx');
    }, 400);
    return;
  }

  if (state.protocol === 'websocket' || state.protocol === 'ros') {
    const targetUrl = state.protocol === 'ros' ? state.rosWsUrl : state.wsUrl;
    log(`[NET] Initiating connection to ${targetUrl}...`, 'info');

    try {
      state.socket = new WebSocket(targetUrl);

      state.socket.onopen = () => {
        state.isConnected = true;
        state.isConnecting = false;
        updateConnectionUI('online', 'CONNECTED');
        log(`[NET] Connected successfully to ${targetUrl}`, 'rx');
        startPingTimer();

        // If ROS, advertise topic
        if (state.protocol === 'ros') {
          const advertiseMsg = {
            op: 'advertise',
            topic: state.rosTopic,
            type: 'geometry_msgs/Twist'
          };
          state.socket.send(JSON.stringify(advertiseMsg));
          log(`[ROS] Advertised topic: ${state.rosTopic}`, 'info');
        }
      };

      state.socket.onmessage = (event) => {
        state.stats.recv++;
        if (dom.statRecv) dom.statRecv.textContent = state.stats.recv;

        try {
          const data = JSON.parse(event.data);
          if (data.type === 'pong' || data.cmd === 'PONG') {
            const rtt = Date.now() - state.lastPingTime;
            state.latency = rtt;
            if (dom.latencyBadge) dom.latencyBadge.textContent = `PING: ${rtt} ms`;
            return;
          }
          log(`[RX] ${typeof data === 'object' ? JSON.stringify(data) : event.data}`, 'rx');
        } catch {
          log(`[RX] ${event.data}`, 'rx');
        }
      };

      state.socket.onerror = (err) => {
        log(`[NET] WebSocket error: unable to reach host`, 'err');
      };

      state.socket.onclose = () => {
        disconnect();
        log(`[NET] Connection closed`, 'warn');
      };
    } catch (err) {
      disconnect();
      log(`[NET] Connection error: ${err.message}`, 'err');
    }
  } else if (state.protocol === 'http') {
    // HTTP Mode is stateless
    state.isConnected = true;
    updateConnectionUI('online', 'HTTP READY');
    log(`[NET] HTTP Endpoint configured: ${state.httpUrl}`, 'rx');
  }
}

function disconnect() {
  if (state.protocol === 'serial') {
    disconnectSerial();
    return;
  }

  state.isConnected = false;
  state.isConnecting = false;
  if (state.socket) {
    try { state.socket.close(); } catch {}
    state.socket = null;
  }
  if (state.pingInterval) {
    clearInterval(state.pingInterval);
    state.pingInterval = null;
  }
  updateConnectionUI('offline', 'DISCONNECTED');
  if (dom.latencyBadge) dom.latencyBadge.textContent = 'PING: -- ms';
}

function startPingTimer() {
  if (state.pingInterval) clearInterval(state.pingInterval);
  state.pingInterval = setInterval(() => {
    if (state.isConnected && state.socket && state.socket.readyState === WebSocket.OPEN) {
      state.lastPingTime = Date.now();
      state.socket.send(JSON.stringify({ type: 'ping', t: state.lastPingTime }));
    }
  }, 3000);
}

/**
 * Transmits a command packet according to current protocol.
 */
let lastHttpDispatch = 0;
function sendCommand(payload) {
  state.stats.sent++;
  if (dom.statSent) dom.statSent.textContent = state.stats.sent;
  pulseTx();

  if (state.protocol === 'serial') {
    if (payload.cmd === 'MOUTH') {
      sendSerialData(payload.state === 'ON' ? 'MOUTH ON\n' : 'MOUTH OFF\n');
    } else if (payload.cmd === 'PULSE_LOOP') {
      sendSerialData(payload.state === 'ON' ? 'PULSE ON\n' : 'PULSE OFF\n');
    } else if (payload.cmd === 'PULSE') {
      sendSerialData('PULSE ON\n');
      setTimeout(() => sendSerialData('PULSE OFF\n'), 800);
    } else if (payload.action === 'SET_PULSE_SPEED' || (payload.cmd && payload.cmd.startsWith('PULSE SPEED'))) {
      const spd = payload.speed !== undefined ? payload.speed : (state.pulseSpeed || 20);
      sendSerialData(`PULSE SPEED ${spd}\n`);
    } else if (payload.cmd === 'BLOW' || payload.cmd === 'BLOW ON' || payload.action === 'BLOW_ON') {
      const sec = payload.interval || state.blowInterval || 2.0;
      sendSerialData(`INTERVAL ${sec}\n1\n`);
    } else if (payload.cmd === 'BLOW OFF' || payload.action === 'BLOW_OFF') {
      sendSerialData('0\n');
      setTimeout(() => {
        sendSerialData('MOUTH OFF\nPULSE OFF\n');
      }, 50);
    } else if (payload.action === 'SET_BLOW_INTERVAL' || (payload.cmd && payload.cmd.startsWith('INTERVAL'))) {
      const sec = payload.interval || state.blowInterval || 2.0;
      sendSerialData(`INTERVAL ${sec}\n`);
    } else if (payload.cmd === 'ESTOP') {
      sendSerialData('ESTOP\n');
    } else if (payload.action === 'UP') {
      sendSerialData('UP\n');
    } else if (payload.action === 'DOWN') {
      sendSerialData('DOWN\n');
    } else if (payload.action === 'LEFT') {
      sendSerialData('LEFT\n');
    } else if (payload.action === 'RIGHT') {
      sendSerialData('RIGHT\n');
    } else if (payload.action === 'CENTER' || payload.action === 'HOME') {
      sendSerialData('HOME\n');
    } else if (payload.action === 'HAND_TRACK') {
      const now = performance.now();
      if (now - handTrack.lastSerialDispatch >= 50) { // 20 Hz throttle
        handTrack.lastSerialDispatch = now;
        if (state.isConnected) {
          sendSerialData(`TILT ${payload.tilt}\nPAN ${payload.pan}\n`);
        }
      }
    } else if (payload.action === 'TILT_SLIDER' || payload.action === 'SWEEP_TILT' || payload.action === 'NOD') {
      sendSerialData(`TILT ${payload.tilt}\n`);
    } else if (payload.action === 'PAN_SLIDER' || payload.action === 'SWEEP_PAN' || payload.action === 'SHAKE') {
      sendSerialData(`PAN ${payload.pan}\n`);
    } else if (payload.raw) {
      sendSerialData(payload.raw.endsWith('\n') ? payload.raw : payload.raw + '\n');
    } else {
      sendSerialData(`${payload.action || payload.cmd || payload.tilt}\n`);
    }
    return;
  }

  if (state.protocol === 'websocket') {
    if (state.socket && state.socket.readyState === WebSocket.OPEN) {
      const jsonStr = JSON.stringify(payload);
      state.socket.send(jsonStr);
      log(`[TX] ${jsonStr}`, 'tx');
    } else {
      state.stats.dropped++;
      if (dom.statDropped) dom.statDropped.textContent = state.stats.dropped;
      log(`[TX-OFFLINE] ${JSON.stringify(payload)}`, 'tx');
    }
  } else if (state.protocol === 'ros') {
    // ROSBridge Twist / Head message formatting
    const rosMsg = {
      op: 'publish',
      topic: state.rosTopic,
      msg: {
        linear: { x: payload.pan, y: payload.tilt, z: 0 },
        angular: { x: payload.relYaw, y: payload.relPitch, z: 0 }
      }
    };
    if (state.socket && state.socket.readyState === WebSocket.OPEN) {
      state.socket.send(JSON.stringify(rosMsg));
      log(`[ROS-TX] /cmd => pan:${payload.pan} tilt:${payload.tilt}`, 'tx');
    } else {
      state.stats.dropped++;
      if (dom.statDropped) dom.statDropped.textContent = state.stats.dropped;
      log(`[ROS-OFFLINE] pan:${payload.pan} tilt:${payload.tilt}`, 'tx');
    }
  } else if (state.protocol === 'http') {
    // Debounce rapid HTTP calls to 50ms to prevent socket exhaustion
    const now = Date.now();
    if (now - lastHttpDispatch < 50) return;
    lastHttpDispatch = now;

    let targetUrl = state.httpUrl
      .replace('{pan}', payload.pan)
      .replace('{tilt}', payload.tilt)
      .replace('{dir}', payload.action || 'MOVE');

    log(`[HTTP ${state.httpMethod}] ${targetUrl}`, 'tx');

    if (state.httpMethod === 'GET') {
      fetch(targetUrl, { mode: 'no-cors' }).catch(() => {});
    } else {
      fetch(targetUrl, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
        mode: 'no-cors'
      }).catch(() => {});
    }
  } else if (state.protocol === 'simulation') {
    log(`[SIM] Head moved -> Pan:${payload.pan}° Tilt:${payload.tilt}° [${payload.action}]`, 'tx');
  }
}

function updateConnectionUI(status, label) {
  dom.connIndicator.className = `status-indicator ${status}`;
  dom.connStatusText.textContent = label;

  if (status === 'online') {
    dom.btnQuickConnect.textContent = 'DISCONNECT';
    dom.btnQuickConnect.classList.add('connected');
  } else if (status === 'connecting') {
    dom.btnQuickConnect.textContent = 'CONNECTING...';
    dom.btnQuickConnect.classList.remove('connected');
  } else {
    dom.btnQuickConnect.textContent = 'CONNECT USB';
    dom.btnQuickConnect.classList.remove('connected');
  }
}

function pulseTx() {
  if (dom.txStatus) {
    dom.txStatus.textContent = 'TRANSMITTING';
    dom.txStatus.classList.add('transmitting');
    setTimeout(() => {
      dom.txStatus.textContent = 'IDLE';
      dom.txStatus.classList.remove('transmitting');
    }, 120);
  }
}

// Logs modal handlers
if (dom.btnOpenLogs && dom.logsModal) {
  dom.btnOpenLogs.addEventListener('click', () => {
    dom.logsModal.style.display = 'flex';
  });
}

if (dom.btnCloseLogs && dom.logsModal) {
  dom.btnCloseLogs.addEventListener('click', () => {
    dom.logsModal.style.display = 'none';
  });
}

// Close modals on backdrop click
[dom.settingsModal, dom.logsModal, dom.adminModal].forEach(modal => {
  if (modal) {
    modal.addEventListener('click', (e) => {
      if (e.target === modal) {
        modal.style.display = 'none';
      }
    });
  }
});

// ============================================================================
// Admin Panel Handlers (Triggered by double-tapping Header Left Pulse Ring)
// ============================================================================
function openAdminPanel() {
  if (!dom.adminModal) return;
  dom.adminModal.classList.add('active');
  dom.adminModal.style.setProperty('display', 'flex', 'important');
  if (dom.headerLogoBadge) {
    dom.headerLogoBadge.classList.add('pulse-tapped');
    setTimeout(() => {
      if (dom.headerLogoBadge) dom.headerLogoBadge.classList.remove('pulse-tapped');
    }, 450);
  }
  log('[ADMIN] 🛡️ Admin Panel opened via Header Pulse Ring double-tap (Root Access)', 'warn');
}

function closeAdminPanel() {
  if (dom.adminModal) {
    dom.adminModal.classList.remove('active');
    dom.adminModal.style.setProperty('display', 'none', 'important');
    log('[ADMIN] 🛩️ Returned to 3D Cockpit Mission Control', 'info');
  }
}

// Double-tap & Double-click on Header Left Logo Badge / Pulse Ring
let lastLogoTap = 0;
if (dom.headerLogoBadge) {
  dom.headerLogoBadge.addEventListener('dblclick', (e) => {
    e.preventDefault();
    openAdminPanel();
  });

  // Touch & pointer double-tap (works on touchscreen & mouse within 420ms)
  dom.headerLogoBadge.addEventListener('pointerdown', (e) => {
    const now = performance.now();
    if (now - lastLogoTap < 420) {
      e.preventDefault();
      openAdminPanel();
      lastLogoTap = 0;
    } else {
      lastLogoTap = now;
    }
  });
}

// Close Admin Panel Buttons (Back to Cockpit / Close triggers)
[dom.btnCloseAdmin, dom.btnCloseAdminFooter, dom.btnBackFromAdmin].forEach(btn => {
  if (btn) {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      e.stopPropagation();
      closeAdminPanel();
    });
  }
});

// Escape key listener to close full-page admin overlay
window.addEventListener('keydown', (e) => {
  if (e.key === 'Escape' && dom.adminModal && (dom.adminModal.classList.contains('active') || dom.adminModal.style.display !== 'none')) {
    e.preventDefault();
    closeAdminPanel();
  }
});

// Admin Direct Actuator Testing Buttons
document.querySelectorAll('.btn-admin-action').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.dataset.cmd;
    if (cmd) {
      sendSerialData(cmd + '\n');
      log(`[ADMIN DIRECT] ⚙️ Sent command: ${cmd}`, 'tx');
    }
  });
});

// Admin Quick Inject Command Chips
document.querySelectorAll('.btn-chip-inject').forEach(chip => {
  chip.addEventListener('click', () => {
    if (dom.adminCmdInput) {
      dom.adminCmdInput.value = chip.dataset.val;
      dom.adminCmdInput.focus();
    }
  });
});

// Admin Raw Serial Command Dispatcher
if (dom.btnAdminSendCmd && dom.adminCmdInput) {
  const sendAdminRaw = () => {
    const raw = dom.adminCmdInput.value.trim();
    if (!raw) return;
    sendSerialData(raw + '\n');
    log(`[ADMIN RAW] 📡 Dispatched: ${raw}`, 'tx');
    dom.adminCmdInput.value = '';
  };

  dom.btnAdminSendCmd.addEventListener('click', sendAdminRaw);
  dom.adminCmdInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') sendAdminRaw();
  });
}

// Admin Navigation Quick Links
if (dom.btnAdminOpenSettings) {
  dom.btnAdminOpenSettings.addEventListener('click', () => {
    closeAdminPanel();
    if (dom.settingsModal) dom.settingsModal.style.display = 'flex';
  });
}

if (dom.btnAdminOpenLogs) {
  dom.btnAdminOpenLogs.addEventListener('click', () => {
    closeAdminPanel();
    if (dom.logsModal) dom.logsModal.style.display = 'flex';
  });
}

// Manual command sender
if (dom.btnSendCustom) {
  dom.btnSendCustom.addEventListener('click', sendCustomCommand);
}
if (dom.customCmdInput) {
  dom.customCmdInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') sendCustomCommand();
  });
}

function sendCustomCommand() {
  if (!dom.customCmdInput) return;
  const text = dom.customCmdInput.value.trim();
  if (!text) return;
  try {
    const parsed = JSON.parse(text);
    sendCommand(parsed);
  } catch {
    sendCommand({ raw: text, timestamp: Date.now() });
  }
  dom.customCmdInput.value = '';
}

// Terminal Logging
function log(msg, type = 'info') {
  console.log(`[${type.toUpperCase()}]`, msg);

  if (!dom.terminalLog) return;
  const entry = document.createElement('div');
  entry.className = `log-entry log-${type}`;

  const timeStr = new Date().toTimeString().split(' ')[0];
  entry.innerHTML = `<span class="log-time">[${timeStr}]</span> <span>${escapeHtml(msg)}</span>`;

  dom.terminalLog.appendChild(entry);
  dom.terminalLog.scrollTop = dom.terminalLog.scrollHeight;

  // Prune history
  if (dom.terminalLog.children.length > 150) {
    dom.terminalLog.removeChild(dom.terminalLog.firstChild);
  }
}

if (dom.btnClearLog && dom.terminalLog) {
  dom.btnClearLog.addEventListener('click', () => {
    dom.terminalLog.innerHTML = '';
  });
}

function escapeHtml(text) {
  const div = document.createElement('div');
  div.innerText = text;
  return div.innerHTML;
}

// ============================================================================
// Three.js 3D Interactive Wireframe Head Visualizer
// ============================================================================

let scene, camera, renderer;
let robotHeadGroup, neckPivot, eyeMesh, visorLight, skullMesh, faceMesh;
let upperFaceMesh, jawPivot, jawMesh, mouthInteriorMesh;
const MOUTH_SEAM_Y = -0.6041; // Exact vertical Y level of lip parting line
let currentMouthOpenProgress = 0.0;
let neckPulseRing, neckCoilRing, neckPulseLight;
let neckPulseActive = false, neckPulseStartTime = 0;
const NECK_PULSE_DURATION = 1400; // ms
let blowActionActive = false, blowStartTime = 0;
const BLOW_DURATION = 950; // ms
let blowRingGroup, blowRings = [];
let targetEulerYaw = 0;
let targetEulerPitch = 0;
let currentEulerYaw = 0;
let currentEulerPitch = 0;

function initThreeJS() {
  const container = dom.threeContainer;
  const width = container.clientWidth || 400;
  const height = container.clientHeight || 340;

  // 1. Scene
  scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x04060a, 0.035);

  // 2. Camera positioned to frame the 3D floating wireframe head
  camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
  camera.position.set(0, 0.45, 4.3);
  camera.lookAt(0, 0.25, 0);

  // 3. Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  // 4. Lighting
  const ambientLight = new THREE.AmbientLight(0x667799, 1.4);
  scene.add(ambientLight);

  const keyLight = new THREE.DirectionalLight(0x00f0ff, 1.6);
  keyLight.position.set(3, 4, 3);
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0xff0055, 1.2);
  fillLight.position.set(-3, 2, -2);
  scene.add(fillLight);

  // 5. Grid Ground
  const gridHelper = new THREE.GridHelper(6, 20, 0x00f0ff, 0x152238);
  gridHelper.position.y = -1.25;
  scene.add(gridHelper);

  // 6. Build High-Tech 3D Wireframe Head Model (No Neck)
  buildRobotModel();

  // 7. Interactive orbit drag on 3D view
  setupViewDragging(container);

  // 8. Resize Listener
  window.addEventListener('resize', onWindowResize);

  // 9. Animation Loop
  animate();
}

function buildRobotModel() {
  // Base Holographic Emitter Pedestal (Ground level, below the floating head - no neck)
  const pedestalGeo = new THREE.CylinderGeometry(0.85, 1.05, 0.1, 32);
  const pedestalMat = new THREE.MeshStandardMaterial({
    color: 0x0c1322,
    metalness: 0.85,
    roughness: 0.3
  });
  const pedestal = new THREE.Mesh(pedestalGeo, pedestalMat);
  pedestal.position.y = -1.2;
  scene.add(pedestal);

  // Outer glowing holographic base ring
  const ringGeo = new THREE.TorusGeometry(0.95, 0.02, 16, 64);
  const ringMat = new THREE.MeshBasicMaterial({
    color: 0x00f0ff,
    transparent: true,
    opacity: 0.5
  });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = Math.PI / 2;
  ring.position.y = -1.14;
  scene.add(ring);

  // Holographic Floating Energy Rings for Pulse Waves (positioned cleanly below the chin)
  const neckPulseRingGeo = new THREE.TorusGeometry(0.65, 0.025, 16, 48);
  const neckPulseRingMat = new THREE.MeshBasicMaterial({
    color: 0x00f0ff,
    transparent: true,
    opacity: 0.45
  });
  neckPulseRing = new THREE.Mesh(neckPulseRingGeo, neckPulseRingMat);
  neckPulseRing.rotation.x = Math.PI / 2;
  neckPulseRing.position.set(0, -1.12, 0);
  scene.add(neckPulseRing);

  const neckCoilGeo = new THREE.TorusGeometry(0.48, 0.02, 16, 48);
  const neckCoilMat = new THREE.MeshBasicMaterial({
    color: 0xff0055,
    transparent: true,
    opacity: 0.35
  });
  neckCoilRing = new THREE.Mesh(neckCoilGeo, neckCoilMat);
  neckCoilRing.rotation.x = Math.PI / 2;
  neckCoilRing.position.set(0, -1.05, 0);
  scene.add(neckCoilRing);

  // Flaring Pulse Energy Light
  neckPulseLight = new THREE.PointLight(0x00f0ff, 0.8, 3.2);
  neckPulseLight.position.set(0, -1.05, 0);
  scene.add(neckPulseLight);

  // Pan / Yaw Pivot Group (Rotates Left / Right)
  neckPivot = new THREE.Group();
  neckPivot.position.set(0, 0, 0);
  scene.add(neckPivot);

  // Head Pitch / Tilt Pivot Group (Rotates Up / Down)
  robotHeadGroup = new THREE.Group();
  robotHeadGroup.position.set(0, 0, 0);
  neckPivot.add(robotHeadGroup);

  // -------------------------------------------------------------------------
  // 3D Curved Face Mesh with User-Uploaded Wireframe Head Texture (No Neck)
  // -------------------------------------------------------------------------
  const textureLoader = new THREE.TextureLoader();
  const faceTextureSrc = (typeof window !== 'undefined' && window.HEAD_IMAGE_DATA)
    ? window.HEAD_IMAGE_DATA
    : (typeof HEAD_IMAGE_DATA !== 'undefined' && HEAD_IMAGE_DATA ? HEAD_IMAGE_DATA : 'head_mesh.png');
  const faceTexture = textureLoader.load(faceTextureSrc);
  faceTexture.minFilter = THREE.LinearFilter;
  faceTexture.magFilter = THREE.LinearFilter;

  // Articulated 3D Curved Face & Robotic Jaw with User Texture
  const faceWidth = 2.05;
  const faceHeight = 2.68;
  // Natural lip seam: exactly between upper lip and lower lip (y=293 in orig, y=280 in mesh)
  const splitRatio = 0.1813; // 18.13% from bottom (exact lip parting line)
  const jawHeight = faceHeight * splitRatio;           // ~0.4859 (lower lip & chin)
  const upperHeight = faceHeight * (1.0 - splitRatio); // ~2.1941 (forehead, eyes, nose & upper lip)
  const halfW = faceWidth / 2;
  const totalHalfH = faceHeight / 2; // 1.34
  const seamY = MOUTH_SEAM_Y; // -0.6041
  const upperCenterY = seamY + upperHeight / 2;
  const jawCenterY = -jawHeight / 2;

  const faceMat = new THREE.MeshBasicMaterial({
    map: faceTexture,
    transparent: true,
    alphaTest: 0.05,
    side: THREE.DoubleSide
  });

  // 1. Upper Face Mesh (forehead, eyes, nose, and UPPER LIP)
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
  faceMesh = upperFaceMesh; // Alias for legacy safety

  // 2. Articulated Lower Jaw Group (LOWER LIP and chin)
  jawPivot = new THREE.Group();
  jawPivot.position.set(0, seamY, 0); // Pivot hinge at exact lip seam level
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

  jawMesh = new THREE.Mesh(jawGeo, faceMat);
  jawMesh.position.set(0, jawCenterY, 0);
  jawPivot.add(jawMesh);

  // 3. Inner Mouth Cavity (dark oral interior revealed when mouth opens)
  const cavityGeo = new THREE.PlaneGeometry(0.70, 0.35);
  const cavityMat = new THREE.MeshBasicMaterial({
    color: 0x05070c,
    side: THREE.DoubleSide
  });
  mouthInteriorMesh = new THREE.Mesh(cavityGeo, cavityMat);
  mouthInteriorMesh.position.set(0, seamY - 0.04, 0.26);
  robotHeadGroup.add(mouthInteriorMesh);

  // 3D Wireframe Skull / Cranium Oval Cage with Front-Side Transparency
  const skullGeo = new THREE.SphereGeometry(0.96, 30, 22);
  skullGeo.scale(0.88, 1.25, 0.98);

  const skullMat = new THREE.ShaderMaterial({
    wireframe: true,
    transparent: true,
    depthWrite: false,
    uniforms: {
      uColor: { value: new THREE.Color(0x4d88b0) },
      uMaxOpacity: { value: 0.28 }
    },
    vertexShader: [
      'varying vec3 vLocalPos;',
      'void main() {',
      '  vLocalPos = position;',
      '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
      '}'
    ].join('\n'),
    fragmentShader: [
      'uniform vec3 uColor;',
      'uniform float uMaxOpacity;',
      'varying vec3 vLocalPos;',
      'void main() {',
      '  // Front side of oval (+Z) is completely transparent so it never obscures the face',
      '  float alpha = uMaxOpacity;',
      '  if (vLocalPos.z > -0.15) {',
      '    alpha = mix(uMaxOpacity, 0.0, clamp((vLocalPos.z + 0.15) / 0.35, 0.0, 1.0));',
      '  }',
      '  if (alpha <= 0.005) discard;',
      '  gl_FragColor = vec4(uColor, alpha);',
      '}'
    ].join('\n')
  });
  skullMesh = new THREE.Mesh(skullGeo, skullMat);
  skullMesh.position.set(0, 0.25, -0.22);
  robotHeadGroup.add(skullMesh);

  // Subtle interior cyan holographic glow inside the wireframe head
  visorLight = new THREE.PointLight(0x00f0ff, 1.2, 3.5);
  visorLight.position.set(0, 0.25, 0.3);
  robotHeadGroup.add(visorLight);

  // 3D Air Blow / Wind Puff Effect Ring Group (positioned in front of lips)
  blowRingGroup = new THREE.Group();
  blowRingGroup.position.set(0, MOUTH_SEAM_Y + 0.02, 0.44);
  robotHeadGroup.add(blowRingGroup);

  blowRings = [];
  for (let i = 0; i < 3; i++) {
    const bGeo = new THREE.TorusGeometry(0.12 + i * 0.05, 0.016, 12, 36);
    const bMat = new THREE.MeshBasicMaterial({
      color: 0x88ffff,
      transparent: true,
      opacity: 0,
      side: THREE.DoubleSide
    });
    const bMesh = new THREE.Mesh(bGeo, bMat);
    bMesh.position.set(0, 0, 0);
    blowRingGroup.add(bMesh);
    blowRings.push(bMesh);
  }
}

function setupViewDragging(container) {
  let isDragging = false;
  let prevMouseX = 0;
  let prevMouseY = 0;
  let camRadius = 4.3;
  let camTheta = 0;
  let camPhi = 0.15;

  container.addEventListener('mousedown', (e) => {
    isDragging = true;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;
  });

  window.addEventListener('mousemove', (e) => {
    if (!isDragging) return;
    const deltaX = e.clientX - prevMouseX;
    const deltaY = e.clientY - prevMouseY;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;

    camTheta -= deltaX * 0.008;
    camPhi = Math.max(-0.4, Math.min(Math.PI / 2 - 0.1, camPhi + deltaY * 0.008));

    updateCameraOrbit();
  });

  window.addEventListener('mouseup', () => { isDragging = false; });

  container.addEventListener('wheel', (e) => {
    e.preventDefault();
    camRadius = Math.max(2.0, Math.min(6.5, camRadius + e.deltaY * 0.003));
    updateCameraOrbit();
  }, { passive: false });

  function updateCameraOrbit() {
    camera.position.x = camRadius * Math.sin(camTheta) * Math.cos(camPhi);
    camera.position.y = 0.25 + camRadius * Math.sin(camPhi);
    camera.position.z = camRadius * Math.cos(camTheta) * Math.cos(camPhi);
    camera.lookAt(0, 0.25, 0);
  }
}

function onWindowResize() {
  if (!renderer || !camera || !dom.threeContainer) return;
  const width = dom.threeContainer.clientWidth;
  const height = dom.threeContainer.clientHeight;
  if (width === 0 || height === 0) return;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

let lastFrameTime = performance.now();
let frameCount = 0;

function animate() {
  requestAnimationFrame(animate);

  // FPS Counter
  frameCount++;
  const now = performance.now();
  if (now - lastFrameTime >= 1000) {
    if (dom.fpsBadge) dom.fpsBadge.textContent = `${frameCount} FPS`;
    frameCount = 0;
    lastFrameTime = now;
  }

  // Calculate target rotation angles (converting 0-180 servo degrees into radians)
  // Yaw: 90 is center (0 rad). Left (0) turns left, Right (180) turns right
  targetEulerYaw = ((state.pan - 90) * (Math.PI / 180));
  // Pitch: 90 is center (0 rad). Up (180) tilts head up, Down (0) tilts head down
  targetEulerPitch = -((state.tilt - 90) * (Math.PI / 180)) * 0.8;

  // Smooth damping / interpolation towards target angles
  const lerpFactor = 0.18;
  currentEulerYaw += (targetEulerYaw - currentEulerYaw) * lerpFactor;
  currentEulerPitch += (targetEulerPitch - currentEulerPitch) * lerpFactor;

  if (neckPivot) {
    neckPivot.rotation.y = currentEulerYaw;
  }
  if (robotHeadGroup) {
    robotHeadGroup.rotation.x = currentEulerPitch;
  }

  // Handle Dynamic Holographic Energy Pulse Animation
  if (neckPulseActive) {
    const elapsed = now - neckPulseStartTime;
    const progress = elapsed / NECK_PULSE_DURATION;

    if (progress >= 1) {
      neckPulseActive = false;
      if (robotHeadGroup) robotHeadGroup.position.y = 0;
      if (neckPulseRing) {
        neckPulseRing.scale.set(1, 1, 1);
        neckPulseRing.position.y = -1.12;
        neckPulseRing.material.opacity = 0.45;
        neckPulseRing.material.color.setHex(0x00f0ff);
      }
      if (neckCoilRing) {
        neckCoilRing.scale.set(1, 1, 1);
        neckCoilRing.position.y = -1.05;
        neckCoilRing.material.opacity = 0.35;
      }
      if (neckPulseLight) neckPulseLight.intensity = 0.8;
      if (skullMesh) {
        if (skullMesh.material.uniforms && skullMesh.material.uniforms.uMaxOpacity) {
          skullMesh.material.uniforms.uMaxOpacity.value = 0.28;
        } else {
          skullMesh.material.opacity = 0.28;
        }
      }
    } else {
      // 3 rhythmic energetic wave pulses across duration
      const wavePhase = progress * Math.PI * 6;
      const wave = Math.sin(wavePhase);
      const envelope = Math.sin(progress * Math.PI); // Smooth envelope ramp in and out

      // 1. Subtle kinetic harmonic recoil on head position
      if (robotHeadGroup) {
        robotHeadGroup.position.y = wave * 0.04 * envelope;
      }

      // 2. Expansion & upward surge of energy pulse ring towards the head
      if (neckPulseRing) {
        const ringScale = 1.0 + Math.max(0, wave) * 0.5 * envelope;
        neckPulseRing.scale.set(ringScale, ringScale, 1.0);
        neckPulseRing.position.y = -1.12 + ((progress * 2.5) % 1) * 0.35;
        neckPulseRing.material.opacity = 0.35 + Math.abs(wave) * 0.5;
        neckPulseRing.material.color.setHex(wave > 0.4 ? 0xffffff : 0x00f0ff);
      }

      if (neckCoilRing) {
        const coilScale = 1.0 + Math.max(0, -wave) * 0.4 * envelope;
        neckCoilRing.scale.set(coilScale, coilScale, 1.0);
        neckCoilRing.position.y = -1.05 + Math.cos(wavePhase) * 0.06;
        neckCoilRing.material.opacity = 0.25 + Math.abs(wave) * 0.6;
      }

      // 3. Flaring light from the holographic base
      if (neckPulseLight) {
        neckPulseLight.intensity = 0.8 + Math.abs(wave) * 4.5 * envelope;
      }

      // 4. Wireframe skull cranium glow intensification (front side remains transparent)
      if (skullMesh) {
        const pulseAlpha = 0.28 + Math.abs(wave) * 0.45 * envelope;
        if (skullMesh.material.uniforms && skullMesh.material.uniforms.uMaxOpacity) {
          skullMesh.material.uniforms.uMaxOpacity.value = pulseAlpha;
        } else {
          skullMesh.material.opacity = pulseAlpha;
        }
      }
    }
  }

  // Subtle breathing pulse on interior light
  if (visorLight) {
    visorLight.intensity = 1.2 + Math.sin(now * 0.004) * 0.4;
  }

  // Handle Dynamic Air Blow / Wind Puff Animation & Active Relay Cycle
  if (blowActionActive) {
    const elapsed = now - blowStartTime;
    const intervalMs = (state.blowInterval || 2.0) * 1000;
    const totalCycleMs = intervalMs * 2;

    if (!state.blowActive && elapsed >= intervalMs) {
      blowActionActive = false;
      blowRings.forEach(r => {
        r.material.opacity = 0;
        r.scale.set(1, 1, 1);
        r.position.z = 0;
      });
      if (robotHeadGroup) {
        robotHeadGroup.rotation.x = currentEulerPitch;
      }
      if (dom.blowRelayDot) dom.blowRelayDot.className = 'strip-dot';
      if (dom.blowCycleText) {
        dom.blowCycleText.className = 'strip-label';
        dom.blowCycleText.textContent = 'RELAY: IDLE (OFF)';
      }
      if (dom.blowCycleProgress) dom.blowCycleProgress.style.width = '0%';
    } else {
      const cycleElapsed = elapsed % totalCycleMs;
      const isBlowPhase = cycleElapsed < intervalMs;
      const phaseElapsed = isBlowPhase ? cycleElapsed : (cycleElapsed - intervalMs);
      const phaseProgress = phaseElapsed / intervalMs;
      const phaseRemaining = ((intervalMs - phaseElapsed) / 1000).toFixed(1);

      // Update live status strip and meter
      if (dom.blowCycleProgress) {
        dom.blowCycleProgress.style.width = `${Math.min(100, Math.round(phaseProgress * 100))}%`;
      }

      if (isBlowPhase) {
        if (dom.blowRelayDot && !dom.blowRelayDot.classList.contains('relay-on')) {
          dom.blowRelayDot.className = 'strip-dot relay-on';
        }
        if (dom.blowCycleText) {
          dom.blowCycleText.className = 'strip-label active-on';
          dom.blowCycleText.textContent = `BLOWING (ON) - ${phaseRemaining}s`;
        }

        const wave = Math.sin(phaseProgress * Math.PI);
        if (robotHeadGroup) {
          robotHeadGroup.rotation.x = currentEulerPitch - wave * 0.14;
        }

        // Air blow rings expand and travel outward from the mouth (+Z)
        const ringLoopDur = Math.min(800, intervalMs);
        const ringProg = (phaseElapsed % ringLoopDur) / ringLoopDur;
        blowRings.forEach((r, idx) => {
          const rProg = Math.max(0, Math.min(1, (ringProg * 1.5) - (idx * 0.22)));
          const rScale = 1.0 + rProg * 3.5;
          r.scale.set(rScale, rScale, 1.0);
          r.position.z = rProg * 1.7;
          r.material.opacity = Math.sin(rProg * Math.PI) * 0.75;
        });
      } else {
        if (dom.blowRelayDot && !dom.blowRelayDot.classList.contains('relay-off-cycle')) {
          dom.blowRelayDot.className = 'strip-dot relay-off-cycle';
        }
        if (dom.blowCycleText) {
          dom.blowCycleText.className = 'strip-label active-off';
          dom.blowCycleText.textContent = `CYCLE PAUSE (OFF) - ${phaseRemaining}s`;
        }

        blowRings.forEach(r => {
          r.material.opacity = 0;
          r.scale.set(1, 1, 1);
          r.position.z = 0;
        });

        if (robotHeadGroup) {
          robotHeadGroup.rotation.x = currentEulerPitch;
        }
      }
    }
  }

  // Robotic Mouth Actuator 3D Smooth Articulation (Opens on MOUTH ON, Closes on CLOSE)
  const targetMouthOpen = state.mouthActive ? 1.0 : 0.0;
  currentMouthOpenProgress += (targetMouthOpen - currentMouthOpenProgress) * 0.22;

  if (jawPivot) {
    // Smoothly drops and pivots the lower jaw open, revealing inner mouth between the lips
    jawPivot.position.y = MOUTH_SEAM_Y - (currentMouthOpenProgress * 0.16);
    jawPivot.rotation.x = currentMouthOpenProgress * 0.12;
  }

  renderer.render(scene, camera);
}

// ============================================================================
// Settings Modal Handling
// ============================================================================

dom.btnOpenSettings.addEventListener('click', () => {
  dom.txtWsUrl.value = state.wsUrl;
  dom.txtRosTopic.value = state.rosTopic;
  dom.txtRosWsUrl.value = state.rosWsUrl;
  dom.txtHttpUrl.value = state.httpUrl;
  dom.selHttpMethod.value = state.httpMethod;
  dom.txtCamUrl.value = state.camUrl;
  dom.numPanMin.value = state.panMin;
  dom.numPanMax.value = state.panMax;
  dom.numTiltMin.value = state.tiltMin;
  dom.numTiltMax.value = state.tiltMax;

  const radio = dom.settingsModal.querySelector(`input[name="protocolType"][value="${state.protocol}"]`);
  if (radio) radio.checked = true;
  updateProtocolFields();

  dom.settingsModal.style.display = 'flex';
});

[dom.btnCloseSettings, dom.btnCancelSettings].forEach(btn => {
  btn.addEventListener('click', () => {
    dom.settingsModal.style.display = 'none';
  });
});

dom.settingsModal.querySelectorAll('input[name="protocolType"]').forEach(r => {
  r.addEventListener('change', updateProtocolFields);
});

function updateProtocolFields() {
  const selected = dom.settingsModal.querySelector('input[name="protocolType"]:checked')?.value;
  if (dom.serialGroup) dom.serialGroup.style.display = selected === 'serial' ? 'flex' : 'none';
  if (dom.wsGroup) dom.wsGroup.style.display = selected === 'websocket' ? 'flex' : 'none';
  if (dom.rosGroup) dom.rosGroup.style.display = selected === 'ros' ? 'flex' : 'none';
  if (dom.httpGroup) dom.httpGroup.style.display = selected === 'http' ? 'flex' : 'none';
}

dom.btnSaveSettings.addEventListener('click', () => {
  const newProtocol = dom.settingsModal.querySelector('input[name="protocolType"]:checked').value;
  const protocolChanged = newProtocol !== state.protocol;

  state.protocol = newProtocol;
  if (dom.selBaudRate) state.baudRate = parseInt(dom.selBaudRate.value, 10);
  state.wsUrl = dom.txtWsUrl.value.trim();
  state.rosTopic = dom.txtRosTopic.value.trim();
  state.rosWsUrl = dom.txtRosWsUrl.value.trim();
  state.httpUrl = dom.txtHttpUrl.value.trim();
  state.httpMethod = dom.selHttpMethod.value;
  state.camUrl = dom.txtCamUrl.value.trim();

  state.panMin = parseInt(dom.numPanMin.value, 10);
  state.panMax = parseInt(dom.numPanMax.value, 10);
  state.tiltMin = parseInt(dom.numTiltMin.value, 10);
  state.tiltMax = parseInt(dom.numTiltMax.value, 10);

  // Update HUD displays
  if (dom.protocolBadge) {
    dom.protocolBadge.textContent = `PROTOCOL: ${state.protocol === 'serial' ? 'USB SERIAL' : state.protocol.toUpperCase()}`;
  }
  dom.activeProtoLabel.textContent = state.protocol === 'serial' ? 'USB' : state.protocol.substring(0, 4).toUpperCase();
  
  if (state.protocol === 'serial') {
    dom.activeUrlDisplay.textContent = `USB COM PORT (${state.baudRate} Baud)`;
  } else if (state.protocol === 'websocket') {
    dom.activeUrlDisplay.textContent = state.wsUrl;
  } else if (state.protocol === 'ros') {
    dom.activeUrlDisplay.textContent = `${state.rosWsUrl} [${state.rosTopic}]`;
  } else if (state.protocol === 'http') {
    dom.activeUrlDisplay.textContent = state.httpUrl;
  } else {
    dom.activeUrlDisplay.textContent = 'SIMULATION (OFFLINE)';
  }

  // Update slider bounds
  dom.sliderPan.min = state.panMin;
  dom.sliderPan.max = state.panMax;
  dom.sliderTilt.min = state.tiltMin;
  dom.sliderTilt.max = state.tiltMax;

  dom.settingsModal.style.display = 'none';
  log(`[CONFIG] Settings saved. Protocol: ${state.protocol}`, 'info');

  if (protocolChanged && state.isConnected) {
    disconnect();
    connect();
  }
  refreshCameraStream();
});

// Helper math clamp
function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

// ============================================================================
// Interactive Robot AI Q&A Engine (Voice + Typewriter + 3D & Physical Mouth)
// ============================================================================

const ROBOT_QA = [
  {
    id: 1,
    question: "What is Robo Miracle?",
    answer: "Robomiracle is an Indian robotics company that builds AI service robots and humanoids.",
    speech: "Robomiracle is an Indian robotics company that builds A I service robots and humanoids.",
    action: () => {
      // Gentle nod & glowing visor pulse
      if (visorLight) visorLight.intensity = 2.4;
      setTimeout(() => { if (visorLight) visorLight.intensity = 1.2; }, 1200);
    }
  },
  {
    id: 2,
    question: "What is Robotics?",
    answer: "Robotics is the science of designing and building automated, intelligent machines.",
    speech: "Robotics is the science of designing and building automated, intelligent machines.",
    action: () => {
      // Sweep head slightly side to side to demonstrate movement
      setHeadPosition(105, 88, 'DEMO_PAN');
      setTimeout(() => { setHeadPosition(75, 92, 'DEMO_PAN'); }, 500);
      setTimeout(() => { setHeadPosition(90, 90, 'CENTER'); }, 1000);
    }
  },
  {
    id: 3,
    question: "What is your name?",
    answer: "My name is RoboHead, your robotic assistant.",
    speech: "My name is RoboHead, your robotic assistant.",
    action: () => {
      // Confident nod "Yes"
      playNodSequence();
    }
  },
  {
    id: 4,
    question: "Where is the pupil located?",
    answer: "The pupil is located in the center of the eye's iris.",
    speech: "The pupil is located in the center of the eye's iris.",
    action: () => {
      // Focus eyes forward at center (90, 90) and brighten visor eye glow
      setHeadPosition(90, 90, 'CENTER');
      if (visorLight) {
        visorLight.intensity = 3.0;
        setTimeout(() => { if (visorLight) visorLight.intensity = 1.2; }, 1400);
      }
    }
  }
];

let voiceEnabled = true;
let isAnswering = false;
let mouthFlapInterval = null;
let qaTypingTimer = null;

function setupQAEngine() {
  if (dom.qaVoiceBtn) {
    dom.qaVoiceBtn.addEventListener('click', () => {
      voiceEnabled = !voiceEnabled;
      if (voiceEnabled) {
        dom.qaVoiceBtn.textContent = 'VOICE AUDIO: ON';
        dom.qaVoiceBtn.classList.remove('muted');
      } else {
        dom.qaVoiceBtn.textContent = 'VOICE AUDIO: OFF';
        dom.qaVoiceBtn.classList.add('muted');
        if (window.speechSynthesis) window.speechSynthesis.cancel();
        stopMouthSpeaking();
      }
    });
  }

  // Voice recognition mic buttons
  [dom.btnVoiceMic, dom.btnHeaderMic].forEach(btn => {
    if (btn) {
      btn.addEventListener('click', () => {
        toggleVoiceRecognition();
      });
    }
  });

  if (dom.qaButtons) {
    dom.qaButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        const qid = parseInt(btn.dataset.qid, 10);
        triggerQuestion(qid);
      });
    });
  }
}

function triggerQuestion(qid) {
  const item = ROBOT_QA.find(q => q.id === qid);
  if (!item) return;

  // Visual button states
  if (dom.qaButtons) {
    dom.qaButtons.forEach(b => {
      b.classList.toggle('active', parseInt(b.dataset.qid, 10) === qid);
      b.classList.remove('speaking');
    });
    const activeBtn = Array.from(dom.qaButtons).find(b => parseInt(b.dataset.qid, 10) === qid);
    if (activeBtn) activeBtn.classList.add('speaking');
  }

  // Update prompt & status badge
  if (dom.qaDisplayPrompt) {
    dom.qaDisplayPrompt.textContent = `Q: ${item.question.toUpperCase()}`;
  }
  if (dom.qaStatusTag) {
    dom.qaStatusTag.textContent = 'SPEAKING...';
    dom.qaStatusTag.classList.add('active');
  }

  // Log in cockpit terminal
  log(`[AI QUESTION] ❓ ${item.question}`, 'tx');
  log(`[ROBO ANSWER] 💬 ${item.answer}`, 'rx');

  // Interactive typewriter text reveal
  typeAnswerText(item.answer);

  // Trigger specialized question gesture
  if (typeof item.action === 'function') {
    item.action();
  }

  // Stop any active previous speech
  if (window.speechSynthesis) {
    window.speechSynthesis.cancel();
  }
  stopMouthSpeaking();

  // Speak aloud with synchronized mouth movement
  if (voiceEnabled && ('speechSynthesis' in window)) {
    speakAnswerWithMouth(item.speech || item.answer);
  } else {
    // If voice muted, simulate physical mouth movement for ~2s
    startMouthSpeaking();
    setTimeout(() => {
      stopMouthSpeaking();
      if (dom.qaStatusTag) {
        dom.qaStatusTag.textContent = 'READY';
        dom.qaStatusTag.classList.remove('active');
      }
      if (dom.qaButtons) dom.qaButtons.forEach(b => b.classList.remove('speaking'));
    }, 2000);
  }
}

function typeAnswerText(fullText) {
  if (!dom.qaAnswerText) return;
  clearInterval(qaTypingTimer);
  dom.qaAnswerText.classList.add('answering');
  dom.qaAnswerText.textContent = '';

  let idx = 0;
  const speed = Math.max(14, Math.floor(1200 / fullText.length)); // Smooth reveal over ~1.2s
  qaTypingTimer = setInterval(() => {
    if (idx < fullText.length) {
      dom.qaAnswerText.textContent += fullText.charAt(idx);
      idx++;
    } else {
      clearInterval(qaTypingTimer);
      dom.qaAnswerText.classList.remove('answering');
    }
  }, speed);
}

function startMouthSpeaking() {
  if (mouthFlapInterval) clearInterval(mouthFlapInterval);
  isAnswering = true;
  state.mouthActive = true;

  // Send MOUTH ON command to physical Arduino
  sendCommand({ cmd: 'MOUTH', state: 'ON', timestamp: Date.now() });

  // Rhythmic physical speech cadence: flaps mouth open and closed during speech
  let flap = true;
  mouthFlapInterval = setInterval(() => {
    if (!isAnswering) return;
    flap = !flap;
    state.mouthActive = flap;
    // Rhythmic subtle pitch cadence
    if (robotHeadGroup) {
      robotHeadGroup.position.y = flap ? -0.015 : 0;
    }
  }, 160);
}

function stopMouthSpeaking() {
  isAnswering = false;
  state.mouthActive = false;
  if (mouthFlapInterval) {
    clearInterval(mouthFlapInterval);
    mouthFlapInterval = null;
  }
  if (robotHeadGroup) {
    robotHeadGroup.position.y = 0;
  }
  // Send MOUTH OFF command to physical Arduino to return mouth to closed position
  sendCommand({ cmd: 'MOUTH', state: 'OFF', timestamp: Date.now() });
}

function speakAnswerWithMouth(text) {
  try {
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.rate = 0.98;
    utterance.pitch = 1.04;

    // Select preferred english voice if available
    const voices = window.speechSynthesis.getVoices();
    const preferredVoice = voices.find(v => v.lang.startsWith('en') && (v.name.includes('Natural') || v.name.includes('Google') || v.name.includes('Samantha') || v.name.includes('David')));
    if (preferredVoice) utterance.voice = preferredVoice;

    utterance.onstart = () => {
      startMouthSpeaking();
    };

    utterance.onend = () => {
      stopMouthSpeaking();
      if (dom.qaStatusTag) {
        dom.qaStatusTag.textContent = 'READY';
        dom.qaStatusTag.classList.remove('active');
      }
      if (dom.qaButtons) dom.qaButtons.forEach(b => b.classList.remove('speaking'));
    };

    utterance.onerror = () => {
      stopMouthSpeaking();
      if (dom.qaStatusTag) {
        dom.qaStatusTag.textContent = 'READY';
        dom.qaStatusTag.classList.remove('active');
      }
      if (dom.qaButtons) dom.qaButtons.forEach(b => b.classList.remove('speaking'));
    };

    window.speechSynthesis.speak(utterance);
  } catch (e) {
    console.warn('[QA SPEECH]', e);
    startMouthSpeaking();
    setTimeout(stopMouthSpeaking, 3500);
  }
}

// ============================================================================
// Two-Way Voice Recognition Interaction Engine (Speech-to-Text & Command Parser)
// ============================================================================

let recognition = null;
let isVoiceListeningActive = false;
let isSpeakingVoice = false;
let heardBannerTimer = null;

function initVoiceRecognition() {
  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRecognition) {
    console.warn('[VOICE] SpeechRecognition API not supported in this browser.');
    return false;
  }

  try {
    recognition = new SpeechRecognition();
    recognition.continuous = true;
    recognition.interimResults = false;
    recognition.lang = 'en-US';

    recognition.onstart = () => {
      isVoiceListeningActive = true;
      updateVoiceUI(true);
      log('[VOICE] 🎙️ Microphone ACTIVE. Speak commands or ask questions...', 'rx');
    };

    recognition.onresult = (event) => {
      // Ignore microphone while robot is speaking or answering
      if (isSpeakingVoice || isAnswering) return;

      const lastResultIndex = event.results.length - 1;
      const transcript = event.results[lastResultIndex][0].transcript;
      if (transcript && transcript.trim()) {
        handleVoiceCommand(transcript.trim());
      }
    };

    recognition.onerror = (event) => {
      console.warn('[VOICE ERROR]', event.error);
      if (event.error === 'not-allowed') {
        alert("⚠️ Microphone Access Denied!\nPlease allow microphone permissions in your browser URL bar to use voice commands.");
        stopVoiceRecognition();
      }
    };

    recognition.onend = () => {
      // Automatically keep listening if user enabled voice mode
      if (isVoiceListeningActive) {
        try {
          recognition.start();
        } catch (e) {}
      } else {
        updateVoiceUI(false);
      }
    };

    return true;
  } catch (e) {
    console.error('[VOICE INIT ERROR]', e);
    return false;
  }
}

function startVoiceRecognition() {
  if (!recognition && !initVoiceRecognition()) {
    alert("⚠️ Web Speech Recognition is supported in Google Chrome, Microsoft Edge, and Opera.\nPlease open this dashboard in Chrome or Edge to use voice interaction!");
    return;
  }

  isVoiceListeningActive = true;
  try {
    recognition.start();
  } catch (e) {
    updateVoiceUI(true);
  }
}

function stopVoiceRecognition() {
  isVoiceListeningActive = false;
  if (recognition) {
    try { recognition.stop(); } catch (e) {}
  }
  updateVoiceUI(false);
  log('[VOICE] 🔇 Voice recognition paused.', 'info');
}

function toggleVoiceRecognition() {
  if (isVoiceListeningActive) {
    stopVoiceRecognition();
  } else {
    startVoiceRecognition();
  }
}

function updateVoiceUI(active) {
  if (dom.btnVoiceMic) {
    dom.btnVoiceMic.classList.toggle('listening', active);
  }
  if (dom.voiceMicTitle) {
    dom.voiceMicTitle.textContent = active ? 'LISTENING (SPEAK NOW)...' : 'VOICE CONTROL: OFF';
  }
  if (dom.voiceMicSub) {
    dom.voiceMicSub.textContent = active ? 'SAY "TURN RIGHT", "TURN LEFT", "LOOK UP" OR ASK A QUESTION' : 'CLICK TO START LISTENING';
  }
  if (dom.voiceMicBadge) {
    dom.voiceMicBadge.textContent = active ? 'LIVE' : 'MIC OFF';
    dom.voiceMicBadge.classList.toggle('active', active);
  }
  if (dom.btnHeaderMic) {
    dom.btnHeaderMic.classList.toggle('active', active);
    dom.btnHeaderMic.textContent = active ? 'LISTENING' : 'VOICE';
  }
  if (dom.qaStatusTag && !isAnswering) {
    dom.qaStatusTag.textContent = active ? 'LISTENING' : 'READY';
    dom.qaStatusTag.classList.toggle('active', active);
  }
}

function showVoiceHeardBanner(text) {
  if (!dom.voiceHeardBanner || !dom.voiceHeardText) return;
  dom.voiceHeardText.textContent = `"${text}"`;
  dom.voiceHeardBanner.style.display = 'flex';

  clearTimeout(heardBannerTimer);
  heardBannerTimer = setTimeout(() => {
    if (dom.voiceHeardBanner) {
      dom.voiceHeardBanner.style.display = 'none';
    }
  }, 4500);
}

function speakVoiceFeedback(phrase) {
  if (!voiceEnabled || !('speechSynthesis' in window)) return;
  try {
    isSpeakingVoice = true;
    const u = new SpeechSynthesisUtterance(phrase);
    u.rate = 1.05;
    u.pitch = 1.08;
    u.onend = () => { setTimeout(() => { isSpeakingVoice = false; }, 350); };
    u.onerror = () => { isSpeakingVoice = false; };
    window.speechSynthesis.speak(u);
  } catch (e) {
    isSpeakingVoice = false;
  }
}

function handleVoiceCommand(rawText) {
  const text = rawText.trim().toLowerCase();
  showVoiceHeardBanner(rawText);
  log(`[VOICE INPUT] 🎙️ Heard: "${rawText}"`, 'tx');

  // 1. Customized Questions Recognition
  if (text.includes('miracle') || text.includes('robo miracle') || text.includes('robomiracle')) {
    triggerQuestion(1);
    return;
  }
  if (text.includes('robotics') || text.includes('what is robotics') || text.includes('define robotics')) {
    triggerQuestion(2);
    return;
  }
  if (text.includes('your name') || text.includes('who are you') || text.includes('ur name') || text.includes("what's your name") || text.includes('what is your name')) {
    triggerQuestion(3);
    return;
  }
  if (text.includes('pupil') || text.includes('where is pupil') || text.includes('where does pupil') || text.includes('where pupil is located') || text.includes('eye')) {
    triggerQuestion(4);
    return;
  }

  // 2. Directional Kinematic Commands
  // TURN RIGHT / LOOK RIGHT
  if (text.includes('right') || text.includes('turn right') || text.includes('look right') || text.includes('pan right') || text.includes('move right')) {
    const newPan = clamp(state.pan + 25, state.panMin, state.panMax);
    setHeadPosition(newPan, state.tilt, 'RIGHT');
    speakVoiceFeedback("Turning right");
    log('[VOICE COMMAND] 👉 Turned RIGHT', 'rx');
    return;
  }

  // TURN LEFT / LOOK LEFT
  if (text.includes('left') || text.includes('turn left') || text.includes('look left') || text.includes('pan left') || text.includes('move left')) {
    const newPan = clamp(state.pan - 25, state.panMin, state.panMax);
    setHeadPosition(newPan, state.tilt, 'LEFT');
    speakVoiceFeedback("Turning left");
    log('[VOICE COMMAND] 👈 Turned LEFT', 'rx');
    return;
  }

  // LOOK UP / TURN UP
  if (text.includes('up') || text.includes('look up') || text.includes('turn up') || text.includes('tilt up') || text.includes('head up')) {
    const newTilt = clamp(state.tilt + 20, state.tiltMin, state.tiltMax);
    setHeadPosition(state.pan, newTilt, 'UP');
    speakVoiceFeedback("Looking up");
    log('[VOICE COMMAND] 👆 Looked UP', 'rx');
    return;
  }

  // LOOK DOWN / TURN DOWN
  if (text.includes('down') || text.includes('look down') || text.includes('turn down') || text.includes('tilt down') || text.includes('head down')) {
    const newTilt = clamp(state.tilt - 20, state.tiltMin, state.tiltMax);
    setHeadPosition(state.pan, newTilt, 'DOWN');
    speakVoiceFeedback("Looking down");
    log('[VOICE COMMAND] 👇 Looked DOWN', 'rx');
    return;
  }

  // CENTER / HOME / STRAIGHT
  if (text.includes('center') || text.includes('straight') || text.includes('forward') || text.includes('home') || text.includes('neutral') || text.includes('reset')) {
    setHeadPosition(90, 90, 'CENTER');
    speakVoiceFeedback("Centered");
    log('[VOICE COMMAND] 🎯 Centered head position', 'rx');
    return;
  }

  // 3. Mouth Actuator Commands
  if (text.includes('open mouth') || text.includes('mouth on') || text.includes('mouth open')) {
    toggleMouth(true);
    speakVoiceFeedback("Mouth opened");
    log('[VOICE COMMAND] 👄 Opened mouth (Pin 3)', 'rx');
    return;
  }

  if (text.includes('close mouth') || text.includes('mouth off') || text.includes('mouth close')) {
    toggleMouth(false);
    speakVoiceFeedback("Mouth closed");
    log('[VOICE COMMAND] Closed mouth (Pin 3)', 'rx');
    return;
  }

  // Food Pipe Commands
  if (text.includes('food pipe open') || text.includes('open food pipe')) {
    toggleFoodPipe(true);
    speakVoiceFeedback("Food pipe opened");
    log('[VOICE COMMAND] Opened food pipe', 'rx');
    return;
  }

  if (text.includes('food pipe close') || text.includes('close food pipe') || text.includes('food pipe off')) {
    toggleFoodPipe(false);
    speakVoiceFeedback("Food pipe closed");
    log('[VOICE COMMAND] Closed food pipe', 'rx');
    return;
  }

  // 4. Special Actions: Lungs / Air Blower
  if (text.includes('blow off') || text.includes('stop blow') || text.includes('lungs off') || text.includes('stop lungs')) {
    stopBlowAction();
    speakVoiceFeedback("Lungs off");
    log('[VOICE COMMAND] Lungs turned OFF', 'rx');
    return;
  }

  if (text.includes('lungs step 1') || text.includes('lung step 1') || text.includes('lungs 1')) {
    setBlowInterval(1.0, true);
    speakVoiceFeedback("Lungs step 1");
    log('[VOICE COMMAND] Lungs set to STEP 1 (1s)', 'rx');
    return;
  }
  if (text.includes('lungs step 2') || text.includes('lung step 2') || text.includes('lungs 2')) {
    setBlowInterval(2.0, true);
    speakVoiceFeedback("Lungs step 2");
    log('[VOICE COMMAND] Lungs set to STEP 2 (2s)', 'rx');
    return;
  }
  if (text.includes('lungs step 3') || text.includes('lung step 3') || text.includes('lungs 3')) {
    setBlowInterval(3.0, true);
    speakVoiceFeedback("Lungs step 3");
    log('[VOICE COMMAND] Lungs set to STEP 3 (3s)', 'rx');
    return;
  }

  if (text.includes('blow') || text.includes('blow on') || text.includes('blow air') || text.includes('lungs on') || text.includes('lungs') || text.includes('breathe')) {
    startBlowAction();
    speakVoiceFeedback("Lungs on");
    log('[VOICE COMMAND] Lungs turned ON', 'rx');
    return;
  }

  // Pulse Speed Controls (HIGH: 1ms, MEDIUM: 20ms, LOW: 35ms)
  if (text.includes('pulse high') || text.includes('pulse speed high') || text.includes('fast pulse') || text.includes('high pulse')) {
    setPulseSpeed(1, true);
    speakVoiceFeedback("Pulse speed set to high");
    log('[VOICE COMMAND] Pulse speed set to HIGH', 'rx');
    return;
  }
  if (text.includes('pulse medium') || text.includes('pulse speed medium') || text.includes('normal pulse') || text.includes('medium pulse')) {
    setPulseSpeed(20, true);
    speakVoiceFeedback("Pulse speed set to medium");
    log('[VOICE COMMAND] Pulse speed set to MEDIUM', 'rx');
    return;
  }
  if (text.includes('pulse low') || text.includes('pulse speed low') || text.includes('slow pulse') || text.includes('low pulse')) {
    setPulseSpeed(35, true);
    speakVoiceFeedback("Pulse speed set to low");
    log('[VOICE COMMAND] Pulse speed set to LOW', 'rx');
    return;
  }

  if (text.includes('pulse') || text.includes('neck pulse')) {
    triggerNeckPulse();
    log('[VOICE COMMAND] ⚡ Pulse triggered', 'rx');
    return;
  }

  if (text.includes('nod') || text.includes('yes')) {
    playNodSequence();
    speakVoiceFeedback("Yes");
    return;
  }

  if (text.includes('shake') || text.includes('no')) {
    playShakeSequence();
    speakVoiceFeedback("No");
    return;
  }

  // 5. Hand Tracking Voice Commands
  if (text.includes('hand track') || text.includes('hand control') || text.includes('follow my hand') || text.includes('track hand') || text.includes('start hand')) {
    speakVoiceFeedback("Opening hand tracking studio");
    log('[VOICE COMMAND] ✋ Opening Hand Tracking Studio...', 'rx');
    navigateToHandTrackStudio();
    return;
  }

  if (text.includes('stop hand') || text.includes('stop tracking')) {
    stopHandTracking();
    speakVoiceFeedback("Hand tracking stopped");
    log('[VOICE COMMAND] ✋ Hand tracking stopped', 'rx');
    return;
  }

  // Log unmapped speech
  log(`[VOICE UNRECOGNIZED] "${rawText}" - (Try: "Turn Right", "Turn Left", "Look Up", "Look Down", "Center", or ask a question)`, 'warn');
}

// ============================================================================
// Real-Time Hand Tracking & Gesture Control Engine (MediaPipe + Motion Fallback)
// ============================================================================

const handTrack = {
  active: false,
  stream: null,
  camera: null,
  hands: null,
  hasHand: false,
  usingMediaPipe: false,
  animationId: null,
  rawX: 0.5,
  rawY: 0.5,
  targetPan: 90,
  targetTilt: 90,
  smoothedPan: 90,
  smoothedTilt: 90,
  mirrorX: true,
  invertY: false,
  smoothing: 0.35,
  lastSerialDispatch: 0,
  isFist: false,
  prevFrameData: null,
  offCanvas: null,
  offCtx: null
};

async function navigateToHandTrackStudio() {
  log('[NAVIGATION] ✋ Opening Hand Tracking Studio (2-Column Vision Control)...', 'info');
  // Cleanly close serial reader & port if open so hand_track.html can immediately connect
  if (state.serialReader) {
    try { await state.serialReader.cancel(); } catch (e) {}
    state.serialReader = null;
  }
  if (state.serialPort) {
    try { await state.serialPort.close(); } catch (e) {}
    state.serialPort = null;
    state.isConnected = false;
  }
  window.location.href = 'hand_track.html';
}

function initHandTracking() {
  if (dom.btnHeaderHand) {
    dom.btnHeaderHand.addEventListener('click', (e) => {
      e.preventDefault();
      navigateToHandTrackStudio();
    });
  }
  if (dom.btnPresetHand) {
    dom.btnPresetHand.addEventListener('click', (e) => {
      e.preventDefault();
      navigateToHandTrackStudio();
    });
  }
  if (dom.btnCloseHandHud) {
    dom.btnCloseHandHud.addEventListener('click', stopHandTracking);
  }
  if (dom.btnMinimizeHandHud) {
    dom.btnMinimizeHandHud.addEventListener('click', () => {
      if (dom.handHudOverlay) {
        dom.handHudOverlay.classList.toggle('minimized');
      }
    });
  }
  if (dom.chkHandMirror) {
    dom.chkHandMirror.addEventListener('change', (e) => {
      handTrack.mirrorX = e.target.checked;
    });
  }
  if (dom.chkHandInvertY) {
    dom.chkHandInvertY.addEventListener('change', (e) => {
      handTrack.invertY = e.target.checked;
    });
  }
}

async function getPreferredWebcamStream() {
  const builtinPatterns = [
    'integrated', 'built-in', 'builtin', 'internal', 'laptop',
    'facetime', 'truevision', 'user facing', 'hd user', 'acer hd', 'acer crystal', 'crystal eye'
  ];
  const virtualIrPatterns = ['ir camera', 'windows hello', 'infrared', 'depth', 'obs virtual', 'virtual camera'];
  const isBuiltIn = (lbl) => builtinPatterns.some((kw) => (lbl || '').toLowerCase().includes(kw));
  const isVirtualIr = (lbl) => virtualIrPatterns.some((kw) => (lbl || '').toLowerCase().includes(kw));

  let devices = await navigator.mediaDevices.enumerateDevices();
  let videoDevices = devices.filter((d) => d.kind === 'videoinput');
  let tempStream = null;

  if (videoDevices.length > 0 && videoDevices.every((d) => !d.label)) {
    tempStream = await navigator.mediaDevices.getUserMedia({ video: { width: { ideal: 640 }, height: { ideal: 480 } }, audio: false });
    devices = await navigator.mediaDevices.enumerateDevices();
    videoDevices = devices.filter((d) => d.kind === 'videoinput');
  }

  const realCams = videoDevices.filter((d) => !isVirtualIr(d.label));
  const candidates = realCams.length > 0 ? realCams : videoDevices;

  // Primary = External USB Web Camera; Secondary = Built-in Laptop Camera (only when Web Camera is not connected)
  let chosen = null;
  if (candidates.length >= 2) {
    const externalCams = candidates.filter((d) => !isBuiltIn(d.label));
    chosen = externalCams.length > 0 ? externalCams[0] : candidates[candidates.length - 1];
    log(`[CAMERA] 🎥 Primary Web Camera selected: ${chosen.label || 'USB Web Camera'} (Laptop camera disabled)`, 'rx');
  } else if (candidates.length === 1) {
    chosen = candidates[0];
    log(`[CAMERA] 💻 Secondary Laptop Camera active (No external web camera detected): ${chosen.label || 'Default Camera'}`, 'info');
  }

  if (tempStream) {
    const activeTrack = tempStream.getVideoTracks()[0];
    const activeId = activeTrack ? activeTrack.getSettings().deviceId : null;
    if (!chosen || activeId === chosen.deviceId) {
      return tempStream;
    }
    tempStream.getTracks().forEach((t) => t.stop());
  }

  return navigator.mediaDevices.getUserMedia({
    video: chosen && chosen.deviceId
      ? { deviceId: { exact: chosen.deviceId }, width: { ideal: 640 }, height: { ideal: 480 } }
      : { width: { ideal: 640 }, height: { ideal: 480 } },
    audio: false
  });
}

async function startHandTracking() {
  if (handTrack.active) return;

  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    alert("⚠️ Webcam access is not supported by your browser.\nPlease use Google Chrome or Microsoft Edge and ensure camera permissions are allowed.");
    return;
  }

  log('[HAND TRACK] ✋ Requesting camera video feed (Web Camera primary, Laptop secondary)...', 'info');

  try {
    const stream = await getPreferredWebcamStream();

    handTrack.stream = stream;
    handTrack.active = true;

    if (dom.handWebcam) {
      dom.handWebcam.srcObject = stream;
      await dom.handWebcam.play();
    }

    if (dom.handHudOverlay) {
      dom.handHudOverlay.style.display = 'flex';
      dom.handHudOverlay.classList.remove('minimized');
    }

    if (dom.btnHeaderHand) dom.btnHeaderHand.classList.add('active');
    if (dom.btnPresetHand) dom.btnPresetHand.classList.add('active');

    log('[HAND TRACK] 🟢 Webcam active. Initializing Hand Pose Detector...', 'rx');
    updateHandStatusBadge('SEARCHING', 'searching');

    // Attempt MediaPipe Hands
    if (typeof window.Hands !== 'undefined') {
      setupMediaPipeHands();
    } else {
      log('[HAND TRACK] ⚡ MediaPipe CDN initializing... Starting instant Motion Tracker fallback', 'info');
      startMotionTrackerFallback();
      let mpCheckCount = 0;
      const mpTimer = setInterval(() => {
        mpCheckCount++;
        if (typeof window.Hands !== 'undefined') {
          clearInterval(mpTimer);
          setupMediaPipeHands();
        } else if (mpCheckCount > 20) {
          clearInterval(mpTimer);
        }
      }, 500);
    }
  } catch (err) {
    console.error('[HAND TRACK ERROR]', err);
    log(`[HAND TRACK ERROR] ${err.name || 'Camera error'}: ${err.message}`, 'err');
    if (err.name === 'NotAllowedError' || err.name === 'PermissionDeniedError') {
      alert("⚠️ Camera Access Denied!\nPlease allow camera permissions in your browser URL bar to use hand tracking.");
    } else {
      alert(`⚠️ Could not access webcam: ${err.message}`);
    }
    stopHandTracking();
  }
}

function stopHandTracking() {
  handTrack.active = false;

  if (handTrack.camera) {
    try { handTrack.camera.stop(); } catch (e) {}
    handTrack.camera = null;
  }

  if (handTrack.animationId) {
    cancelAnimationFrame(handTrack.animationId);
    handTrack.animationId = null;
  }

  if (handTrack.stream) {
    handTrack.stream.getTracks().forEach(track => track.stop());
    handTrack.stream = null;
  }

  if (dom.handWebcam) {
    dom.handWebcam.srcObject = null;
  }

  if (dom.handHudOverlay) {
    dom.handHudOverlay.style.display = 'none';
  }

  if (dom.btnHeaderHand) dom.btnHeaderHand.classList.remove('active');
  if (dom.btnPresetHand) dom.btnPresetHand.classList.remove('active');

  log('[HAND TRACK] ⏹ Hand tracking stopped.', 'warn');
}

function toggleHandTracking() {
  if (handTrack.active) {
    stopHandTracking();
  } else {
    startHandTracking();
  }
}

function setupMediaPipeHands() {
  try {
    if (handTrack.hands) return;

    log('[HAND TRACK] 🚀 Loading MediaPipe Hands 21-Joint Model...', 'info');
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
    handTrack.hands = hands;
    handTrack.usingMediaPipe = true;

    if (typeof Camera !== 'undefined' && dom.handWebcam) {
      handTrack.camera = new Camera(dom.handWebcam, {
        onFrame: async () => {
          if (handTrack.active && dom.handWebcam && dom.handWebcam.readyState >= 2) {
            await hands.send({ image: dom.handWebcam });
          }
        },
        width: 640,
        height: 480
      });
      handTrack.camera.start();
      log('[HAND TRACK] ✅ MediaPipe Camera loop started!', 'rx');
    } else {
      startManualVideoSendLoop();
    }
  } catch (e) {
    console.warn('[HAND TRACK] MediaPipe init error, using fallback:', e);
    startMotionTrackerFallback();
  }
}

function startManualVideoSendLoop() {
  async function loop() {
    if (!handTrack.active) return;
    if (dom.handWebcam && dom.handWebcam.readyState >= 2 && handTrack.hands) {
      try {
        await handTrack.hands.send({ image: dom.handWebcam });
      } catch (err) {}
    }
    handTrack.animationId = requestAnimationFrame(loop);
  }
  handTrack.animationId = requestAnimationFrame(loop);
}

function onMediaPipeHandResults(results) {
  if (!handTrack.active) return;
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

  // Dark sci-fi tint overlay
  ctx.fillStyle = 'rgba(7, 10, 18, 0.28)';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
    handTrack.hasHand = true;
    const rawLandmarks = results.multiHandLandmarks[0];

    // Mirror landmarks coordinates since video is flipped horizontally
    const landmarks = rawLandmarks.map(pt => ({
      x: 1.0 - pt.x,
      y: pt.y,
      z: pt.z
    }));

    // Draw Cybernetic Neon Skeleton
    drawCyberHandSkeleton(ctx, landmarks, canvas.width, canvas.height);

    // Track palm position: landmark 9 (Middle finger MCP) + landmark 0 (wrist) midpoint
    const palmX = (landmarks[9].x * 0.7) + (landmarks[0].x * 0.3);
    const palmY = (landmarks[9].y * 0.7) + (landmarks[0].y * 0.3);

    // Detect gesture
    const gesture = analyzeHandGesture(landmarks);
    handTrack.isFist = gesture.isFist;

    // Draw Tracking Reticle & Hologram line
    drawTargetReticle(ctx, palmX * canvas.width, palmY * canvas.height, gesture.name);

    if (gesture.isCenter || gesture.isFist || gesture.isOne) {
      const badgeText = gesture.isFist ? 'FIST // CENTER' : (gesture.isOne ? 'SHOW 1 // CENTER' : 'CENTER');
      updateHandStatusBadge(badgeText, 'tracking');
      if (dom.handValGesture) dom.handValGesture.textContent = gesture.name;
      // Quick return to Center / Home (90°, 90°)
      setHeadPosition(90, 90, 'CENTER');
    } else if (gesture.isPinch) {
      updateHandStatusBadge('PINCH // BLOW', 'active');
      if (dom.handValGesture) dom.handValGesture.textContent = gesture.name;
      triggerBlowAction();
    } else {
      updateHandStatusBadge('TRACKING', 'tracking');
      if (dom.handValGesture) dom.handValGesture.textContent = gesture.name;
      computeAndApplyHandKinematics(palmX, palmY);
    }
  } else {
    handTrack.hasHand = false;
    updateHandStatusBadge('SEARCHING', 'searching');
    if (dom.handValGesture) dom.handValGesture.textContent = 'NO HAND';
    drawSearchingCrosshairs(ctx, canvas.width, canvas.height);
  }
}

function analyzeHandGesture(lm) {
  const dist = (i, j) => Math.hypot(lm[i].x - lm[j].x, lm[i].y - lm[j].y);

  const indexExtended = (dist(8, 0) > dist(6, 0) * 1.10) && (lm[8].y < lm[6].y || dist(8, 0) > dist(6, 0) * 1.25);
  const middleExtended = (dist(12, 0) > dist(10, 0) * 1.10) && (lm[12].y < lm[10].y || dist(12, 0) > dist(10, 0) * 1.25);
  const ringExtended = (dist(16, 0) > dist(14, 0) * 1.10) && (lm[16].y < lm[14].y || dist(16, 0) > dist(14, 0) * 1.25);
  const pinkyExtended = (dist(20, 0) > dist(18, 0) * 1.10) && (lm[20].y < lm[18].y || dist(20, 0) > dist(18, 0) * 1.25);
  const pinchDist = Math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y);

  // 1. Closed hand / Fist: all 4 fingers folded
  const isFist = !indexExtended && !middleExtended && !ringExtended && !pinkyExtended;

  // 2. Shows 1: index finger extended, middle, ring, pinky folded
  const isOne = indexExtended && !middleExtended && !ringExtended && !pinkyExtended;

  if (isFist) {
    return { isFist: true, isOne: false, isCenter: true, name: '✊ CLOSED HAND (CENTER)' };
  }

  if (isOne) {
    return { isFist: false, isOne: true, isCenter: true, name: '☝️ SHOWING 1 (CENTER)' };
  }

  if (pinchDist < 0.055) {
    return { isFist: false, isPinch: true, name: '👌 PINCH' };
  }

  if (indexExtended && middleExtended && !ringExtended && !pinkyExtended) {
    return { isFist: false, isPeace: true, isCenter: true, name: '✌️ PEACE (CENTER)' };
  }

  return { isFist: false, name: '🖐️ OPEN PALM' };
}

function computeAndApplyHandKinematics(visualX, visualY) {
  // visualX: 0.0 (left) to 1.0 (right) in mirrored view
  // Active tracking area with padding (0.15 to 0.85)
  const normX = clamp((visualX - 0.15) / 0.70, 0, 1);
  const normY = clamp((visualY - 0.15) / 0.70, 0, 1);

  // Horizontal Pan:
  // Moving hand Left (normX near 0) -> Pan Left (140° or panMax)
  // Moving hand Right (normX near 1) -> Pan Right (50° or panMin)
  const panRatio = handTrack.mirrorX ? (1.0 - normX) : normX;
  const targetPan = Math.round(state.panMin + panRatio * (state.panMax - state.panMin));

  // Vertical Tilt:
  // Moving hand Up (normY near 0) -> Tilt Up (135° or tiltMax)
  // Moving hand Down (normY near 1) -> Tilt Down (60° or tiltMin)
  const tiltRatio = handTrack.invertY ? normY : (1.0 - normY);
  const targetTilt = Math.round(state.tiltMin + tiltRatio * (state.tiltMax - state.tiltMin));

  // Lerp smoothing to eliminate camera tremor
  handTrack.smoothedPan += (targetPan - handTrack.smoothedPan) * handTrack.smoothing;
  handTrack.smoothedTilt += (targetTilt - handTrack.smoothedTilt) * handTrack.smoothing;

  const currentPan = Math.round(handTrack.smoothedPan);
  const currentTilt = Math.round(handTrack.smoothedTilt);

  // Update HUD text
  if (dom.handValPan) dom.handValPan.textContent = `${currentPan}°`;
  if (dom.handValTilt) dom.handValTilt.textContent = `${currentTilt}°`;

  // Update Head Position (both 3D model and physical serial output)
  setHeadPosition(currentPan, currentTilt, 'HAND_TRACK');
}

function startMotionTrackerFallback() {
  if (handTrack.usingMediaPipe) return;
  const canvas = dom.handCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  if (!handTrack.offCanvas) {
    handTrack.offCanvas = document.createElement('canvas');
    handTrack.offCanvas.width = 80;
    handTrack.offCanvas.height = 60;
    handTrack.offCtx = handTrack.offCanvas.getContext('2d', { willReadFrequently: true });
  }

  function frameLoop() {
    if (!handTrack.active || handTrack.usingMediaPipe) return;

    if (dom.handWebcam && dom.handWebcam.readyState >= 2) {
      // 1. Draw mirrored frame to main canvas
      ctx.save();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.translate(canvas.width, 0);
      ctx.scale(-1, 1);
      ctx.drawImage(dom.handWebcam, 0, 0, canvas.width, canvas.height);
      ctx.restore();

      // Dark sci-fi tint
      ctx.fillStyle = 'rgba(7, 10, 18, 0.28)';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // 2. Optical Motion Centroid Detection on 80x60
      const offW = 80, offH = 60;
      handTrack.offCtx.drawImage(dom.handWebcam, 0, 0, offW, offH);
      const curImgData = handTrack.offCtx.getImageData(0, 0, offW, offH);
      const curPixels = curImgData.data;

      if (handTrack.prevFrameData) {
        let sumX = 0, sumY = 0, totalDiff = 0;
        const prevPixels = handTrack.prevFrameData.data;

        for (let y = 0; y < offH; y++) {
          for (let x = 0; x < offW; x++) {
            const idx = (y * offW + x) * 4;
            const diffR = Math.abs(curPixels[idx] - prevPixels[idx]);
            const diffG = Math.abs(curPixels[idx + 1] - prevPixels[idx + 1]);
            const diffB = Math.abs(curPixels[idx + 2] - prevPixels[idx + 2]);
            const diff = (diffR + diffG + diffB) / 3;

            if (diff > 25) { // Motion threshold
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

          handTrack.hasHand = true;
          updateHandStatusBadge('MOTION TRACKING', 'tracking');
          if (dom.handValGesture) dom.handValGesture.textContent = '🖐️ MOTION FOLLOW';

          drawTargetReticle(ctx, centroidX * canvas.width, centroidY * canvas.height, 'MOTION');
          computeAndApplyHandKinematics(centroidX, centroidY);
        } else {
          handTrack.hasHand = false;
          updateHandStatusBadge('SEARCHING', 'searching');
          if (dom.handValGesture) dom.handValGesture.textContent = 'WAVE HAND';
          drawSearchingCrosshairs(ctx, canvas.width, canvas.height);
        }
      }

      handTrack.prevFrameData = curImgData;
    }

    handTrack.animationId = requestAnimationFrame(frameLoop);
  }

  handTrack.animationId = requestAnimationFrame(frameLoop);
}

function drawCyberHandSkeleton(ctx, lm, w, h) {
  const connections = [
    [0, 1], [1, 2], [2, 3], [3, 4],       // Thumb
    [0, 5], [5, 6], [6, 7], [7, 8],       // Index
    [5, 9], [9, 10], [10, 11], [11, 12],  // Middle
    [9, 13], [13, 14], [14, 15], [15, 16],// Ring
    [13, 17], [17, 18], [18, 19], [19, 20],// Pinky
    [0, 17]                                // Palm base
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
    ctx.arc(pt.x * w, pt.y * h, (i === 4 || i === 8 || i === 12 || i === 16 || i === 20) ? 4.5 : 2.5, 0, Math.PI * 2);
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
  ctx.arc(x, y, 22, 0, Math.PI * 2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(x - 30, y); ctx.lineTo(x - 10, y);
  ctx.moveTo(x + 10, y); ctx.lineTo(x + 30, y);
  ctx.moveTo(x, y - 30); ctx.lineTo(x, y - 10);
  ctx.moveTo(x, y + 10); ctx.lineTo(x, y + 30);
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

  ctx.font = '10px JetBrains Mono, monospace';
  ctx.fillStyle = '#00ff88';
  ctx.fillText(label, x + 26, y - 10);

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
  ctx.arc(cx, cy, 35, 0, Math.PI * 2);
  ctx.stroke();

  ctx.font = '11px Orbitron, sans-serif';
  ctx.fillStyle = 'rgba(255, 170, 0, 0.85)';
  ctx.textAlign = 'center';
  ctx.fillText('SHOW HAND TO CAMERA', cx, cy + 55);

  ctx.restore();
}

function updateHandStatusBadge(text, className) {
  if (dom.handStatusBadge) {
    dom.handStatusBadge.textContent = text;
    dom.handStatusBadge.className = `hand-badge ${className}`;
  }
}

// ============================================================================
// Initialization on Page Load
// ============================================================================
window.addEventListener('DOMContentLoaded', () => {
  log('[SYSTEM] Initializing RoboHead Control Interface...', 'info');
  initThreeJS();
  setHeadPosition(90, 90, 'INIT');
  setupQAEngine();
  initHandTracking();
  log('[SYSTEM] Ready. Press W/A/S/D, Arrow keys, or click ✋ HAND TRACK to control with hand gestures.', 'rx');
});

