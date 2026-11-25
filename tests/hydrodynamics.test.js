const assert = require('node:assert/strict');
const test = require('node:test');
const { AUVHydrodynamics2D } = require('../simulasi_auv/hydrodynamics.js');

test('a fresh model keeps thrusters neutral at the surface until depth hold is requested', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.reset(1.0, 0.0);

  const state = model.step(1 / 60, { pulses: [1500, 1500, 1500, 1500, 1500, 1500] });

  assert.equal(model.rlEnabled, false);
  assert.equal(model.pid.targetZ, 0);
  assert.equal(state.depthHoldActive, false);
  assert.deepEqual(state.motorPulses, [1500, 1500, 1500, 1500, 1500, 1500]);
});

test('depth hold generates visible T5/T6 motor pulses matching its thrust', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.pid.targetZ = 0.75;

  const state = model.step(1 / 60, { pulses: [1500, 1500, 1500, 1500, 1500, 1500] });
  const allocation = model.thrustAllocation(state.motorPulses);

  assert.equal(state.depthHoldActive, true);
  assert.ok(state.motorPulses[4] > 1500);
  assert.ok(state.motorPulses[5] < 1500);
  assert.ok(Math.abs(allocation.tauZ - state.commandThrustZ) < 0.2);
  assert.ok(state.thrustZ > 0);
  assert.ok(state.thrustZ < state.commandThrustZ);
});

test('depth hold settles near the selected depth without overshoot', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.pid.targetZ = 0.5;

  let state;
  let maximumDepth = 0;
  for (let i = 0; i < 600; i++) {
    state = model.step(1 / 60, {
      pulses: [1500, 1500, 1500, 1500, 1500, 1500]
    });
    maximumDepth = Math.max(maximumDepth, state.depthM);
  }

  assert.ok(Math.abs(state.depthM - 0.5) < 0.03);
  assert.ok(maximumDepth < 0.55);
});

test('vertical thruster imbalance creates physical roll', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;

  let state;
  for (let i = 0; i < 30; i++) {
    state = model.step(1 / 60, {
      pulses: [1500, 1500, 1500, 1500, 1680, 1500]
    });
  }

  assert.ok(state.rollDeg > 0);
  assert.ok(state.thrustRoll > 0);
});

test('dive and surface pulses agree with the firmware thruster polarities', () => {
  const model = new AUVHydrodynamics2D();
  const dive = model.thrustAllocation([1500, 1500, 1500, 1500, 1680, 1320]);
  const surface = model.thrustAllocation([1500, 1500, 1500, 1500, 1320, 1680]);

  assert.ok(dive.tauZ > 0);
  assert.ok(surface.tauZ < 0);
  assert.ok(Math.abs(dive.tauRoll) < 1e-9);
  assert.ok(Math.abs(surface.tauRoll) < 1e-9);
});

test('horizontal mission commands compensate motor polarity for surge, yaw, and sway', () => {
  const model = new AUVHydrodynamics2D();
  const forward = model.horizontalCommandPulses('MAJU', 100);
  const backward = model.horizontalCommandPulses('MUNDUR', 100);
  const turnRight = model.horizontalCommandPulses('PUTAR', 100);
  const turnLeft = model.horizontalCommandPulses('TURN_LEFT', 100);
  const swayRight = model.horizontalCommandPulses('KANAN', 100);
  const swayLeft = model.horizontalCommandPulses('KIRI', 100);

  assert.deepEqual(forward, [1400, 1400, 1600, 1600, 1500, 1500]);
  assert.deepEqual(backward, [1600, 1600, 1400, 1400, 1500, 1500]);
  assert.ok(model.thrustAllocation(forward).tauX > 0);
  assert.ok(model.thrustAllocation(backward).tauX < 0);
  assert.ok(model.thrustAllocation(turnRight).tauYaw > 0);
  assert.ok(model.thrustAllocation(turnLeft).tauYaw < 0);
  assert.ok(model.thrustAllocation(swayRight).tauY > 0);
  assert.ok(model.thrustAllocation(swayLeft).tauY < 0);
});

test('mission surge, yaw, and sway commands move the vehicle on their intended axes', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.reset(1.0, 0.5);

  let state;
  const forward = model.horizontalCommandPulses('MAJU', 150);
  for (let i = 0; i < 120; i++) {
    state = model.step(1 / 60, { pulses: forward });
  }
  assert.ok(state.xM > 1.0);

  model.reset(1.0, 0.5);
  const turn = model.horizontalCommandPulses('PUTAR', 150);
  for (let i = 0; i < 120; i++) {
    state = model.step(1 / 60, { pulses: turn });
  }
  assert.ok(state.yawDeg > 0);

  model.reset(1.0, 0.5);
  const sway = model.horizontalCommandPulses('KANAN', 150);
  for (let i = 0; i < 120; i++) {
    state = model.step(1 / 60, { pulses: sway });
  }
  assert.ok(state.yM > 0);
});

test('thruster allocation follows updated reverse and deadband configuration', () => {
  const model = new AUVHydrodynamics2D();
  const pulses = [1600, 1500, 1500, 1500, 1500, 1500];
  const defaultThrust = model.thrustAllocation(pulses).tauX;

  model.setThrusterReverse([0, 1, 0, 0, 0, 1]);
  const configuredThrust = model.thrustAllocation(pulses).tauX;
  assert.ok(defaultThrust < 0);
  assert.ok(configuredThrust > 0);

  model.setDeadbandUs(30);
  assert.equal(model.pwmToThrust(1530, 0), 0);
  assert.ok(model.pwmToThrust(1531, 0) > 0);
});

test('physics setters accept the complete simulator parameter ranges', () => {
  const model = new AUVHydrodynamics2D();
  model.setMass(50);
  model.setNetBuoyancy(-1000);
  model.setDragScale(5);

  assert.equal(model.m, 50);
  assert.equal(model.netBuoyancyGrams, -1000);
  assert.equal(model.dragScale, 5);
  assert.ok(Number.isFinite(model.getPhysicsSummary().naturalPeriodSec));
});

test('pitch recovers through buoyancy without a hidden motor controller', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.rlEnabled = false;
  model.pid.targetZ = 0;
  model.reset(1.0, 0.5, 0.2);

  const state = model.step(1 / 60, {
    pulses: [1500, 1500, 1500, 1500, 1500, 1500]
  });

  assert.ok(state.pitchRad < 0.2);
  assert.equal(state.thrustQ, 0);
  assert.deepEqual(state.motorPulses, [1500, 1500, 1500, 1500, 1500, 1500]);
});

test('depth limit blocks downward travel but still allows surfacing', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.rlEnabled = false;
  model.pid.targetZ = 0;
  model.reset(1.0, 1.39);

  for (let i = 0; i < 300; i++) {
    model.step(1 / 60, {
      pulses: [1500, 1500, 1500, 1500, 1680, 1320]
    });
  }

  assert.equal(model.eta[1], 1.4);
  assert.equal(model.nu[1], 0);

  for (let i = 0; i < 60; i++) {
    model.step(1 / 60, {
      pulses: [1500, 1500, 1500, 1500, 1320, 1680]
    });
  }

  assert.ok(model.eta[1] < 1.4);
});

test('keeps the complete hull clear of both pool end walls', () => {
  const model = new AUVHydrodynamics2D();
  model.waveActive = false;
  model.currentVelocityX = 0;
  model.rlEnabled = false;
  model.pid.targetZ = 0;
  const endClearance = 0.08 + model.L / 2.0;
  const minX = endClearance;
  const maxX = 12.0 - endClearance;
  const neutralPulses = [1500, 1500, 1500, 1500, 1500, 1500];

  model.reset(maxX - 0.01, 0.5);
  model.nu[0] = 1.5;
  let state = model.step(1 / 60, { pulses: neutralPulses });
  assert.equal(state.xM, maxX);
  assert.equal(state.u, 0);

  model.reset(minX + 0.01, 0.5);
  model.nu[0] = -1.5;
  state = model.step(1 / 60, { pulses: neutralPulses });
  assert.equal(state.xM, minX);
  assert.equal(state.u, 0);
});

test('neutral thrusters keep the AUV stationary in the default calm water', () => {
  const model = new AUVHydrodynamics2D();
  model.rlEnabled = false;
  model.pid.targetZ = 0;
  model.reset(6.0, 0.5);

  let state;
  for (let i = 0; i < 300; i++) {
    state = model.step(1 / 60, {
      pulses: [1500, 1500, 1500, 1500, 1500, 1500]
    });
  }

  assert.equal(model.currentVelocityX, 0);
  assert.equal(model.waveActive, false);
  assert.ok(Math.abs(state.xM - 6.0) < 1e-6);
});
