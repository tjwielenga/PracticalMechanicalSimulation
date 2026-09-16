import assert from "node:assert/strict";
import test from "node:test";
import {
  prepareDocument,
  signalOptionLabel,
  validateDocument,
} from "../src/schema.js";

const valid = {
  format: "SimpView",
  version: 4,
  choices: [{ times: [0, 1], scene: {} }],
};

test("accepts the current viewer document", () => {
  assert.equal(validateDocument(valid), valid);
});

test("rejects another format or an empty result", () => {
  assert.throws(() => validateDocument({ ...valid, version: 3 }),
    /not a supported/);
  assert.throws(() => validateDocument({ ...valid, choices: [] }),
    /does not contain/);
});

test("formats hierarchical signal names for the selector", () => {
  assert.equal(signalOptionLabel("van.steering.pitman.theta"),
    "van / steering / pitman — theta");
  assert.equal(signalOptionLabel("energy"), "energy");
});

test("uses normalized pseudo-time for modal playback", () => {
  const modal = {
    ...valid,
    choice_name: "Mode",
    choices: [{ times: [0, 1.0e9, 2.0e9], scene: {} }],
  };
  const prepared = prepareDocument(modal);
  assert.deepEqual(prepared.choices[0].times, [0, 0.5, 1]);
  assert.equal(prepared.choices[0].time_label, "mode phase");
});
