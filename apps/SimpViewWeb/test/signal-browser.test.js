import assert from "node:assert/strict";
import test from "node:test";
import { optionTree, signalOptions } from "../src/signal-browser.js";

test("builds assembly and component hierarchy from signal names", () => {
  const options = signalOptions([
    { name: "van.steering.pitman.theta" },
    { name: "van.steering.cross_link.V_x" },
    { name: "van.chassis.R_z" },
  ], true);
  const tree = optionTree(options);

  assert.equal(tree.leaves[0].name, "time (s)");
  const van = tree.groups.get("van");
  assert.ok(van);
  assert.equal(van.groups.get("chassis").leaves[0].label, "R_z");
  const steering = van.groups.get("steering");
  assert.equal(steering.groups.get("pitman").leaves[0].label, "theta");
  assert.equal(steering.groups.get("cross_link").leaves[0].label, "V_x");
});

test("gives time a distinct X-axis option", () => {
  const options = signalOptions([{ name: "body.R_x" }], true);
  assert.deepEqual(options.map((option) => option.id), ["time", "signal:0"]);
  assert.equal(options[0].index, null);
});

test("labels a static history axis as samples", () => {
  const options = signalOptions([], true, "static history sample");
  assert.equal(options[0].id, "time");
  assert.equal(options[0].name, "static history sample");
});
