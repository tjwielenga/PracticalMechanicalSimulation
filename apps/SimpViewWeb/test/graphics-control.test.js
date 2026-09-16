import assert from "node:assert/strict";
import test from "node:test";
import {
  effectiveGraphicsScale,
  followTargetPath,
  graphicsPathKey,
  graphicsPathPrefixes,
  graphicsTree,
} from "../src/graphics-control.js";

test("builds graphics categories and assembly hierarchy", () => {
  const tree = graphicsTree([
    ["Forces", "Applied", "van", "gravity"],
    ["Forces", "Reactions", "van", "left_front", "spring"],
    ["Joints", "van", "left_front", "upper_arm"],
  ]);

  assert.equal(tree.name, "All graphics");
  assert.ok(tree.children.get("Forces").children.get("Applied"));
  assert.ok(tree.children.get("Forces").children.get("Reactions"));
  assert.ok(tree.children.get("Joints").children.get("van")
    .children.get("left_front"));
});

test("places follow bodies in the assembly hierarchy", () => {
  assert.deepEqual(followTargetPath("van.front_left.spindle"),
    ["Follow", "van", "front_left", "spindle"]);
});

test("multiplies local graphics scales through the selected hierarchy", () => {
  const scales = new Map([
    [graphicsPathKey([]), 2],
    [graphicsPathKey(["Forces"]), 10],
    [graphicsPathKey(["Forces", "Reactions"]), 0.5],
  ]);
  const path = ["Forces", "Reactions", "van", "spring"];

  assert.deepEqual(graphicsPathPrefixes(path).slice(0, 3),
    [[], ["Forces"], ["Forces", "Reactions"]]);
  assert.equal(effectiveGraphicsScale(path, scales), 10);
  scales.delete(graphicsPathKey(["Forces"]));
  assert.equal(effectiveGraphicsScale(path, scales), 1);
});
