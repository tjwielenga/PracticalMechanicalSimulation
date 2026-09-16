import assert from "node:assert/strict";
import test from "node:test";
import { saveResultFile } from "../src/file-save.js";

test("uses a Save As picker for a result when available", async () => {
  const writes = [];
  let pickerOptions;
  const environment = {
    async showSaveFilePicker(options) {
      pickerOptions = options;
      return {
        async createWritable() {
          return {
            async write(value) { writes.push(value); },
            async close() { writes.push("closed"); },
          };
        },
      };
    },
    async fetch(url) {
      assert.equal(url, "/result");
      return { ok: true, async blob() { return "result bytes"; } };
    },
  };

  assert.equal(await saveResultFile("/result", "run.simp", environment),
    "saved");
  assert.equal(pickerOptions.suggestedName, "run.simp");
  assert.deepEqual(writes, ["result bytes", "closed"]);
});

test("does not download when the Save As picker is cancelled", async () => {
  const environment = {
    async showSaveFilePicker() {
      const error = new Error("cancelled");
      error.name = "AbortError";
      throw error;
    },
    async fetch() { throw new Error("fetch should not run"); },
  };
  assert.equal(await saveResultFile("/result", "run.simp", environment),
    "cancelled");
});

test("falls back to a browser download without a Save As picker", async () => {
  const events = [];
  const link = {
    click() { events.push("click"); },
    remove() { events.push("remove"); },
  };
  const environment = {
    document: {
      createElement(name) {
        assert.equal(name, "a");
        return link;
      },
      body: { append(value) { assert.equal(value, link); events.push("append"); } },
    },
  };

  assert.equal(await saveResultFile("/result", "run.simp", environment),
    "download");
  assert.equal(link.href, "/result");
  assert.equal(link.download, "run.simp");
  assert.deepEqual(events, ["append", "click", "remove"]);
});
