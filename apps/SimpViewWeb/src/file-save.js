function browserDownload(url, fileName, documentObject) {
  const link = documentObject.createElement("a");
  link.href = url;
  link.download = fileName;
  documentObject.body.append(link);
  link.click();
  link.remove();
}

export async function saveResultFile(url, fileName, environment = globalThis) {
  const picker = typeof environment.showSaveFilePicker === "function"
    ? environment.showSaveFilePicker.bind(environment) : null;
  if (!picker) {
    browserDownload(url, fileName, environment.document);
    return "download";
  }

  let handle;
  try {
    handle = await picker({
      suggestedName: fileName,
      types: [{
        description: "Simp simulation result",
        accept: { "application/x-hdf5": [".simp"] },
      }],
    });
  } catch (error) {
    if (error?.name === "AbortError") return "cancelled";
    throw error;
  }

  const response = await environment.fetch(url);
  if (!response.ok) {
    throw new Error(`The result could not be saved (${response.status}).`);
  }
  const writable = await handle.createWritable();
  try {
    await writable.write(await response.blob());
    await writable.close();
  } catch (error) {
    await writable.abort?.();
    throw error;
  }
  return "saved";
}
