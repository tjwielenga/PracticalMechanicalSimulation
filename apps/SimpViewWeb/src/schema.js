export function validateDocument(documentValue) {
  if (documentValue?.format !== "SimpView" || documentValue?.version !== 4) {
    throw new Error("This is not a supported SimpView document.");
  }
  if (!Array.isArray(documentValue.choices) || documentValue.choices.length < 1) {
    throw new Error("The SimpView document does not contain a result.");
  }
  for (const item of documentValue.choices) {
    if (!Array.isArray(item.times) || item.times.length < 1 || !item.scene) {
      throw new Error("A result choice has an invalid timeline or scene.");
    }
  }
  return documentValue;
}

export function prepareDocument(documentValue) {
  const document = validateDocument(documentValue);
  if (document.choice_name !== "Mode") return document;
  for (const choice of document.choices) {
    const last = Math.max(choice.times.length - 1, 1);
    choice.times = choice.times.map((_, index) => index / last);
    choice.time_label = "mode phase";
  }
  return document;
}

export function signalOptionLabel(name) {
  const parts = name.split(".");
  if (parts.length < 2) return name;
  return `${parts.slice(0, -1).join(" / ")} — ${parts.at(-1)}`;
}
