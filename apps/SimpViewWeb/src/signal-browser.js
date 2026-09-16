function signalPath(name) {
  return String(name).split(".").filter(Boolean);
}

export function signalOptions(signals, includeTime = false,
    timeLabel = "time (s)") {
  const options = signals.map((signal, index) => ({
    id: `signal:${index}`,
    index,
    name: signal.name,
    path: signalPath(signal.name),
  }));
  if (includeTime) {
    options.unshift({
      id: "time",
      index: null,
      name: timeLabel,
      path: [timeLabel],
    });
  }
  return options;
}

export function optionTree(options) {
  const root = { groups: new Map(), leaves: [] };
  for (const option of options) {
    let node = root;
    for (const part of option.path.slice(0, -1)) {
      if (!node.groups.has(part)) {
        node.groups.set(part, { groups: new Map(), leaves: [] });
      }
      node = node.groups.get(part);
    }
    node.leaves.push({ ...option, label: option.path.at(-1) });
  }
  return root;
}

function appendTree(parent, node, onSelect, expand = false) {
  for (const [name, child] of [...node.groups.entries()]
    .sort(([first], [second]) => first.localeCompare(second))) {
    const details = document.createElement("details");
    details.open = expand;
    const summary = document.createElement("summary");
    summary.textContent = name;
    details.append(summary);
    const contents = document.createElement("div");
    contents.className = "signal-tree-children";
    appendTree(contents, child, onSelect, expand);
    details.append(contents);
    parent.append(details);
  }
  for (const leaf of [...node.leaves]
    .sort((first, second) => first.label.localeCompare(second.label))) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "signal-leaf";
    button.dataset.optionId = leaf.id;
    button.textContent = leaf.label;
    button.title = leaf.name;
    button.addEventListener("click", () => onSelect(leaf));
    parent.append(button);
  }
}

export class SignalBrowser {
  constructor(root, { includeTime = false, onSelect = () => {} } = {}) {
    this.root = root;
    this.includeTime = includeTime;
    this.onSelect = onSelect;
    this.button = root.querySelector(".signal-browser-button");
    this.popup = root.querySelector(".signal-browser-popup");
    this.search = root.querySelector(".signal-browser-search");
    this.tree = root.querySelector(".signal-tree");
    this.options = [];
    this.selected = null;

    this.button.addEventListener("click", () => this.toggle());
    this.search.addEventListener("input", () => this.render());
    this.search.addEventListener("keydown", (event) => {
      if (event.key === "Escape") this.close();
    });
    document.addEventListener("pointerdown", (event) => {
      if (!this.root.contains(event.target)) this.close();
    });
  }

  setSignals(signals, preferredId = null, timeLabel = "time (s)") {
    this.options = signalOptions(signals, this.includeTime, timeLabel);
    const selected = this.options.find((option) => option.id === preferredId) ??
      this.options[0] ?? null;
    this.select(selected, false);
    this.button.disabled = this.options.length === 0;
    this.render();
  }

  select(option, notify = true) {
    if (!option) {
      this.selected = null;
      this.button.textContent = "No variables";
      return;
    }
    this.selected = option;
    this.button.textContent = option.name;
    this.button.title = option.name;
    this.close();
    this.renderSelection();
    if (notify) this.onSelect(option);
  }

  renderSelection() {
    for (const leaf of this.tree.querySelectorAll(".signal-leaf")) {
      leaf.classList.toggle("selected",
        leaf.dataset.optionId === this.selected?.id);
    }
  }

  render() {
    const query = this.search.value.trim().toLowerCase();
    const visible = query
      ? this.options.filter((option) => option.name.toLowerCase().includes(query))
      : this.options;
    this.tree.replaceChildren();
    if (visible.length === 0) {
      const empty = document.createElement("div");
      empty.className = "signal-tree-empty";
      empty.textContent = "No matching variables";
      this.tree.append(empty);
      return;
    }
    appendTree(this.tree, optionTree(visible), (option) => this.select(option),
      query.length > 0);
    this.renderSelection();
  }

  toggle() {
    if (this.popup.hidden) this.open();
    else this.close();
  }

  open() {
    this.popup.hidden = false;
    this.button.setAttribute("aria-expanded", "true");
    this.search.focus();
    this.search.select();
  }

  close() {
    this.popup.hidden = true;
    this.button.setAttribute("aria-expanded", "false");
  }
}
