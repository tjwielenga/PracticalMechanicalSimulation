function finiteRange(values) {
  let low = Infinity;
  let high = -Infinity;
  for (const value of values) {
    if (!Number.isFinite(value)) continue;
    low = Math.min(low, value);
    high = Math.max(high, value);
  }
  if (!Number.isFinite(low)) return [-1, 1];
  if (low === high) {
    const padding = Math.max(Math.abs(low) * 0.05, 1e-6);
    return [low - padding, high + padding];
  }
  const padding = 0.06 * (high - low);
  return [low - padding, high + padding];
}

function numberLabel(value) {
  if (value === 0) return "0";
  if (Math.abs(value) >= 1e4 || Math.abs(value) < 1e-3) {
    return value.toExponential(3);
  }
  return value.toPrecision(5).replace(/\.?0+$/, "");
}

export class SignalPlot {
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d");
    this.xValues = [];
    this.yValues = [];
    this.xLabel = "";
    this.yLabel = "";
    this.sample = 0;
    this.resizeObserver = new ResizeObserver(() => this.draw());
    this.resizeObserver.observe(canvas);
  }

  setSignals(times, xSignal, ySignal, timeLabel = "time (s)") {
    this.xValues = xSignal?.values ?? times;
    this.yValues = ySignal?.values ?? [];
    this.xLabel = xSignal?.name ?? timeLabel;
    this.yLabel = ySignal?.name ?? "";
    this.sample = 0;
    this.draw();
  }

  setSample(sample) {
    this.sample = sample;
    this.draw();
  }

  draw() {
    const bounds = this.canvas.getBoundingClientRect();
    if (bounds.width < 2 || bounds.height < 2) return;
    const ratio = window.devicePixelRatio || 1;
    const width = Math.round(bounds.width * ratio);
    const height = Math.round(bounds.height * ratio);
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    const context = this.context;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, bounds.width, bounds.height);
    context.fillStyle = "#fff";
    context.fillRect(0, 0, bounds.width, bounds.height);
    if (this.xValues.length < 2 ||
        this.yValues.length !== this.xValues.length) {
      context.fillStyle = "#7a848f";
      context.font = "13px system-ui";
      context.fillText("Select a signal to plot", 18, 28);
      return;
    }

    const margin = { left: 78, right: 22, top: 16, bottom: 42 };
    const plotWidth = Math.max(bounds.width - margin.left - margin.right, 1);
    const plotHeight = Math.max(bounds.height - margin.top - margin.bottom, 1);
    const [xLow, xHigh] = finiteRange(this.xValues);
    const [yLow, yHigh] = finiteRange(this.yValues);
    const x = (value) => margin.left +
      ((value - xLow) / Math.max(xHigh - xLow, Number.EPSILON)) * plotWidth;
    const y = (value) => margin.top +
      (1 - (value - yLow) / (yHigh - yLow)) * plotHeight;

    context.strokeStyle = "#d6dce2";
    context.lineWidth = 1;
    context.fillStyle = "#64707c";
    context.font = "11px ui-monospace, monospace";
    context.textBaseline = "middle";
    context.textAlign = "right";
    for (let index = 0; index <= 4; index += 1) {
      const fraction = index / 4;
      const yy = margin.top + fraction * plotHeight;
      context.beginPath();
      context.moveTo(margin.left, yy);
      context.lineTo(margin.left + plotWidth, yy);
      context.stroke();
      const value = yHigh - fraction * (yHigh - yLow);
      context.fillText(numberLabel(value), margin.left - 8, yy);
    }

    context.textBaseline = "top";
    for (let index = 0; index <= 4; index += 1) {
      const fraction = index / 4;
      const xx = margin.left + fraction * plotWidth;
      context.beginPath();
      context.moveTo(xx, margin.top);
      context.lineTo(xx, margin.top + plotHeight);
      context.stroke();
      const value = xLow + fraction * (xHigh - xLow);
      context.textAlign = index === 0 ? "left" : index === 4 ? "right" : "center";
      context.fillText(numberLabel(value), xx, margin.top + plotHeight + 5);
    }

    context.strokeStyle = "#27678f";
    context.lineWidth = 1.8;
    context.beginPath();
    let drawing = false;
    for (let index = 0; index < this.xValues.length; index += 1) {
      const xValue = this.xValues[index];
      const yValue = this.yValues[index];
      if (!Number.isFinite(xValue) || !Number.isFinite(yValue)) {
        drawing = false;
        continue;
      }
      const xx = x(xValue);
      const yy = y(yValue);
      if (!drawing) context.moveTo(xx, yy);
      else context.lineTo(xx, yy);
      drawing = true;
    }
    context.stroke();

    const selected = Math.max(0,
      Math.min(this.sample, this.xValues.length - 1));
    if (Number.isFinite(this.xValues[selected]) &&
        Number.isFinite(this.yValues[selected])) {
      context.fillStyle = "#c24b37";
      context.beginPath();
      context.arc(x(this.xValues[selected]), y(this.yValues[selected]),
        3.8, 0, 2 * Math.PI);
      context.fill();
    }

    context.fillStyle = "#64707c";
    context.font = "11px system-ui";
    context.textAlign = "center";
    context.textBaseline = "bottom";
    context.fillText(this.xLabel, margin.left + plotWidth / 2,
      bounds.height - 3);
    context.save();
    context.translate(13, margin.top + plotHeight / 2);
    context.rotate(-Math.PI / 2);
    context.fillText(this.yLabel, 0, 0);
    context.restore();
  }
}
