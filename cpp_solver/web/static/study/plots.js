/* plots.js - canvas 2D plotting, written here on purpose.
 *
 * No chart library and no CDN: a line plot with labelled axes, ticks, a grid,
 * a legend and a hover readout, plus a complex plane for pole/zero and Nyquist
 * style output. Everything is driven by the data shape, never by a module name.
 */
(function (global) {
  'use strict';

  var Plots = {};

  var COLORS = ['#38bdf8', '#f472b6', '#4ade80', '#fbbf24', '#a78bfa', '#fb7185', '#2dd4bf', '#f97316'];
  var AXIS = '#64748b';
  var GRID = 'rgba(148, 163, 184, 0.18)';
  var TEXT = '#cbd5e1';
  var PAD = { left: 68, right: 18, top: 16, bottom: 44 };

  function dpr() { return global.devicePixelRatio || 1; }

  function prepare(canvas) {
    var ratio = dpr();
    var w = canvas.clientWidth || 640;
    var h = canvas.clientHeight || 280;
    if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
      canvas.width = Math.round(w * ratio);
      canvas.height = Math.round(h * ratio);
    }
    var ctx = canvas.getContext('2d');
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.font = '11px ui-monospace, Consolas, monospace';
    return { ctx: ctx, w: w, h: h };
  }

  function niceStep(span, target) {
    if (!(span > 0)) { return 1; }
    var raw = span / Math.max(1, target);
    var mag = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10));
    var norm = raw / mag;
    var mult = (norm < 1.5) ? 1 : (norm < 3) ? 2 : (norm < 7) ? 5 : 10;
    return mult * mag;
  }

  function tickLabel(v, step) {
    var a = Math.abs(v);
    if (a !== 0 && (a < 1e-3 || a >= 1e5)) { return v.toExponential(1); }
    var decimals = Math.max(0, Math.min(6, Math.ceil(-Math.log(step) / Math.LN10) + 1));
    return v.toFixed(decimals);
  }

  /** Normalise anything series-shaped into a list of {label, x[], y[]}. */
  Plots.normalise = function (value) {
    var out = [];
    function one(s, idx) {
      if (!s) { return; }
      if (Array.isArray(s) && Array.isArray(s[0])) {
        // [[x,y], ...] - accepted as a courtesy shape
        out.push({
          label: 'series ' + (idx + 1),
          x: s.map(function (p) { return Number(p[0]); }),
          y: s.map(function (p) { return Number(p[1]); })
        });
        return;
      }
      if (Array.isArray(s.y)) {
        var xs = Array.isArray(s.x) ? s.x.map(Number) : s.y.map(function (_, i) { return i; });
        out.push({ label: s.label || ('series ' + (idx + 1)), x: xs, y: s.y.map(Number) });
      }
    }
    if (Array.isArray(value)) { value.forEach(one); } else { one(value, 0); }
    return out.filter(function (s) { return s.y.length > 0; });
  };

  function bounds(series) {
    var b = { x0: Infinity, x1: -Infinity, y0: Infinity, y1: -Infinity };
    series.forEach(function (s) {
      for (var i = 0; i < s.y.length; i += 1) {
        var x = s.x[i];
        var y = s.y[i];
        if (isFinite(x)) { b.x0 = Math.min(b.x0, x); b.x1 = Math.max(b.x1, x); }
        if (isFinite(y)) { b.y0 = Math.min(b.y0, y); b.y1 = Math.max(b.y1, y); }
      }
    });
    if (!isFinite(b.x0)) { b.x0 = 0; b.x1 = 1; }
    if (!isFinite(b.y0)) { b.y0 = 0; b.y1 = 1; }
    if (b.x1 - b.x0 < 1e-12) { b.x0 -= 0.5; b.x1 += 0.5; }
    if (b.y1 - b.y0 < 1e-12) { b.y0 -= 0.5; b.y1 += 0.5; }
    var padY = (b.y1 - b.y0) * 0.08;          // never clip a curve at the frame
    b.y0 -= padY;
    b.y1 += padY;
    return b;
  }

  function frame(g, b, labels) {
    var ctx = g.ctx;
    var plotW = g.w - PAD.left - PAD.right;
    var plotH = g.h - PAD.top - PAD.bottom;
    var sx = function (x) { return PAD.left + (x - b.x0) / (b.x1 - b.x0) * plotW; };
    var sy = function (y) { return PAD.top + plotH - (y - b.y0) / (b.y1 - b.y0) * plotH; };

    var stepX = niceStep(b.x1 - b.x0, Math.max(2, Math.floor(plotW / 90)));
    var stepY = niceStep(b.y1 - b.y0, Math.max(2, Math.floor(plotH / 44)));

    ctx.lineWidth = 1;
    ctx.strokeStyle = GRID;
    ctx.fillStyle = TEXT;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    var first = Math.ceil(b.x0 / stepX) * stepX;
    for (var x = first; x <= b.x1 + stepX * 0.001; x += stepX) {
      var px = Math.round(sx(x)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(px, PAD.top);
      ctx.lineTo(px, PAD.top + plotH);
      ctx.stroke();
      ctx.fillText(tickLabel(x, stepX), px, PAD.top + plotH + 6);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    var firstY = Math.ceil(b.y0 / stepY) * stepY;
    for (var y = firstY; y <= b.y1 + stepY * 0.001; y += stepY) {
      var py = Math.round(sy(y)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(PAD.left, py);
      ctx.lineTo(PAD.left + plotW, py);
      ctx.stroke();
      ctx.fillText(tickLabel(y, stepY), PAD.left - 8, py);
    }

    ctx.strokeStyle = AXIS;
    ctx.strokeRect(PAD.left + 0.5, PAD.top + 0.5, plotW, plotH);
    if (b.y0 < 0 && b.y1 > 0) {
      ctx.strokeStyle = 'rgba(148,163,184,0.55)';
      ctx.beginPath();
      ctx.moveTo(PAD.left, Math.round(sy(0)) + 0.5);
      ctx.lineTo(PAD.left + plotW, Math.round(sy(0)) + 0.5);
      ctx.stroke();
    }

    ctx.fillStyle = TEXT;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    if (labels.x) { ctx.fillText(labels.x, PAD.left + plotW / 2, g.h - 4); }
    if (labels.y) {
      ctx.save();
      ctx.translate(12, PAD.top + plotH / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textBaseline = 'top';
      ctx.fillText(labels.y, 0, 0);
      ctx.restore();
    }
    return { sx: sx, sy: sy, plotW: plotW, plotH: plotH };
  }

  function legend(g, series) {
    if (series.length < 1) { return; }
    var ctx = g.ctx;
    var items = series.map(function (s, i) { return { label: s.label, color: COLORS[i % COLORS.length] }; });
    var width = 0;
    items.forEach(function (it) { width = Math.max(width, ctx.measureText(it.label).width); });
    var boxW = width + 30;
    var boxH = items.length * 16 + 8;
    var x = g.w - PAD.right - boxW - 4;
    var y = PAD.top + 4;
    ctx.fillStyle = 'rgba(15, 23, 42, 0.82)';
    ctx.strokeStyle = 'rgba(148,163,184,0.3)';
    ctx.beginPath();
    ctx.rect(x, y, boxW, boxH);
    ctx.fill();
    ctx.stroke();
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    items.forEach(function (it, i) {
      var cy = y + 12 + i * 16;
      ctx.strokeStyle = it.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x + 8, cy);
      ctx.lineTo(x + 22, cy);
      ctx.stroke();
      ctx.fillStyle = TEXT;
      ctx.fillText(it.label, x + 27, cy);
    });
  }

  function attachHover(canvas, draw) {
    if (canvas._hoverBound) { canvas._hoverDraw = draw; return; }
    canvas._hoverBound = true;
    canvas._hoverDraw = draw;
    canvas.addEventListener('mousemove', function (ev) {
      var r = canvas.getBoundingClientRect();
      canvas._hover = { x: ev.clientX - r.left, y: ev.clientY - r.top };
      if (canvas._hoverDraw) { canvas._hoverDraw(); }
    });
    canvas.addEventListener('mouseleave', function () {
      canvas._hover = null;
      if (canvas._hoverDraw) { canvas._hoverDraw(); }
    });
  }

  function readout(g, lines, at) {
    if (!lines.length) { return; }
    var ctx = g.ctx;
    var w = 0;
    lines.forEach(function (l) { w = Math.max(w, ctx.measureText(l).width); });
    var boxW = w + 16;
    var boxH = lines.length * 14 + 10;
    var x = Math.min(Math.max(at.x + 12, PAD.left), g.w - PAD.right - boxW);
    var y = Math.min(Math.max(at.y - boxH - 10, PAD.top), g.h - PAD.bottom - boxH);
    ctx.fillStyle = 'rgba(2, 6, 23, 0.92)';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(x, y, boxW, boxH);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#e2e8f0';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    lines.forEach(function (l, i) { ctx.fillText(l, x + 8, y + 5 + i * 14); });
  }

  /** Draw one or many curves. opts: {xLabel, yLabel, precision} */
  Plots.series = function (canvas, value, opts) {
    var options = opts || {};
    var series = Plots.normalise(value);
    function draw() {
      var g = prepare(canvas);
      if (!series.length) {
        g.ctx.fillStyle = TEXT;
        g.ctx.textAlign = 'center';
        g.ctx.fillText('no samples returned', g.w / 2, g.h / 2);
        return;
      }
      var b = bounds(series);
      var f = frame(g, b, { x: options.xLabel || 'x', y: options.yLabel || 'y' });
      var ctx = g.ctx;
      ctx.save();
      ctx.beginPath();
      ctx.rect(PAD.left, PAD.top, f.plotW, f.plotH);
      ctx.clip();
      series.forEach(function (s, i) {
        ctx.strokeStyle = COLORS[i % COLORS.length];
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        var started = false;
        for (var k = 0; k < s.y.length; k += 1) {
          var xv = s.x[k];
          var yv = s.y[k];
          if (!isFinite(xv) || !isFinite(yv)) { started = false; continue; }
          var px = f.sx(xv);
          var py = f.sy(yv);
          if (!started) { ctx.moveTo(px, py); started = true; } else { ctx.lineTo(px, py); }
        }
        ctx.stroke();
        if (s.y.length <= 60) {
          ctx.fillStyle = COLORS[i % COLORS.length];
          for (var m = 0; m < s.y.length; m += 1) {
            if (!isFinite(s.x[m]) || !isFinite(s.y[m])) { continue; }
            ctx.beginPath();
            ctx.arc(f.sx(s.x[m]), f.sy(s.y[m]), 2.2, 0, Math.PI * 2);
            ctx.fill();
          }
        }
      });
      ctx.restore();
      legend(g, series);

      var hover = canvas._hover;
      if (hover && hover.x > PAD.left && hover.x < PAD.left + f.plotW) {
        var xAt = b.x0 + (hover.x - PAD.left) / f.plotW * (b.x1 - b.x0);
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(Math.round(hover.x) + 0.5, PAD.top);
        ctx.lineTo(Math.round(hover.x) + 0.5, PAD.top + f.plotH);
        ctx.stroke();
        var lines = [(options.xLabel || 'x') + ' = ' + xAt.toPrecision(5)];
        series.forEach(function (s, i) {
          var best = 0;
          var bestD = Infinity;
          for (var k = 0; k < s.x.length; k += 1) {
            var d = Math.abs(s.x[k] - xAt);
            if (d < bestD) { bestD = d; best = k; }
          }
          ctx.fillStyle = COLORS[i % COLORS.length];
          ctx.beginPath();
          ctx.arc(f.sx(s.x[best]), f.sy(s.y[best]), 3.4, 0, Math.PI * 2);
          ctx.fill();
          lines.push(s.label + ' = ' + Number(s.y[best]).toPrecision(5));
        });
        readout(g, lines, hover);
      }
    }
    attachHover(canvas, draw);
    draw();
    return draw;
  };

  /** Poles, zeros, a Nyquist contour: the complex plane with the unit circle. */
  Plots.complex = function (canvas, value, opts) {
    var options = opts || {};
    var pts = (value || []).map(function (p) {
      if (Array.isArray(p)) { return { re: Number(p[0]), im: Number(p[1]) }; }
      return { re: Number(p.re), im: Number(p.im) };
    }).filter(function (p) { return isFinite(p.re) && isFinite(p.im); });

    function draw() {
      var g = prepare(canvas);
      var ctx = g.ctx;
      var maxR = 1.0;
      pts.forEach(function (p) { maxR = Math.max(maxR, Math.abs(p.re), Math.abs(p.im)); });
      var showCircle = maxR <= 20;             // only when the unit circle means something
      var lim = maxR * 1.25;
      var b = { x0: -lim, x1: lim, y0: -lim, y1: lim };
      var f = frame(g, b, { x: options.xLabel || 'Re', y: options.yLabel || 'Im' });

      ctx.save();
      ctx.beginPath();
      ctx.rect(PAD.left, PAD.top, f.plotW, f.plotH);
      ctx.clip();

      // real and imaginary axes through the origin
      ctx.strokeStyle = 'rgba(203, 213, 225, 0.55)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(PAD.left, f.sy(0));
      ctx.lineTo(PAD.left + f.plotW, f.sy(0));
      ctx.moveTo(f.sx(0), PAD.top);
      ctx.lineTo(f.sx(0), PAD.top + f.plotH);
      ctx.stroke();

      if (showCircle) {
        ctx.strokeStyle = COLORS[1];
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        var steps = 180;
        for (var i = 0; i <= steps; i += 1) {
          var a = i / steps * Math.PI * 2;
          var px = f.sx(Math.cos(a));
          var py = f.sy(Math.sin(a));
          if (i === 0) { ctx.moveTo(px, py); } else { ctx.lineTo(px, py); }
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }

      var connect = pts.length > 24;           // a contour, not a pole set
      if (connect) {
        ctx.strokeStyle = COLORS[0];
        ctx.lineWidth = 1.6;
        ctx.beginPath();
        pts.forEach(function (p, i) {
          if (i === 0) { ctx.moveTo(f.sx(p.re), f.sy(p.im)); } else { ctx.lineTo(f.sx(p.re), f.sy(p.im)); }
        });
        ctx.stroke();
      }
      ctx.strokeStyle = COLORS[0];
      ctx.lineWidth = 1.6;
      pts.forEach(function (p) {
        var px = f.sx(p.re);
        var py = f.sy(p.im);
        ctx.beginPath();
        ctx.moveTo(px - 4, py - 4);
        ctx.lineTo(px + 4, py + 4);
        ctx.moveTo(px + 4, py - 4);
        ctx.lineTo(px - 4, py + 4);
        ctx.stroke();
      });
      ctx.restore();

      legend(g, showCircle
        ? [{ label: 'samples (' + pts.length + ')' }, { label: 'unit circle' }]
        : [{ label: 'samples (' + pts.length + ')' }]);

      var hover = canvas._hover;
      if (hover) {
        var best = null;
        var bestD = Infinity;
        pts.forEach(function (p) {
          var d = Math.pow(f.sx(p.re) - hover.x, 2) + Math.pow(f.sy(p.im) - hover.y, 2);
          if (d < bestD) { bestD = d; best = p; }
        });
        if (best && bestD < 900) {
          var mag = Math.sqrt(best.re * best.re + best.im * best.im);
          readout(g, [
            're = ' + best.re.toPrecision(5),
            'im = ' + best.im.toPrecision(5),
            '|z| = ' + mag.toPrecision(5),
            'arg = ' + Math.atan2(best.im, best.re).toPrecision(5) + ' rad'
          ], hover);
        }
      }
    }
    attachHover(canvas, draw);
    draw();
    return draw;
  };

  Plots.colors = COLORS;
  global.Plots = Plots;
}(window));
