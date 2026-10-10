/* panels.js - controls and readouts, generated from describe() alone.
 *
 * Every control comes from a params[] entry and every readout from an
 * outputs[] entry, so a module nobody wrote yet already has a working panel.
 */
(function (global) {
  'use strict';

  var D = global.Describe;
  var Panels = {};

  function el(tag, cls, text) {
    var node = document.createElement(tag);
    if (cls) { node.className = cls; }
    if (text !== undefined && text !== null) { node.textContent = text; }
    return node;
  }

  function unitSuffix(unit) {
    return unit ? ' ' + unit : '';
  }

  function stepFor(param, range) {
    if (param.type === 'int') { return 1; }
    var span = range.max - range.min;
    var raw = span / 500;
    var mag = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10));
    return Math.max(mag, 1e-6);
  }

  /* ======================================================= panel header === */

  Panels.renderHeader = function (host, entry, meta) {
    host.innerHTML = '';
    var module = entry.module;
    var op = entry.op;

    var top = el('div', 'panel-head');
    var titles = el('div', 'panel-titles');
    titles.appendChild(el('h2', null, op.title || op.name));
    var sub = el('div', 'panel-sub');
    sub.appendChild(el('span', 'mono', module.name + '.' + op.name));
    (module.topics || []).forEach(function (t) { sub.appendChild(el('span', 'topic', t)); });
    titles.appendChild(sub);
    top.appendChild(titles);

    var badges = el('div', 'panel-badges');
    var course = module.course || {};
    var url = D.courseUrl(course);
    var badge;
    if (url) {
      badge = el('a', 'badge course');
      badge.href = url;
      badge.target = '_blank';
      badge.rel = 'noopener';
    } else {
      badge = el('span', 'badge course');
    }
    badge.textContent = [course.code, course.name].filter(Boolean).join(' · ') || 'course';
    badge.title = 'Moodle course ' + (course.id === undefined ? '?' : course.id);
    badges.appendChild(badge);

    var timing = el('span', 'badge timing');
    timing.textContent = (meta && typeof meta.us === 'number')
      ? ('C++ ' + meta.us + ' us')
      : 'C++ - us';
    timing.title = 'Microseconds measured inside the C++ op on the last call';
    badges.appendChild(timing);

    var prec = el('select', 'prec-select');
    [2, 4, 6, 9, 15].forEach(function (p) {
      var o = el('option', null, p + ' digits');
      o.value = String(p);
      if (meta && meta.precision === p) { o.selected = true; }
      prec.appendChild(o);
    });
    prec.title = 'Displayed precision';
    prec.addEventListener('change', function () {
      if (meta && meta.onPrecision) { meta.onPrecision(parseInt(prec.value, 10)); }
    });
    badges.appendChild(prec);
    top.appendChild(badges);
    host.appendChild(top);

    if (op.formula) {
      var f = el('div', 'formula');
      f.innerHTML = D.latex(op.formula);
      f.title = op.formula;
      host.appendChild(f);
    }
    if (op.explain) { host.appendChild(el('p', 'explain', op.explain)); }
    if (module.summary) { host.appendChild(el('p', 'summary', module.summary)); }

    var src = el('div', 'source');
    src.appendChild(el('span', 'source-label', 'source'));
    src.appendChild(el('code', null, module.source || 'unknown'));
    host.appendChild(src);
  };

  /* ============================================================ controls == */

  function numberBox(value, step, onInput) {
    var box = el('input', 'num');
    box.type = 'number';
    box.step = String(step);
    box.value = String(value);
    box.addEventListener('input', function () {
      var v = parseFloat(box.value);
      if (isFinite(v)) { onInput(v); }
    });
    return box;
  }

  function sliderPair(value, range, step, onInput) {
    var wrap = el('div', 'slider-row');
    var slider = el('input', 'slider');
    slider.type = 'range';
    slider.min = String(range.min);
    slider.max = String(range.max);
    slider.step = String(step);
    slider.value = String(value);
    var box = numberBox(value, step, function (v) {
      slider.value = String(v);
      onInput(v);
    });
    slider.addEventListener('input', function () {
      var v = parseFloat(slider.value);
      box.value = String(v);
      onInput(v);
    });
    wrap.appendChild(slider);
    wrap.appendChild(box);
    return wrap;
  }

  function controlShell(param, extraClass) {
    var wrap = el('div', 'ctl' + (extraClass ? ' ' + extraClass : ''));
    var head = el('div', 'ctl-head');
    head.appendChild(el('span', 'ctl-label', param.label || param.name));
    var meta = el('span', 'ctl-meta');
    meta.textContent = param.type + (param.unit ? ' · ' + param.unit : '');
    head.appendChild(meta);
    wrap.appendChild(head);
    return wrap;
  }

  var COMPONENTS = { 3: ['x', 'y', 'z'], 6: ['1', '2', '3', '4', '5', '6'] };

  function buildControl(param, value, commit) {
    var range = D.paramRange(param);
    var step = stepFor(param, range);

    if (param.type === 'bool') {
      var w = controlShell(param, 'ctl-bool');
      var lab = el('label', 'check');
      var box = el('input');
      box.type = 'checkbox';
      box.checked = !!value;
      box.addEventListener('change', function () { commit(box.checked); });
      lab.appendChild(box);
      lab.appendChild(el('span', null, param.label || param.name));
      w.appendChild(lab);
      return w;
    }

    if (param.type === 'enum') {
      var we = controlShell(param, 'ctl-enum');
      var sel = el('select', 'enum');
      (param.options || []).forEach(function (opt) {
        var o = el('option', null, String(opt));
        o.value = String(opt);
        if (String(opt) === String(value)) { o.selected = true; }
        sel.appendChild(o);
      });
      sel.addEventListener('change', function () { commit(sel.value); });
      we.appendChild(sel);
      return we;
    }

    if (param.type === 'int') {
      var wi = controlShell(param, 'ctl-int');
      var row = el('div', 'stepper');
      var minus = el('button', 'step-btn', '-');
      var box2 = numberBox(value, 1, function (v) { commit(Math.round(v)); });
      var plus = el('button', 'step-btn', '+');
      function bump(d) {
        var v = Math.round(parseFloat(box2.value) || 0) + d;
        if (typeof param.min === 'number') { v = Math.max(param.min, v); }
        if (typeof param.max === 'number') { v = Math.min(param.max, v); }
        box2.value = String(v);
        commit(v);
      }
      minus.addEventListener('click', function () { bump(-1); });
      plus.addEventListener('click', function () { bump(1); });
      row.appendChild(minus);
      row.appendChild(box2);
      row.appendChild(plus);
      wi.appendChild(row);
      return wi;
    }

    if (param.type === 'scalar') {
      var ws = controlShell(param, 'ctl-scalar');
      var readout = el('span', 'ctl-value', D.fmt(value, 4) + unitSuffix(param.unit));
      ws.querySelector('.ctl-head').appendChild(readout);
      ws.appendChild(sliderPair(Number(value) || 0, range, step, function (v) {
        readout.textContent = D.fmt(v, 4) + unitSuffix(param.unit);
        commit(v);
      }));
      return ws;
    }

    if (param.type === 'vec3' || param.type === 'vec6' || (param.type === 'points' && !D.isMatrix(value))) {
      var n = D.paramLength(param) || (Array.isArray(value) ? value.length : 3);
      var wv = controlShell(param, 'ctl-vec');
      var vec = (Array.isArray(value) ? value.slice() : []).map(Number);
      while (vec.length < n) { vec.push(0); }
      var names = COMPONENTS[n] || vec.map(function (_, i) { return String(i + 1); });
      vec.forEach(function (v, i) {
        var line = el('div', 'vec-line');
        line.appendChild(el('span', 'vec-name', names[i]));
        var out = el('span', 'vec-value', D.fmt(v, 4));
        line.appendChild(sliderPair(v, range, step, function (nv) {
          vec[i] = nv;
          out.textContent = D.fmt(nv, 4);
          commit(vec.slice());
        }));
        line.appendChild(out);
        wv.appendChild(line);
      });
      return wv;
    }

    if (param.type === 'text') {
      var wt = controlShell(param, 'ctl-text');
      var input = el('input', 'textbox');
      input.type = 'text';
      input.value = (value === undefined || value === null) ? '' : String(value);
      input.addEventListener('input', function () { commit(input.value); });
      wt.appendChild(input);
      return wt;
    }

    if (D.isTable(value) || param.type === 'table') {
      return tableControl(param, D.isTable(value) ? value : { columns: ['value'], rows: [[0]] }, commit);
    }

    if (D.isMatrix(value) || param.type === 'mat3' || param.type === 'mat4' || param.type === 'matrix'
        || param.type === 'complex_set' || param.type === 'points') {
      var rows = D.isMatrix(value) ? value : D.defaultValue(param);
      return gridControl(param, rows, commit, param.type === 'matrix' || param.type === 'points'
        || param.type === 'complex_set');
    }

    if (value && typeof value === 'object') { return structuredControl(param, value, commit); }

    // last resort: a value of a type this build has never seen - still editable
    var wj = controlShell(param, 'ctl-json');
    var area = el('textarea', 'json');
    area.value = JSON.stringify(value);
    area.addEventListener('change', function () {
      try { commit(JSON.parse(area.value)); } catch (err) { area.classList.add('bad'); }
    });
    wj.appendChild(area);
    return wj;
  }

  function cellEditor(value, onCommit) {
    if (typeof value === 'boolean') {
      var box = el('input', 'cell-bool');
      box.type = 'checkbox';
      box.checked = value;
      box.addEventListener('change', function () { onCommit(box.checked); });
      return box;
    }
    if (typeof value === 'string') {
      var text = el('input', 'cell-text');
      text.type = 'text';
      text.value = value;
      text.addEventListener('input', function () { onCommit(text.value); });
      return text;
    }
    var num = el('input', 'cell-num');
    num.type = 'number';
    num.step = 'any';
    num.value = String(value === undefined || value === null ? 0 : value);
    num.addEventListener('input', function () {
      var v = parseFloat(num.value);
      if (isFinite(v)) { onCommit(v); }
    });
    return num;
  }

  function gridControl(param, rows, commit, resizable) {
    var wrap = controlShell(param, 'ctl-grid');
    var data = rows.map(function (r) { return r.slice(); });
    var table = el('table', 'edit-grid');
    var body = el('tbody');

    function rebuild() {
      body.innerHTML = '';
      data.forEach(function (row, i) {
        var tr = el('tr');
        row.forEach(function (cell, j) {
          var td = el('td');
          td.appendChild(cellEditor(cell, function (v) { data[i][j] = v; commit(D.clone(data)); }));
          tr.appendChild(td);
        });
        body.appendChild(tr);
      });
    }
    rebuild();
    table.appendChild(body);
    wrap.appendChild(table);

    if (resizable) {
      var bar = el('div', 'grid-bar');
      var add = el('button', 'step-btn', '+ row');
      add.addEventListener('click', function () {
        var width = data.length ? data[0].length : 1;
        var fresh = [];
        for (var k = 0; k < width; k += 1) { fresh.push(0); }
        data.push(fresh);
        rebuild();
        commit(D.clone(data));
      });
      var drop = el('button', 'step-btn', '- row');
      drop.addEventListener('click', function () {
        if (data.length > 1) { data.pop(); rebuild(); commit(D.clone(data)); }
      });
      bar.appendChild(add);
      bar.appendChild(drop);
      wrap.appendChild(bar);
    }
    return wrap;
  }

  function tableControl(param, value, commit) {
    var wrap = controlShell(param, 'ctl-grid');
    var columns = (value.columns || []).slice();
    var data = (value.rows || []).map(function (r) { return r.slice(); });
    var table = el('table', 'edit-grid');
    var head = el('thead');
    var hr = el('tr');
    columns.forEach(function (c) { hr.appendChild(el('th', null, String(c))); });
    head.appendChild(hr);
    var body = el('tbody');

    function send() { commit({ columns: columns.slice(), rows: D.clone(data) }); }

    function rebuild() {
      body.innerHTML = '';
      data.forEach(function (row, i) {
        var tr = el('tr');
        columns.forEach(function (_, j) {
          var td = el('td');
          td.appendChild(cellEditor(row[j], function (v) { data[i][j] = v; send(); }));
          tr.appendChild(td);
        });
        body.appendChild(tr);
      });
    }
    rebuild();
    table.appendChild(head);
    table.appendChild(body);
    wrap.appendChild(table);

    var bar = el('div', 'grid-bar');
    var add = el('button', 'step-btn', '+ row');
    add.addEventListener('click', function () {
      var template = data.length ? data[data.length - 1] : columns.map(function () { return 0; });
      data.push(template.map(function (c) { return (typeof c === 'string') ? '' : (typeof c === 'boolean' ? false : 0); }));
      rebuild();
      send();
    });
    var drop = el('button', 'step-btn', '- row');
    drop.addEventListener('click', function () {
      if (data.length > 1) { data.pop(); rebuild(); send(); }
    });
    bar.appendChild(add);
    bar.appendChild(drop);
    wrap.appendChild(bar);
    return wrap;
  }

  function structuredControl(param, value, commit) {
    var wrap = controlShell(param, 'ctl-struct');
    var data = D.clone(value);
    Object.keys(data).forEach(function (key) {
      var line = el('div', 'struct-line');
      line.appendChild(el('span', 'struct-key', key));
      var v = data[key];
      if (v !== null && typeof v === 'object') {
        var area = el('textarea', 'json');
        area.value = JSON.stringify(v);
        area.addEventListener('change', function () {
          try {
            data[key] = JSON.parse(area.value);
            area.classList.remove('bad');
            commit(D.clone(data));
          } catch (err) { area.classList.add('bad'); }
        });
        line.appendChild(area);
      } else {
        line.appendChild(cellEditor(v, function (nv) { data[key] = nv; commit(D.clone(data)); }));
      }
      wrap.appendChild(line);
    });
    return wrap;
  }

  /** Build the whole control stack for an op. Returns an error sink so a
   *  failing call can report next to the control that caused it. */
  Panels.renderControls = function (host, op, args, onChange) {
    host.innerHTML = '';
    var slots = {};
    (op.params || []).forEach(function (param) {
      var holder = el('div', 'ctl-holder');
      var control = buildControl(param, args[param.name], function (value) {
        args[param.name] = value;
        onChange(param.name, value);
      });
      holder.appendChild(control);
      var err = el('div', 'ctl-error');
      err.style.display = 'none';
      holder.appendChild(err);
      host.appendChild(holder);
      slots[param.name] = err;
    });
    if (!(op.params || []).length) {
      host.appendChild(el('p', 'muted', 'This op takes no parameters.'));
    }
    return {
      clear: function () {
        Object.keys(slots).forEach(function (k) {
          slots[k].style.display = 'none';
          slots[k].textContent = '';
        });
      },
      show: function (message) {
        var names = Object.keys(slots);
        var lower = String(message).toLowerCase();
        var hit = names.filter(function (n) { return lower.indexOf(n.toLowerCase()) >= 0; });
        var targets = hit.length ? hit : names;
        if (!targets.length) { return false; }
        slots[targets[0]].textContent = message;
        slots[targets[0]].style.display = '';
        return hit.length > 0;
      }
    };
  };

  /* ============================================================= outputs == */

  function matrixNode(rows, precision) {
    var table = el('table', 'mat');
    var body = el('tbody');
    rows.forEach(function (row) {
      var tr = el('tr');
      (Array.isArray(row) ? row : [row]).forEach(function (cell) {
        tr.appendChild(el('td', null, D.fmt(cell, precision)));
      });
      body.appendChild(tr);
    });
    table.appendChild(body);
    var wrap = el('div', 'mat-wrap');
    wrap.appendChild(table);
    return wrap;
  }

  function tableNode(value, precision) {
    var table = el('table', 'data-table');
    var head = el('thead');
    var hr = el('tr');
    (value.columns || []).forEach(function (c) { hr.appendChild(el('th', null, String(c))); });
    head.appendChild(hr);
    var body = el('tbody');
    (value.rows || []).forEach(function (row) {
      var tr = el('tr');
      (Array.isArray(row) ? row : [row]).forEach(function (cell) {
        var td = el('td', typeof cell === 'number' ? 'num-cell' : null, D.fmt(cell, precision));
        tr.appendChild(td);
      });
      body.appendChild(tr);
    });
    table.appendChild(head);
    table.appendChild(body);
    var wrap = el('div', 'table-wrap');
    wrap.appendChild(table);
    return wrap;
  }

  function canvasCard(height) {
    var holder = el('div', 'plot-holder');
    var canvas = el('canvas', 'plot');
    holder.style.height = height + 'px';
    holder.appendChild(canvas);
    return { holder: holder, canvas: canvas };
  }

  function boundsText(points) {
    var lo = [Infinity, Infinity, Infinity];
    var hi = [-Infinity, -Infinity, -Infinity];
    points.forEach(function (p) {
      for (var i = 0; i < 3; i += 1) {
        lo[i] = Math.min(lo[i], Number(p[i]));
        hi[i] = Math.max(hi[i], Number(p[i]));
      }
    });
    if (!isFinite(lo[0])) { return 'empty'; }
    return lo.map(function (v, i) {
      return 'xyz'[i] + ' ' + v.toFixed(3) + '..' + hi[i].toFixed(3);
    }).join('   ');
  }

  /** Render every output of a reply. Returns the 3D payload it found. */
  Panels.renderOutputs = function (host, op, result, opts) {
    host.innerHTML = '';
    var precision = (opts && opts.precision) || 4;
    var scene = { frames: [], points: [], labels: [] };
    var deferred = [];
    var declared = op.outputs || [];
    var seen = {};
    var list = declared.slice();
    Object.keys(result || {}).forEach(function (key) {
      if (!declared.some(function (o) { return o.name === key; })) {
        list.push({ name: key, type: null, label: key });   // undeclared extra, still shown
      }
    });

    var readouts = el('div', 'readout-row');

    list.forEach(function (out) {
      if (seen[out.name]) { return; }
      seen[out.name] = true;
      var value = (result || {})[out.name];
      if (value === undefined) { return; }
      var type = out.type;
      if (!type) {
        if (D.isTable(value)) { type = 'table'; } else if (D.isMatrix(value)) { type = 'matrix'; } else if (typeof value === 'boolean') { type = 'bool'; } else if (typeof value === 'string') { type = 'text'; } else if (typeof value === 'number') { type = 'scalar'; } else if (Array.isArray(value)) { type = 'vec3'; } else { type = 'json'; }
      }

      if (type === 'scalar' || type === 'int' || type === 'bool') {
        var chip = el('div', 'readout');
        chip.appendChild(el('span', 'readout-label', out.label || out.name));
        var txt = (type === 'bool')
          ? (value ? 'true' : 'false')
          : (type === 'int' ? String(value) : D.fmt(value, precision));
        var v = el('span', 'readout-value' + (type === 'bool' ? (value ? ' good' : ' warn') : ''), txt + unitSuffix(out.unit));
        chip.appendChild(v);
        readouts.appendChild(chip);
        return;
      }

      var card = el('div', 'out-card');
      var head = el('div', 'out-head');
      head.appendChild(el('span', 'out-label', out.label || out.name));
      head.appendChild(el('span', 'out-type', (out.type || type) + (out.unit ? ' · ' + out.unit : '')));
      card.appendChild(head);

      if (type === 'text') {
        card.appendChild(el('p', 'out-text', String(value)));
      } else if (type === 'vec3' || type === 'vec6') {
        var vecNode = el('div', 'vec-out');
        (Array.isArray(value) ? value : [value]).forEach(function (c, i) {
          var cellWrap = el('div', 'vec-cell');
          cellWrap.appendChild(el('span', 'vec-cell-name', (COMPONENTS[Array.isArray(value) ? value.length : 3] || [])[i] || String(i + 1)));
          cellWrap.appendChild(el('span', 'vec-cell-value', D.fmt(c, precision)));
          vecNode.appendChild(cellWrap);
        });
        card.appendChild(vecNode);
      } else if (type === 'mat3' || type === 'mat4' || type === 'matrix') {
        card.appendChild(matrixNode(value, precision));
        if (type === 'mat4' && D.isMatrix(value) && value.length === 4) {
          scene.frames.push(value);
          scene.labels.push(out.label || out.name);
        }
      } else if (type === 'mat4_set') {
        var setWrap = el('div', 'mat-set');
        (value || []).forEach(function (m, i) {
          var item = el('div', 'mat-set-item');
          item.appendChild(el('div', 'mat-set-index', '#' + (i + 1)));
          item.appendChild(matrixNode(m, precision));
          setWrap.appendChild(item);
          scene.frames.push(m);
          scene.labels.push((out.label || out.name) + ' #' + (i + 1));
        });
        card.appendChild(setWrap);
        card.appendChild(el('p', 'muted', (value || []).length + ' frames drawn as RGB axis triads in the 3D view above.'));
      } else if (type === 'table') {
        card.appendChild(D.isTable(value) ? tableNode(value, precision) : matrixNode(value, precision));
      } else if (type === 'series' || type === 'series_set') {
        var plot = canvasCard(280);
        card.appendChild(plot.holder);
        var seriesList = global.Plots.normalise(value);
        var xUnit = out.x_unit || out.xUnit || '';
        var yUnit = out.unit || '';
        deferred.push(function () {
          global.Plots.series(plot.canvas, value, {
            xLabel: (out.x_label || out.xLabel || 'x') + (xUnit ? ' [' + xUnit + ']' : ''),
            yLabel: (out.label || out.name) + (yUnit ? ' [' + yUnit + ']' : '')
          });
        });
        card.appendChild(el('p', 'muted', seriesList.length + ' curve(s), '
          + seriesList.reduce(function (a, s) { return a + s.y.length; }, 0) + ' samples. Hover for values.'));
      } else if (type === 'complex_set') {
        var cplot = canvasCard(300);
        card.appendChild(cplot.holder);
        deferred.push(function () {
          global.Plots.complex(cplot.canvas, value, { xLabel: 'Re', yLabel: 'Im' });
        });
        card.appendChild(el('p', 'muted', (value || []).length + ' point(s) on the complex plane; unit circle dashed.'));
      } else if (type === 'points') {
        var pts = (value || []).filter(function (p) { return Array.isArray(p); });
        scene.points = scene.points.concat(pts);
        card.appendChild(el('p', 'muted', pts.length + ' points drawn in the 3D view above.'));
        card.appendChild(el('div', 'mono small', 'bounds: ' + boundsText(pts)));
        card.appendChild(tableNode({
          columns: ['x', 'y', 'z'],
          rows: pts.slice(0, 32)
        }, precision));
        if (pts.length > 32) { card.appendChild(el('p', 'muted', 'first 32 of ' + pts.length + ' listed.')); }
      } else {
        var pre = el('pre', 'json-out', JSON.stringify(value, null, 2));
        card.appendChild(pre);
      }
      host.appendChild(card);
    });

    if (readouts.childNodes.length) { host.insertBefore(readouts, host.firstChild); }
    deferred.forEach(function (fn) { fn(); });
    return scene;
  };

  Panels.el = el;
  global.Panels = Panels;
}(window));
