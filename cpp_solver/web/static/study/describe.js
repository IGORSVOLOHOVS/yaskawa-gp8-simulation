/* describe.js - everything that reads the catalogue.
 *
 * Nothing here knows the name of a single module. The catalogue is indexed,
 * searched, defaulted and formatted purely from the describe() payload, so a
 * module added in C++ appears in the console with no change to this file.
 */
(function (global) {
  'use strict';

  var Describe = {};

  /* ---------------------------------------------------------------- numbers */

  Describe.fmt = function (value, precision) {
    if (value === null || value === undefined) { return '-'; }
    if (typeof value === 'boolean') { return value ? 'true' : 'false'; }
    if (typeof value === 'string') { return value; }
    var n = Number(value);
    if (!isFinite(n)) { return String(value); }
    if (n === 0) { return (0).toFixed(precision); }
    var mag = Math.abs(n);
    if (mag < Math.pow(10, -precision) || mag >= 1e7) { return n.toExponential(Math.max(1, precision - 1)); }
    return n.toFixed(precision);
  };

  Describe.isMatrix = function (v) {
    return Array.isArray(v) && v.length > 0 && Array.isArray(v[0]);
  };

  Describe.isTable = function (v) {
    return v && typeof v === 'object' && !Array.isArray(v) && Array.isArray(v.columns) && Array.isArray(v.rows);
  };

  Describe.clone = function (v) {
    return (v === undefined) ? v : JSON.parse(JSON.stringify(v));
  };

  /* ------------------------------------------------------- param defaulting */

  var VEC_LEN = { vec3: 3, vec6: 6 };

  Describe.paramLength = function (param) {
    if (VEC_LEN[param.type]) { return VEC_LEN[param.type]; }
    if (Array.isArray(param['default'])) { return param['default'].length; }
    return 0;
  };

  Describe.paramRange = function (param) {
    var lo = (typeof param.min === 'number') ? param.min : null;
    var hi = (typeof param.max === 'number') ? param.max : null;
    if (lo === null || hi === null) {
      var span = 1;
      var d = param['default'];
      var flat = [];
      (function walk(x) {
        if (Array.isArray(x)) { x.forEach(walk); } else if (typeof x === 'number') { flat.push(Math.abs(x)); }
      })(d);
      flat.forEach(function (a) { span = Math.max(span, a * 2); });
      if (lo === null) { lo = -span; }
      if (hi === null) { hi = span; }
    }
    if (hi <= lo) { hi = lo + 1; }
    return { min: lo, max: hi };
  };

  Describe.defaultValue = function (param) {
    if (param['default'] !== undefined) { return Describe.clone(param['default']); }
    var r = Describe.paramRange(param);
    switch (param.type) {
      case 'bool': return false;
      case 'int': return Math.round((r.min + r.max) / 2);
      case 'enum': return (param.options && param.options[0]) || '';
      case 'text': return '';
      case 'vec3': return [0, 0, 0];
      case 'vec6': return [0, 0, 0, 0, 0, 0];
      case 'mat3': return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
      case 'mat4': return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
      case 'matrix': return [[0]];
      case 'table': return { columns: ['value'], rows: [[0]] };
      case 'points': return [[0, 0, 0]];
      case 'complex_set': return [[0, 0]];
      default: return 0;
    }
  };

  Describe.defaultArgs = function (op) {
    var args = {};
    (op.params || []).forEach(function (p) { args[p.name] = Describe.defaultValue(p); });
    return args;
  };

  /* ------------------------------------------------------------- indexing */

  Describe.index = function (catalogue) {
    var modules = (catalogue && catalogue.modules) || [];
    var flat = [];
    var byKey = {};
    modules.forEach(function (m) {
      (m.ops || []).forEach(function (op) {
        var entry = { module: m, op: op, key: m.name + '/' + op.name };
        flat.push(entry);
        byKey[entry.key] = entry;
      });
    });
    return { modules: modules, flat: flat, byKey: byKey };
  };

  Describe.groupByCourse = function (modules) {
    var groups = [];
    var seen = {};
    modules.forEach(function (m) {
      var course = m.course || {};
      var id = String(course.id === undefined ? 'other' : course.id);
      if (!seen[id]) {
        seen[id] = { course: course, modules: [] };
        groups.push(seen[id]);
      }
      seen[id].modules.push(m);
    });
    return groups;
  };

  Describe.courseUrl = function (course) {
    if (!course || course.id === undefined || course.id === null) { return null; }
    return 'https://e.tsi.lv/course/view.php?id=' + course.id;
  };

  /* --------------------------------------------------------------- search */

  function haystack(module, op) {
    var parts = [module.name, module.title, module.summary, (module.topics || []).join(' ')];
    if (module.course) { parts.push(module.course.code, module.course.name); }
    if (op) { parts.push(op.name, op.title, op.explain); }
    return parts.filter(Boolean).join(' ').toLowerCase();
  }

  Describe.matches = function (module, op, query) {
    if (!query) { return true; }
    var terms = query.toLowerCase().split(/\s+/).filter(Boolean);
    var hay = haystack(module, op);
    return terms.every(function (t) { return hay.indexOf(t) >= 0; });
  };

  /* ------------------------------------------------------- latex rendering */

  var SYMBOL = {
    alpha: 'α', beta: 'β', gamma: 'γ', delta: 'δ', epsilon: 'ε',
    varepsilon: 'ε', zeta: 'ζ', eta: 'η', theta: 'θ', vartheta: 'ϑ',
    iota: 'ι', kappa: 'κ', lambda: 'λ', mu: 'μ', nu: 'ν', xi: 'ξ',
    pi: 'π', rho: 'ρ', sigma: 'σ', tau: 'τ', upsilon: 'υ', phi: 'φ',
    varphi: 'ϕ', chi: 'χ', psi: 'ψ', omega: 'ω',
    Gamma: 'Γ', Delta: 'Δ', Theta: 'Θ', Lambda: 'Λ', Xi: 'Ξ', Pi: 'Π',
    Sigma: 'Σ', Phi: 'Φ', Psi: 'Ψ', Omega: 'Ω',
    cdot: '·', cdots: '⋯', ldots: '…', dots: '…', vdots: '⋮', ddots: '⋱',
    times: '×', pm: '±', mp: '∓', approx: '≈', sim: '∼', simeq: '≃',
    neq: '≠', ne: '≠', leq: '≤', le: '≤', geq: '≥', ge: '≥',
    ll: '≪', gg: '≫', equiv: '≡', propto: '∝',
    infty: '∞', partial: '∂', nabla: '∇', forall: '∀', exists: '∃',
    in: '∈', notin: '∉', subset: '⊂', cup: '∪', cap: '∩',
    sum: '∑', prod: '∏', int: '∫', oint: '∮', iint: '∬',
    to: '→', rightarrow: '→', Rightarrow: '⇒', leftarrow: '←',
    Leftarrow: '⇐', leftrightarrow: '↔', mapsto: '↦', langle: '⟨',
    rangle: '⟩', lVert: '‖', rVert: '‖', Vert: '‖', vert: '|',
    lfloor: '⌊', rfloor: '⌋', lceil: '⌈', rceil: '⌉',
    circ: '∘', star: '⋆', ast: '∗', oplus: '⊕', otimes: '⊗',
    perp: '⊥', parallel: '∥', angle: '∠', degree: '°', prime: '′',
    top: '⊤', bot: '⊥', ' ': ' '
  };

  var ACCENT = {
    hat: '̂', widehat: '̂', bar: '̄', overline: '̄', vec: '⃗',
    tilde: '̃', widetilde: '̃', dot: '̇', ddot: '̈', check: '̌'
  };

  var SPACING = { ',': 'thin', ';': 'med', ':': 'thin', '!': 'neg', quad: 'quad', qquad: 'qquad', ' ': 'med' };

  var TEXTY = {
    operatorname: 1, mathrm: 1, text: 1, textrm: 1, mathbf: 1, bm: 1, boldsymbol: 1,
    mathbb: 1, mathcal: 1, mathsf: 1, mathit: 1, textbf: 1
  };

  function esc(s) {
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  }

  function tokenize(src) {
    var out = [];
    var i = 0;
    while (i < src.length) {
      var c = src[i];
      if (c === '\\') {
        if (src[i + 1] === '\\') { out.push({ k: 'rowsep' }); i += 2; continue; }
        var j = i + 1;
        var name = '';
        while (j < src.length && /[a-zA-Z]/.test(src[j])) { name += src[j]; j += 1; }
        if (!name) { out.push({ k: 'cmd', v: src[i + 1] === undefined ? ' ' : src[i + 1] }); i += 2; continue; }
        out.push({ k: 'cmd', v: name });
        i = j;
        continue;
      }
      if (c === '{') { out.push({ k: 'open' }); i += 1; continue; }
      if (c === '}') { out.push({ k: 'close' }); i += 1; continue; }
      if (c === '&') { out.push({ k: 'cell' }); i += 1; continue; }
      if (c === '^') { out.push({ k: 'sup' }); i += 1; continue; }
      if (c === '_') { out.push({ k: 'sub' }); i += 1; continue; }
      if (/\s/.test(c)) { out.push({ k: 'space' }); i += 1; continue; }
      out.push({ k: 'ch', v: c });
      i += 1;
    }
    return out;
  }

  function groupText(tk, st) {
    var text = '';
    if (!tk[st.i] || tk[st.i].k !== 'open') {
      if (tk[st.i]) { text = tk[st.i].v || ''; st.i += 1; }
      return text;
    }
    st.i += 1;
    while (st.i < tk.length && tk[st.i].k !== 'close') {
      if (tk[st.i].v) { text += tk[st.i].v; }
      st.i += 1;
    }
    st.i += 1;
    return text;
  }

  function group(tk, st) {
    if (tk[st.i] && tk[st.i].k === 'open') { st.i += 1; return list(tk, st, true); }
    return step(tk, st);
  }

  function step(tk, st) {
    var t = tk[st.i];
    if (!t) { return ''; }
    if (t.k === 'sup' || t.k === 'sub') {
      st.i += 1;
      var tag = (t.k === 'sup') ? 'sup' : 'sub';
      return '<' + tag + '>' + group(tk, st) + '</' + tag + '>';
    }
    if (t.k === 'open') { st.i += 1; return '<span class="tex-grp">' + list(tk, st, true) + '</span>'; }
    st.i += 1;
    if (t.k === 'ch') { return esc(t.v); }
    if (t.k === 'space') { return ' '; }
    if (t.k === 'rowsep' || t.k === 'cell') { return ' '; }
    if (t.k === 'close') { return ''; }
    if (t.k === 'cmd') { return command(t.v, tk, st); }
    return '';
  }

  function list(tk, st, stopAtClose) {
    var html = '';
    while (st.i < tk.length) {
      var t = tk[st.i];
      if (t.k === 'close') { st.i += 1; if (stopAtClose) { break; } continue; }
      if (t.k === 'cmd' && t.v === 'end') { break; }
      html += step(tk, st);
    }
    return html;
  }

  function matrix(tk, st, kind) {
    var rows = [[]];
    var cur = '';
    while (st.i < tk.length) {
      var t = tk[st.i];
      if (t.k === 'cmd' && t.v === 'end') { st.i += 1; groupText(tk, st); break; }
      if (t.k === 'cell') { st.i += 1; rows[rows.length - 1].push(cur); cur = ''; continue; }
      if (t.k === 'rowsep') { st.i += 1; rows[rows.length - 1].push(cur); cur = ''; rows.push([]); continue; }
      cur += step(tk, st);
    }
    rows[rows.length - 1].push(cur);
    var body = rows.map(function (r) {
      return '<tr>' + r.map(function (c) { return '<td>' + c + '</td>'; }).join('') + '</tr>';
    }).join('');
    var cls = 'tex-matrix tex-' + kind;
    return '<span class="' + cls + '"><table><tbody>' + body + '</tbody></table></span>';
  }

  function command(name, tk, st) {
    if (name === 'frac' || name === 'dfrac' || name === 'tfrac') {
      var num = group(tk, st);
      var den = group(tk, st);
      return '<span class="tex-frac"><span class="tex-num">' + num + '</span><span class="tex-den">' + den + '</span></span>';
    }
    if (name === 'sqrt') {
      return '√<span class="tex-sqrt">' + group(tk, st) + '</span>';
    }
    if (name === 'begin') {
      var env = groupText(tk, st);
      if (/matrix/.test(env)) { return matrix(tk, st, env); }
      return list(tk, st, false);
    }
    if (name === 'end') { groupText(tk, st); return ''; }
    if (name === 'left' || name === 'right') {
      var d = tk[st.i];
      if (!d) { return ''; }
      st.i += 1;
      if (d.k === 'cmd') { return SYMBOL[d.v] || esc(d.v === '{' || d.v === '}' ? d.v : ''); }
      if (d.k === 'ch') { return d.v === '.' ? '' : esc(d.v); }
      if (d.k === 'open') { return '{'; }
      if (d.k === 'close') { return '}'; }
      return '';
    }
    if (TEXTY[name]) {
      var weight = (name === 'mathbf' || name === 'bm' || name === 'boldsymbol' || name === 'textbf') ? ' tex-bold' : '';
      return '<span class="tex-op' + weight + '">' + esc(groupText(tk, st)) + '</span>';
    }
    if (ACCENT[name]) { return '<span class="tex-acc">' + group(tk, st) + ACCENT[name] + '</span>'; }
    if (SPACING[name]) { return '<span class="tex-sp-' + SPACING[name] + '"></span>'; }
    if (name === 'limits' || name === 'displaystyle' || name === 'nolimits') { return ''; }
    if (SYMBOL[name]) { return SYMBOL[name]; }
    if (name === '{' || name === '}' || name === '%' || name === '#' || name === '_' || name === '&') { return esc(name); }
    return '<span class="tex-op">' + esc(name) + '</span>';
  }

  /** Render a LaTeX fragment (no surrounding $) to HTML. Subset renderer: no
   *  CDN is allowed here, so it covers what the module contract uses. */
  Describe.latex = function (src) {
    if (!src) { return ''; }
    try {
      return list(tokenize(String(src)), { i: 0 }, false);
    } catch (err) {
      return esc(String(src));
    }
  };

  /* ------------------------------------------------------------ permalink */

  Describe.readHash = function () {
    var raw = (global.location.hash || '').replace(/^#/, '');
    if (!raw) { return null; }
    var parts = raw.split('?');
    var path = parts[0].split('/');
    if (path.length < 2 || !path[0] || !path[1]) { return null; }
    var args = null;
    if (parts[1]) {
      var params = new global.URLSearchParams(parts[1]);
      var packed = params.get('args');
      if (packed) {
        try { args = JSON.parse(decodeURIComponent(packed)); } catch (err) { args = null; }
      }
    }
    return { module: decodeURIComponent(path[0]), op: decodeURIComponent(path[1]), args: args };
  };

  Describe.writeHash = function (moduleName, opName, args) {
    var hash = '#' + encodeURIComponent(moduleName) + '/' + encodeURIComponent(opName);
    try {
      hash += '?args=' + encodeURIComponent(JSON.stringify(args || {}));
    } catch (err) { /* an unserialisable arg simply drops out of the permalink */ }
    if (global.location.hash !== hash) {
      global.history.replaceState(null, '', global.location.pathname + hash);
    }
  };

  global.Describe = Describe;
}(window));
