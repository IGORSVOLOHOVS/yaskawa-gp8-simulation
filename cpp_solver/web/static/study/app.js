/* app.js - the console itself: catalogue, selection, invocation, permalink.
 *
 * The only things this file knows about the study layer are the wire protocol
 * and the type vocabulary. Modules, ops, params and outputs are whatever the
 * binary reports, so nine new modules need no edit here.
 */
(function (global) {
  'use strict';

  var D = global.Describe;
  var P = global.Panels;

  var state = {
    index: { modules: [], flat: [], byKey: {} },
    entry: null,
    args: {},
    argsByKey: {},
    precision: 4,
    query: '',
    visible: [],
    controls: null,
    lastResult: null,
    lastUs: null,
    pending: null,
    reqId: 0,
    inFlight: 0
  };

  var dom = {};

  function $(id) { return document.getElementById(id); }

  function setStatus(text, kind) {
    dom.status.textContent = text;
    dom.status.className = 'status' + (kind ? ' ' + kind : '');
  }

  function banner(message, hint) {
    dom.banner.innerHTML = '';
    if (!message) { dom.banner.style.display = 'none'; return; }
    dom.banner.style.display = '';
    dom.banner.appendChild(P.el('strong', null, message));
    if (hint) { dom.banner.appendChild(P.el('div', 'banner-hint', hint)); }
    var retry = P.el('button', 'step-btn', 'retry');
    retry.addEventListener('click', boot);
    dom.banner.appendChild(retry);
  }

  /* ------------------------------------------------------------- catalogue */

  function catalogueFrom(reply) {
    if (!reply || typeof reply !== 'object') { return { error: 'empty reply from server' }; }
    if (reply.ok === false) { return { error: reply.error || 'describe failed' }; }
    if (reply.result && Array.isArray(reply.result.modules)) { return { catalogue: reply.result }; }
    if (Array.isArray(reply.modules)) { return { catalogue: reply }; }
    return { error: 'describe returned no modules' };
  }

  function boot() {
    setStatus('loading catalogue', 'busy');
    fetch('/api/describe', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (reply) {
        var parsed = catalogueFrom(reply);
        if (parsed.error) {
          banner(parsed.error, 'Build the C++ study binary with  ./robot build release  (or set STUDY_API to its path), then press retry. The console stays loaded meanwhile.');
          setStatus('no catalogue', 'bad');
          state.index = D.index({ modules: [] });
          renderSidebar();
          return;
        }
        banner(null);
        state.index = D.index(parsed.catalogue);
        renderSidebar();
        var hash = D.readHash();
        var key = hash ? (hash.module + '/' + hash.op) : null;
        if (key && state.index.byKey[key]) {
          select(key, hash.args);
        } else if (state.index.flat.length) {
          select(state.index.flat[0].key, null);
        } else {
          setStatus('catalogue is empty', 'bad');
        }
      })
      .catch(function (err) {
        banner('cannot reach study_server: ' + err, 'Start it with  python cpp_solver/web/study_server.py --port 8090');
        setStatus('offline', 'bad');
      });
  }

  /* --------------------------------------------------------------- sidebar */

  function renderSidebar() {
    var host = dom.tree;
    host.innerHTML = '';
    state.visible = [];
    var groups = D.groupByCourse(state.index.modules);
    var shown = 0;

    groups.forEach(function (group) {
      var modules = group.modules.filter(function (m) {
        return (m.ops || []).some(function (op) { return D.matches(m, op, state.query); })
          || D.matches(m, null, state.query);
      });
      if (!modules.length) { return; }

      var block = P.el('div', 'course-block');
      var head = P.el('div', 'course-head');
      var course = group.course || {};
      var url = D.courseUrl(course);
      var title;
      if (url) {
        title = P.el('a', 'course-name');
        title.href = url;
        title.target = '_blank';
        title.rel = 'noopener';
        title.title = 'Open Moodle course ' + course.id;
      } else {
        title = P.el('span', 'course-name');
      }
      title.textContent = course.name || 'Uncatalogued';
      head.appendChild(title);
      var code = P.el('span', 'course-code', [course.code, course.id !== undefined ? 'id ' + course.id : null]
        .filter(Boolean).join(' · '));
      head.appendChild(code);
      block.appendChild(head);

      modules.forEach(function (m) {
        var modBlock = P.el('div', 'module-block');
        var modHead = P.el('div', 'module-head');
        modHead.appendChild(P.el('span', 'module-title', m.title || m.name));
        modHead.appendChild(P.el('span', 'module-name', m.name));
        modBlock.appendChild(modHead);

        var ops = (m.ops || []).filter(function (op) { return D.matches(m, op, state.query); });
        if (!ops.length) { ops = m.ops || []; }
        ops.forEach(function (op) {
          var key = m.name + '/' + op.name;
          state.visible.push(key);
          var item = P.el('button', 'op-item');
          item.dataset.key = key;
          item.appendChild(P.el('span', 'op-title', op.title || op.name));
          item.appendChild(P.el('span', 'op-name', op.name));
          item.addEventListener('click', function () { select(key, null); });
          modBlock.appendChild(item);
          shown += 1;
        });
        block.appendChild(modBlock);
      });
      host.appendChild(block);
    });

    dom.count.textContent = shown + ' ops · ' + state.index.modules.length + ' modules';
    highlight();
  }

  function highlight() {
    var items = dom.tree.querySelectorAll('.op-item');
    for (var i = 0; i < items.length; i += 1) {
      var on = state.entry && items[i].dataset.key === state.entry.key;
      items[i].classList.toggle('active', !!on);
      if (on && items[i].scrollIntoView) { items[i].scrollIntoView({ block: 'nearest' }); }
    }
  }

  /* ------------------------------------------------------------- selection */

  function select(key, restoredArgs) {
    var entry = state.index.byKey[key];
    if (!entry) { return; }
    state.entry = entry;
    state.lastResult = null;
    state.lastUs = null;

    var args = D.defaultArgs(entry.op);
    var stored = restoredArgs || state.argsByKey[key];
    if (stored && typeof stored === 'object') {
      Object.keys(args).forEach(function (name) {
        if (stored[name] !== undefined) { args[name] = D.clone(stored[name]); }
      });
    }
    state.args = args;
    state.argsByKey[key] = args;

    renderHeader();
    state.controls = P.renderControls(dom.controls, entry.op, state.args, function () {
      D.writeHash(entry.module.name, entry.op.name, state.args);
      scheduleInvoke();
    });
    dom.outputs.innerHTML = '';
    dom.error.style.display = 'none';
    highlight();
    D.writeHash(entry.module.name, entry.op.name, state.args);
    invoke();
  }

  function renderHeader() {
    if (!state.entry) { return; }
    P.renderHeader(dom.header, state.entry, {
      us: state.lastUs,
      precision: state.precision,
      onPrecision: function (p) {
        state.precision = p;
        renderHeader();
        if (state.lastResult) { renderOutputs(state.lastResult); }
      }
    });
  }

  /* -------------------------------------------------------------- invoking */

  function scheduleInvoke() {
    if (state.pending) { global.clearTimeout(state.pending); }
    state.pending = global.setTimeout(function () {
      state.pending = null;
      invoke();
    }, 70);
  }

  function invoke() {
    if (!state.entry) { return; }
    var entry = state.entry;
    state.reqId += 1;
    var id = state.reqId;
    var payload = {
      id: id,
      module: entry.module.name,
      op: entry.op.name,
      args: D.clone(state.args)
    };
    state.inFlight += 1;
    setStatus('running ' + entry.key, 'busy');
    fetch('/api/invoke', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    })
      .then(function (r) { return r.json(); })
      .then(function (reply) {
        state.inFlight -= 1;
        if (id !== state.reqId || state.entry !== entry) { return; }   // a newer drag won
        if (reply && reply.ok) {
          state.lastResult = reply.result || {};
          state.lastUs = (typeof reply.us === 'number') ? reply.us : null;
          if (state.controls) { state.controls.clear(); }
          dom.error.style.display = 'none';
          renderHeader();
          renderOutputs(state.lastResult);
          setStatus('ok · ' + (state.lastUs === null ? '?' : state.lastUs) + ' us', 'good');
        } else {
          var message = (reply && reply.error) ? reply.error : 'the op failed without a message';
          var nearControl = state.controls ? state.controls.show(message) : false;
          dom.error.textContent = message + (nearControl ? '' : '  (last good result kept below)');
          dom.error.style.display = '';
          setStatus('error', 'bad');
          // the last good result deliberately stays on screen
        }
      })
      .catch(function (err) {
        state.inFlight -= 1;
        dom.error.textContent = 'transport error: ' + err;
        dom.error.style.display = '';
        setStatus('transport error', 'bad');
      });
  }

  function jointsFor(op, args, result) {
    function sixRad(spec, source) {
      for (var i = 0; i < spec.length; i += 1) {
        var s = spec[i];
        if (s.type !== 'vec6') { continue; }
        if (s.unit && s.unit !== 'rad') { continue; }
        var v = source[s.name];
        if (Array.isArray(v) && v.length === 6) { return v.map(Number); }
      }
      return null;
    }
    return sixRad(op.params || [], args || {}) || sixRad(op.outputs || [], result || {}) || null;
  }

  function renderOutputs(result) {
    var scene = P.renderOutputs(dom.outputs, state.entry.op, result, { precision: state.precision });
    if (global.Robot3D && global.Robot3D.available()) {
      global.Robot3D.setJoints(jointsFor(state.entry.op, state.args, result));
      global.Robot3D.showScene(scene);
      dom.sceneNote.textContent = scene.frames.length + ' frame(s), ' + scene.points.length + ' point(s) from this op';
    }
  }

  /* -------------------------------------------------------------- keyboard */

  function step(delta) {
    if (!state.visible.length) { return; }
    var at = state.entry ? state.visible.indexOf(state.entry.key) : -1;
    var next = (at < 0) ? 0 : (at + delta + state.visible.length) % state.visible.length;
    select(state.visible[next], null);
  }

  function typing(target) {
    if (!target) { return false; }
    var tag = (target.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'select' || tag === 'textarea';
  }

  /* ------------------------------------------------------------------ boot */

  function ready() {
    dom.tree = $('tree');
    dom.search = $('search');
    dom.count = $('count');
    dom.header = $('panel-header');
    dom.controls = $('controls');
    dom.outputs = $('outputs');
    dom.error = $('panel-error');
    dom.status = $('status');
    dom.banner = $('banner');
    dom.sceneNote = $('scene-note');

    dom.search.addEventListener('input', function () {
      state.query = dom.search.value.trim();
      renderSidebar();
    });
    dom.search.addEventListener('keydown', function (ev) {
      if (ev.key === 'Escape') { dom.search.value = ''; state.query = ''; renderSidebar(); }
    });

    global.addEventListener('keydown', function (ev) {
      if (ev.key === '/' && !typing(ev.target)) { ev.preventDefault(); dom.search.focus(); return; }
      if (typing(ev.target)) { return; }
      if (ev.key === 'ArrowRight') { ev.preventDefault(); step(1); }
      if (ev.key === 'ArrowLeft') { ev.preventDefault(); step(-1); }
    });

    global.addEventListener('hashchange', function () {
      var hash = D.readHash();
      if (!hash) { return; }
      var key = hash.module + '/' + hash.op;
      if (state.entry && state.entry.key === key) { return; }
      if (state.index.byKey[key]) { select(key, hash.args); }
    });

    var viewport = $('viewport');
    if (global.Robot3D && global.Robot3D.available()) {
      global.Robot3D.init(viewport);
      global.Robot3D.setJoints(null);
    } else {
      dom.sceneNote.textContent = 'three.js did not load (offline?) - panels and plots still work';
    }
    boot();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', ready);
  } else {
    ready();
  }
}(window));
