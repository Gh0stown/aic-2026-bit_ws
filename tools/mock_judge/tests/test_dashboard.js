'use strict';

// Standalone regression tests: node tools/mock_judge/tests/test_dashboard.js
// This is a deliberately minimal simulated DOM/fetch, NOT a browser test.
// It verifies dashboard state and API behavior, not layout, native dialog focus,
// image decoding, accessibility, or the browser's actual download experience.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');

class Element {
  constructor(tagName = 'div') {
    this.tagName = tagName;
    this.children = [];
    this.dataset = {};
    this.value = '';
    this.hidden = false;
    this.disabled = false;
    this.listeners = new Map();
    this.attributes = new Map();
    this.classes = new Set();
    this.classList = {toggle: (name, enabled) => enabled ? this.classes.add(name) : this.classes.delete(name)};
  }
  append(...nodes) { for (const item of nodes) this.insertBefore(item, null); }
  insertBefore(item, before) {
    if (item.parentNode) item.remove();
    const index = before ? this.children.indexOf(before) : -1;
    this.children.splice(index < 0 ? this.children.length : index, 0, item);
    item.parentNode = this;
    return item;
  }
  remove() {
    if (!this.parentNode) return;
    this.parentNode.children.splice(this.parentNode.children.indexOf(this), 1);
    this.parentNode = null;
  }
  get options() { return this.children; }
  set textContent(value) {
    this._text = String(value);
    for (const child of this.children) child.parentNode = null;
    this.children = [];
  }
  get textContent() { return this._text || ''; }
  addEventListener(type, listener) {
    if (!this.listeners.has(type)) this.listeners.set(type, []);
    this.listeners.get(type).push(listener);
  }
  dispatch(type, details = {}) {
    for (const listener of this.listeners.get(type) || []) listener({target: this, preventDefault() {}, ...details});
  }
  setAttribute(key, value) { this.attributes.set(key, String(value)); }
  removeAttribute(key) { delete this[key]; this.attributes.delete(key); }
  showModal() { this.open = true; }
  close() { this.open = false; this.dispatch('close'); }
  focus() { this.focused = true; }
  click() { this.dispatch('click'); }
}

const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'templates/index.html'), 'utf8');
const elements = new Map();
for (const match of html.matchAll(/<([a-z][a-z0-9-]*)\b[^>]*\bid="([^"]+)"/gi)) elements.set(match[2], new Element(match[1]));
function element(id) {
  assert(elements.has(id), `JavaScript referenced a missing HTML id: ${id}`);
  return elements.get(id);
}
for (const id of ['session-filter', 'team-filter', 'task-filter', 'lock-team-select']) {
  const option = new Element('option');
  option.value = id === 'session-filter' ? 'current' : '';
  element(id).append(option);
  element(id).value = option.value;
}
const adminIds = ['unlock-team', 'start-session', 'finish-session', 'new-session', 'export-session'];
const documentListeners = new Map();
const document = {
  hidden: false,
  body: new Element('body'),
  getElementById: element,
  createElement: tag => new Element(tag),
  querySelectorAll: selector => selector === '[data-admin]' ? adminIds.map(element) : [],
  addEventListener: (type, listener) => documentListeners.set(type, listener)
};
const windowListeners = new Map();
const tasks = ['crowd', 'sign', 'trash', 'fire', 'plate', 'temperature', 'meter', 'ebike'];
const session = {
  id: 'current-session', created_at: '2026-11-10T00:00:00Z', status: 'ready',
  locked_team: null, started_at: null, finished_at: null, elapsed_seconds: 0
};
const oldSession = {...session, id: 'old-session', status: 'finished', elapsed_seconds: 99};
const maliciousTeam = '  <script>坏队</script>  ';
let reports = Array.from({length: 250}, (_, index) => ({
  id: `report-${250 - index}`, session_id: session.id,
  received_at: '2026-11-10T00:00:00Z', team: maliciousTeam, task: tasks[index % 8],
  result: '<img src=x onerror=alert(1)>', sub_id: `point-${index}`, extra: '原文\n附注', image_count: index === 0 ? 2 : 0,
  images: index === 0 ? [0, 1].map(position => ({
    id: `image-${position}`, url: `/api/images/image-${position}`, original_name: `图片${position}.png`,
    media_type: 'image/png', width: 100, height: 80, position
  })) : []
}));
const historicalReports = [{...reports[0], id: 'old-report', session_id: oldSession.id, team: '历史队伍'}];
const calls = [];
let active = 0;
let maxActive = 0;
let disconnected = false;
let adminFailure = false;
function stateFor(selected) {
  const historical = selected === oldSession.id;
  return {
    ok: true, schema_version: 1, server_time: '2026-11-10T00:00:00Z',
    session: {...historical ? oldSession : session}, sessions: [{...session}, {...oldSession}],
    teams: [{team: historical ? '历史队伍' : maliciousTeam, last_hello_at: null, last_report_at: null, report_count: historical ? 1 : reports.length}],
    tasks: tasks.map(id => ({id, label: id}))
  };
}
const context = {
  console, document, performance, URL, URLSearchParams, AbortController,
  window: {
    location: new URL('http://localhost/'), setTimeout: () => 1, clearTimeout() {}, setInterval: () => 1,
    addEventListener: (type, listener) => windowListeners.set(type, listener), confirm: () => true
  },
  fetch: async (url, options) => {
    active += 1;
    maxActive = Math.max(maxActive, active);
    calls.push({url, options});
    try {
      await new Promise(resolve => setImmediate(resolve));
      if (options.signal.aborted) throw new DOMException('Aborted', 'AbortError');
      if (disconnected) throw new Error('模拟断线');
      const parsed = new URL(url, 'http://localhost');
      let data;
      if (parsed.pathname === '/api/state') data = stateFor(parsed.searchParams.get('session_id'));
      else if (parsed.pathname === '/api/reports') {
        let selected = parsed.searchParams.get('session_id') === oldSession.id ? historicalReports : reports;
        for (const key of ['team', 'task', 'sub_id']) if (parsed.searchParams.has(key)) selected = selected.filter(report => report[key] === parsed.searchParams.get(key));
        const offset = Number(parsed.searchParams.get('offset'));
        data = {ok: true, reports: selected.slice(offset, offset + 100), total: selected.length, limit: 100, offset};
      } else if (parsed.pathname === '/api/admin/export') {
        assert.equal(options.headers.Authorization, 'Bearer test-secret');
        return {ok: true, status: 200, blob: async () => new Blob(['test zip']), headers: new Map([['Content-Disposition', "attachment; filename*=UTF-8''%E5%AF%BC%E5%87%BA.zip"]])};
      } else if (parsed.pathname.startsWith('/api/admin/')) {
        assert.equal(options.headers.Authorization, 'Bearer test-secret');
        if (adminFailure) return {ok: false, status: 403, json: async () => ({ok: false, error: '令牌不正确'})};
        const body = JSON.parse(options.body);
        if (parsed.pathname.endsWith('/team-lock')) session.locked_team = body.team;
        else if (parsed.pathname.endsWith('/start')) session.status = 'running';
        else if (parsed.pathname.endsWith('/finish')) { session.status = 'finished'; session.elapsed_seconds = 12; }
        data = {ok: true, session: {...session}};
      } else throw new Error(`Unexpected request: ${url}`);
      return {ok: true, status: 200, json: async () => data};
    } finally { active -= 1; }
  }
};
context.globalThis = context;
let source = fs.readFileSync(path.join(root, 'static/app.js'), 'utf8');
assert(!/innerHTML|outerHTML|insertAdjacentHTML|localStorage|sessionStorage/.test(source), 'Never render untrusted HTML or persist a token');
source = source.replace(/\}\)\(\);\s*$/, 'globalThis.test={model,cards,poll,setMode,filters,reportsURL,safeImageURL,exportFilename,renderState,updateClock,openGallery,moveGallery,authorization,manage,postAdmin};})();');
vm.createContext(context);
vm.runInContext(source, context, {filename: 'app.js'});
const test = context.test;
async function idle() {
  // Real microtask/event-loop flushing, not polling an external process.
  for (let turn = 0; turn < 3; turn += 1) {
    if (test.model.requestInFlight) await test.model.requestInFlight;
    await new Promise(resolve => setImmediate(resolve));
  }
}
async function run() {
  await idle();
  assert.equal(test.cards.size, 8);
  assert.equal(test.model.reports.length, 250, 'Initial latest scan must not truncate at 100');
  assert.equal(calls.filter(call => call.url.startsWith('/api/reports')).length, 3);
  assert.equal(maxActive, 1);
  assert.equal(test.model.reports[0].id, 'report-250');
  const card = test.cards.get('crowd');
  const stableEntry = card.entries.get('report-250');
  assert.equal(stableEntry.children[1].children[1].textContent, '<img src=x onerror=alert(1)>');
  await Promise.all([test.poll(), test.poll(), test.poll()]);
  assert.equal(maxActive, 1, 'Polls must never overlap');
  assert.equal(card.entries.get('report-250'), stableEntry, 'Unchanged reports retain their DOM');
  reports.unshift({...reports[0], id: 'same-time-new-report'});
  await test.poll();
  assert.equal(test.model.reports.length, 250);
  assert.equal(test.model.reports[0].id, 'same-time-new-report', 'Server append order wins for identical timestamps');
  assert(!test.reportsURL({session_id: session.id, team: '', task: '', sub_id: ''}, 0).includes('team='));
  assert.equal(test.safeImageURL('javascript:alert(1)'), '');
  assert.equal(test.safeImageURL('https://evil.test/api/images/a'), '');
  assert(test.safeImageURL('/api/images/image-0'));
  assert.equal(test.exportFilename("attachment; filename*=UTF-8''%E5%AF%BC%E5%87%BA.zip", 'fallback'), '导出.zip');
  assert.equal(test.exportFilename('attachment; filename="../../a.zip"', 'fallback'), '.._.._a.zip');
  test.openGallery(reports[0], 0);
  test.moveGallery(1);
  assert.equal(element('image-position').textContent, '2 / 2');
  await test.poll();
  assert(element('image-dialog').open, 'Poll must not close gallery');
  const plateReports = [1, 2, 3].map(number => ({...reports[0], id: `plate-${number}`, task: 'plate', sub_id: `车位 ${number}`}));
  reports.unshift(...plateReports);
  await test.poll();
  const plateCard = test.cards.get('plate');
  const otherTaskEntry = test.cards.get('crowd').entries.get('same-time-new-report');
  const callsBeforeTabs = calls.length;
  for (const number of [1, 2, 3]) {
    const point = `车位 ${number}`;
    plateCard.pointButtons.get(point).click();
    assert.equal(plateCard.selectedPoint, point);
    assert.equal(Array.from(plateCard.entries.values()).filter(entry => !entry.hidden).length, 1);
    assert.equal(plateCard.pointButtons.get(point).attributes.get('aria-pressed'), 'true');
  }
  assert.equal(calls.length, callsBeforeTabs, 'Card point tabs filter locally without fetching');
  assert.equal(test.cards.get('crowd').entries.get('same-time-new-report'), otherTaskEntry, 'Local plate filtering preserves other tasks');
  const selectedPointButton = plateCard.pointButtons.get('车位 3');
  await test.poll();
  assert.equal(plateCard.pointButtons.get('车位 3'), selectedPointButton, 'Polling preserves tab DOM and selection');
  assert.equal(plateCard.selectedPoint, '车位 3');
  plateCard.pointButtons.get(null).click();
  assert(Array.from(plateCard.entries.values()).filter(entry => !entry.hidden).length > 3);
  test.setMode('history');
  await idle();
  assert(plateCard.pointNav.hidden, 'Point tabs are hidden in history mode');
  assert.equal(test.model.reports.length, 100);
  assert.equal(element('next-page').disabled, false);
  test.model.offset = 200;
  await test.poll();
  assert.equal(test.model.reports.length, 54);
  element('point-filter').value = '  精确点位  ';
  element('point-form').dispatch('submit');
  await idle();
  assert.equal(test.filters().sub_id, '  精确点位  ', 'Exact point filtering preserves whitespace');
  element('point-filter').value = '';
  element('point-form').dispatch('submit');
  await idle();
  element('session-filter').value = oldSession.id;
  element('session-filter').dispatch('change');
  await idle();
  assert.equal(test.model.reports[0].team, '历史队伍');
  assert(element('team-filter').options.some(option => option.value === '历史队伍'), 'Historical team options come from selected state');
  assert(element('lock-team-select').options.some(option => option.value === maliciousTeam), 'Lock selector still uses CURRENT teams');
  assert.equal(element('current-session').textContent, session.id, 'Top status remains the current management session');
  element('team-filter').value = '历史队伍';
  element('team-filter').dispatch('change');
  await idle();
  assert(calls.some(call => call.url.startsWith('/api/reports') && new URL(call.url, 'http://localhost').searchParams.get('team') === '历史队伍'));
  element('admin-token').value = 'test-secret';
  element('admin-token').dispatch('input');
  assert.equal(test.authorization().Authorization, 'Bearer test-secret');
  assert(!calls.some(call => !call.url.startsWith('/api/admin/') && call.options.headers && call.options.headers.Authorization), 'Public reads never receive Bearer token');
  element('lock-team-select').value = maliciousTeam;
  element('lock-team-select').dispatch('change');
  assert.equal(element('lock-team-input').value, maliciousTeam, 'Selecting an existing team preserves its exact name');
  element('lock-form').dispatch('submit');
  await idle();
  const lockRequest = calls.find(call => call.url === '/api/admin/team-lock');
  assert.deepEqual(JSON.parse(lockRequest.options.body), {team: maliciousTeam});
  assert.equal(session.locked_team, maliciousTeam);
  assert.equal(element('session-filter').value, oldSession.id, 'Management does not silently change the historical view');
  await test.manage(() => test.postAdmin('session/start', {}, '已开始'));
  await idle();
  assert.equal(element('new-session').disabled, true, 'Running session requires finish before new');
  await test.manage(() => test.postAdmin('session/finish', {}, '已结束'));
  await idle();
  test.updateClock();
  assert.equal(element('elapsed').textContent, '00:00:12', 'Finished timer is frozen');
  element('export-session').click();
  await idle();
  const exportCall = calls.find(call => call.url.startsWith('/api/admin/export'));
  assert.equal(new URL(exportCall.url, 'http://localhost').searchParams.get('session_id'), oldSession.id, 'Export targets selected historical session');
  assert.equal(exportCall.options.headers.Authorization, 'Bearer test-secret');
  adminFailure = true;
  await test.manage(() => test.postAdmin('team-lock', {team: null}, '不应成功'));
  await idle();
  assert.equal(element('admin-message').textContent, '令牌不正确');
  disconnected = true;
  await test.poll();
  assert.equal(element('connection-label').textContent, '连接中断');
  assert(test.model.reports.length > 0, 'Disconnect retains last successful records');
  windowListeners.get('pagehide')();
  assert.equal(test.model.token, '');
  assert.equal(element('admin-token').value, '');
  assert.equal(maxActive, 1, 'Management, export and polling are serialized');
  console.log('PASS simulated DOM/fetch: 8 cards; 250-group complete latest scan; same-time append order; incremental cache; stable DOM/gallery; three-plate local point tabs; history pagination; historical team filtering; current management vs selected export; token isolation/clear; safe URLs/text; filenames; frozen timer; Chinese errors; disconnect retention; single in-flight.');
  console.log('NOT browser-verified: responsive layout, native dialog focus, actual image rendering/download, mobile interactions.');
}
run().catch(error => { console.error(error); process.exitCode = 1; });
