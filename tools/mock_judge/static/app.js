(() => {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const TASKS = [
    ['crowd', '人群识别'], ['sign', '标志识别'], ['trash', '垃圾识别'],
    ['fire', '火情识别'], ['plate', '车牌识别'], ['temperature', '温度识别'],
    ['meter', '仪表识别'], ['ebike', '电动车识别']
  ];
  const PAGE_SIZE = 100;
  const model = {
    state: null, mode: 'latest', offset: 0, total: 0, reports: [],
    filterVersion: 0, cacheKey: '', latestCache: new Map(), cacheComplete: false,
    requestInFlight: null, pollController: null, adminBusy: false,
    timerBase: 0, timerSyncedAt: 0, timerStatus: 'ready',
    token: '', gallery: null, galleryIndex: 0, renderedKey: ''
  };
  const cards = new Map();
  const statusLabels = {ready: '待开始', running: '计时中', finished: '已结束'};
  const text = (value) => value == null ? '' : String(value);
  const localTime = (value) => {
    if (!value) return '—';
    const date = new Date(value);
    return Number.isNaN(date.getTime()) ? text(value) : date.toLocaleString('zh-CN', {hour12: false});
  };
  const node = (tag, className, value) => {
    const result = document.createElement(tag);
    if (className) result.className = className;
    if (value !== undefined) result.textContent = text(value);
    return result;
  };
  function notice(id, message, error = false) {
    const element = $(id);
    element.textContent = message;
    element.hidden = !message;
    element.classList.toggle('error', error);
  }
  function setConnection(online, message) {
    $('connection-dot').className = `dot ${online ? 'online' : 'offline'}`;
    $('connection-label').textContent = online ? '实时连接' : '连接中断';
    notice('network-error', online ? '' : `服务连接失败：${message}。保留最后一次数据，每秒自动重试。`, !online);
    if (online) $('last-sync').textContent = `同步于 ${new Date().toLocaleTimeString('zh-CN', {hour12: false})}`;
  }
  async function request(url, options = {}, controller = null, expectBlob = false) {
    const ownController = controller || new AbortController();
    const timeout = window.setTimeout(() => {
      ownController.timedOut = true;
      ownController.abort();
    }, expectBlob ? 120000 : 15000);
    try {
      const response = await fetch(url, {...options, signal: ownController.signal, cache: 'no-store', credentials: 'same-origin'});
      if (expectBlob && response.ok) return {response, blob: await response.blob()};
      let data;
      try { data = await response.json(); }
      catch (_) { throw new Error(`服务返回非 JSON 数据（HTTP ${response.status}）`); }
      if (!response.ok || !data || data.ok !== true) {
        throw new Error(data && data.error ? text(data.error) : `请求失败（HTTP ${response.status}）`);
      }
      return data;
    } finally { window.clearTimeout(timeout); }
  }
  function reconcileOptions(select, entries, firstLabel, firstValue = '') {
    const value = select.value;
    const desired = [[firstValue, firstLabel], ...entries.filter(([id]) => id !== firstValue)];
    // Keep a selected historical team even if it is absent from the current state.
    if (value && value !== firstValue && !desired.some(([id]) => id === value)) desired.push([value, value]);
    const signature = JSON.stringify(desired);
    if (select.dataset.signature === signature) return;
    const existing = new Map(Array.from(select.options, option => [option.value, option]));
    const keep = new Set(desired.map(([id]) => id));
    for (const option of Array.from(select.options)) if (!keep.has(option.value)) option.remove();
    desired.forEach(([id, label], index) => {
      const option = existing.get(id) || document.createElement('option');
      option.value = id;
      option.textContent = label;
      if (select.options[index] !== option) select.insertBefore(option, select.options[index] || null);
    });
    select.value = value || firstValue;
    select.dataset.signature = signature;
  }
  function buildCards() {
    for (const [id, label] of TASKS) {
      const card = node('section', 'task-card');
      const header = node('div', 'task-header');
      const title = node('div', 'task-title');
      const heading = node('h3', '', label);
      title.append(heading, node('span', 'task-code', id));
      const count = node('span', 'task-count', '0 条');
      header.append(title, count);
      const pointNav = node('div', 'point-tabs');
      pointNav.setAttribute('role', 'group');
      pointNav.setAttribute('aria-label', `${label}点位切换`);
      const body = node('div', 'task-reports');
      const empty = node('p', 'task-empty', '暂无上报');
      body.append(empty);
      card.append(header, pointNav, body);
      $('task-grid').append(card);
      cards.set(id, {card, heading, count, pointNav, selectedPoint: null, pointButtons: new Map(), body, empty, entries: new Map()});
    }
  }
  function updateClock() {
    let seconds = model.timerStatus === 'ready' ? 0 : model.timerBase;
    if (model.timerStatus === 'running') seconds += Math.max(0, (performance.now() - model.timerSyncedAt) / 1000);
    seconds = Math.max(0, Math.floor(seconds));
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor(seconds / 60) % 60;
    $('elapsed').textContent = [hours, minutes, seconds % 60].map(n => String(n).padStart(2, '0')).join(':');
  }
  function renderState(data) {
    if (data.schema_version !== 1 || !data.session || !Array.isArray(data.tasks) || !Array.isArray(data.teams) || !Array.isArray(data.sessions)) {
      throw new Error('状态接口结构或 schema_version 不受支持');
    }
    const previousSessionId = model.state && model.state.session.id;
    model.state = data;
    const session = data.session;
    model.timerBase = Number.isFinite(session.elapsed_seconds) ? session.elapsed_seconds : 0;
    model.timerSyncedAt = performance.now();
    model.timerStatus = session.status;
    updateClock();
    $('current-session').textContent = session.id;
    $('session-status').textContent = statusLabels[session.status] || session.status;
    $('session-created').textContent = `创建：${localTime(session.created_at)}`;
    $('locked-team').textContent = session.locked_team == null ? '未锁队 · 允许多队上报' : session.locked_team;
    $('session-times').textContent = `开始：${localTime(session.started_at)} / 结束：${localTime(session.finished_at)}`;
    const sessions = data.sessions.map(item => [item.id, `${localTime(item.created_at)} · ${statusLabels[item.status] || item.status} · ${item.id}`]);
    reconcileOptions($('session-filter'), sessions, '当前场次', 'current');
    const teams = data.teams.map(item => [item.team, item.team]);
    if ($('session-filter').value === 'current' || $('session-filter').value === session.id) renderSelectedTeams(data.teams);
    reconcileOptions($('lock-team-select'), teams, '选择队伍或在右侧输入');
    reconcileOptions($('task-filter'), TASKS.map(([id, label]) => {
      const remote = data.tasks.find(task => task.id === id);
      return [id, remote ? remote.label : label];
    }), '全部任务');
    for (const item of data.tasks) if (cards.has(item.id)) cards.get(item.id).heading.textContent = item.label;
    updateAdminButtons();
    if (previousSessionId && previousSessionId !== session.id && $('session-filter').value === 'current') resetReports(false);
  }
  function renderSelectedTeams(teams) {
    reconcileOptions($('team-filter'), teams.map(item => [item.team, item.team]), '全部队伍');
    renderTeams(teams);
  }
  function renderTeams(teams) {
    $('team-count').textContent = `（${teams.length}）`;
    const tbody = $('team-table');
    const existing = new Map(Array.from(tbody.children, row => [row.dataset.team, row]));
    const keep = new Set(teams.map(team => team.team));
    for (const row of Array.from(tbody.children)) if (!keep.has(row.dataset.team)) row.remove();
    teams.forEach((team, index) => {
      const row = existing.get(team.team) || node('tr');
      row.dataset.team = team.team;
      const values = [team.team, localTime(team.last_hello_at), localTime(team.last_report_at), team.report_count];
      values.forEach((value, cellIndex) => {
        const cell = row.children[cellIndex] || node('td');
        cell.textContent = text(value);
        if (!cell.parentNode) row.append(cell);
      });
      if (tbody.children[index] !== row) tbody.insertBefore(row, tbody.children[index] || null);
    });
  }
  function selectedSessionId() {
    return $('session-filter').value === 'current' ? model.state && model.state.session.id : $('session-filter').value;
  }
  function filters() {
    return {session_id: selectedSessionId(), team: $('team-filter').value, task: $('task-filter').value, sub_id: $('point-filter').dataset.applied || ''};
  }
  function reportsURL(filter, offset) {
    const params = new URLSearchParams({limit: String(PAGE_SIZE), offset: String(offset)});
    for (const [key, value] of Object.entries(filter)) if (value) params.set(key, value);
    return `/api/reports?${params}`;
  }
  function validateReports(data) {
    if (!Array.isArray(data.reports) || !Number.isInteger(data.total) || data.total < 0) throw new Error('上报列表接口结构不正确');
    return data;
  }
  async function fetchReports(version, controller) {
    const filter = filters();
    if (!filter.session_id) return;
    const key = JSON.stringify(filter);
    if (key !== model.cacheKey) {
      model.cacheKey = key;
      model.latestCache = new Map();
      model.cacheComplete = false;
    }
    const mode = model.mode;
    const offset = mode === 'history' ? model.offset : 0;
    const fresh = () => version === model.filterVersion && mode === model.mode && key === JSON.stringify(filters());
    const first = validateReports(await request(reportsURL(filter, offset), {}, controller));
    if (!fresh()) return;
    model.total = first.total;
    if (mode === 'history') {
      if (offset >= first.total && offset > 0) {
        model.offset = Math.max(0, Math.floor((first.total - 1) / PAGE_SIZE) * PAGE_SIZE);
        resetReports(false);
        return;
      }
      model.reports = first.reports;
    } else {
      // Reports are append-only. Initially scan every page; later fetch only the new
      // prefix until a known report is reached. This preserves latest-per-point
      // correctness even when the API has more than 100 reports.
      const oldIds = new Set(model.latestCache.keys());
      let page = first;
      let batch = page.reports;
      let scanned = 0;
      let complete = false;
      const prefix = new Map();
      while (true) {
        if (!fresh()) return;
        const hitKnown = batch.some(report => oldIds.has(report.id));
        for (const report of batch) prefix.set(report.id, report);
        scanned += batch.length;
        complete = scanned >= page.total || (model.cacheComplete && hitKnown);
        if (complete) break;
        if (!batch.length) throw new Error('历史分页提前返回空列表，请稍后重试');
        $('report-summary').textContent = `正在整理各点位最新记录… ${Math.min(scanned, page.total)} / ${page.total}`;
        page = validateReports(await request(reportsURL(filter, scanned), {}, controller));
        batch = page.reports;
        model.total = page.total;
      }
      model.cacheComplete = complete;
      // Keep server ordering, including reports with identical timestamps.
      model.latestCache = new Map([...prefix, ...Array.from(model.latestCache).filter(([id]) => !prefix.has(id))]);
      const ordered = Array.from(model.latestCache.values());
      const latest = new Map();
      for (const report of ordered) {
        const group = JSON.stringify([report.team, report.task, report.sub_id || '']);
        if (!latest.has(group)) latest.set(group, report);
      }
      model.reports = Array.from(latest.values());
    }
    renderReports(key);
    notice('report-error', '');
  }
  function safeImageURL(value) {
    try {
      const url = new URL(value, window.location.href);
      return url.origin === window.location.origin && url.pathname.startsWith('/api/images/') ? url.href : '';
    } catch (_) { return ''; }
  }
  function reportEntry(report) {
    const article = node('article', 'report');
    const meta = node('div', 'report-meta');
    meta.append(node('span', 'team-name', report.team), node('span', 'point-tag', report.sub_id ? `点位 ${report.sub_id}` : '未指定点位'));
    const time = node('time', 'report-time', localTime(report.received_at));
    time.dateTime = report.received_at;
    meta.append(time);
    const dl = node('dl');
    dl.append(node('dt', '', '上报原文'), node('dd', 'original', report.result), node('dt', '', '附注'), node('dd', 'extra', report.extra || '—'));
    article.append(meta, dl);
    const images = Array.isArray(report.images) ? report.images : [];
    if (images.length) {
      const thumbs = node('div', 'thumbnails');
      images.forEach((image, index) => {
        const url = safeImageURL(image.url);
        if (!url) { thumbs.append(node('span', 'image-unavailable', '图片地址不可用')); return; }
        const button = node('button', 'thumbnail');
        button.type = 'button';
        button.setAttribute('aria-label', `查看第 ${index + 1} 张图片：${text(image.original_name)}`);
        const img = node('img');
        img.src = url;
        img.alt = text(image.original_name) || `上报图片 ${index + 1}`;
        img.loading = 'lazy';
        img.addEventListener('error', () => { img.alt = '加载失败，点击重试'; });
        button.append(img, node('span', '', `${index + 1} / ${images.length}`));
        button.addEventListener('click', () => openGallery(report, index));
        thumbs.append(button);
      });
      article.append(thumbs);
    }
    article.append(node('small', '', `记录 ${report.id} · ${images.length} 张图片`));
    return article;
  }
  function renderPointTabs(card, reports) {
    card.pointNav.hidden = model.mode !== 'latest' || reports.length === 0;
    const points = [null, ...new Set(reports.map(report => report.sub_id || ''))];
    if (!points.includes(card.selectedPoint)) card.selectedPoint = null;
    for (const [point, button] of card.pointButtons) {
      if (!points.includes(point)) { button.remove(); card.pointButtons.delete(point); }
    }
    points.forEach((point, index) => {
      let button = card.pointButtons.get(point);
      if (!button) {
        button = node('button', 'secondary', point === null ? '全部点位' : point === '' ? '未指定' : point);
        button.type = 'button';
        button.addEventListener('click', () => {
          card.selectedPoint = point;
          renderReports(model.renderedKey);
        });
        card.pointButtons.set(point, button);
      }
      button.setAttribute('aria-pressed', String(card.selectedPoint === point));
      if (card.pointNav.children[index] !== button) card.pointNav.insertBefore(button, card.pointNav.children[index] || null);
    });
  }
  function renderReports(key) {
    const changedSelection = model.renderedKey !== key;
    model.renderedKey = key;
    const taskFilter = $('task-filter').value;
    for (const [id, card] of cards) {
      card.card.hidden = Boolean(taskFilter && taskFilter !== id);
      const reports = model.reports.filter(report => report.task === id);
      const ids = new Set(reports.map(report => text(report.id)));
      if (changedSelection) {
        for (const entry of card.entries.values()) entry.remove();
        card.entries.clear();
      }
      for (const [reportId, entry] of card.entries) {
        if (!ids.has(reportId)) { entry.remove(); card.entries.delete(reportId); }
      }
      renderPointTabs(card, reports);
      const visibleReports = reports.filter(report => model.mode !== 'latest' || card.selectedPoint === null || (report.sub_id || '') === card.selectedPoint);
      card.empty.hidden = visibleReports.length > 0;
      reports.forEach((report, index) => {
        const reportId = text(report.id);
        let entry = card.entries.get(reportId);
        if (!entry) { entry = reportEntry(report); card.entries.set(reportId, entry); }
        entry.hidden = model.mode === 'latest' && card.selectedPoint !== null && (report.sub_id || '') !== card.selectedPoint;
        // The empty placeholder is always first. Only move genuinely changed
        // positions; stable polls preserve focus, selection and thumbnail nodes.
        const target = card.body.children[index + 1] || null;
        if (entry !== target) card.body.insertBefore(entry, target);
      });
      card.count.textContent = `${visibleReports.length}${visibleReports.length !== reports.length ? ` / ${reports.length}` : ''} ${model.mode === 'latest' ? '组' : '条'}`;
      card.empty.textContent = model.total === 0 ? '此筛选下暂无上报' : (model.mode === 'history' ? '本页暂无此任务记录' : '此任务暂无上报');
    }
    $('report-summary').textContent = model.mode === 'latest' ? `共 ${model.total} 条上报 · ${model.reports.length} 组最新记录` : `共 ${model.total} 条上报`;
    $('pagination').hidden = model.mode !== 'history';
    $('page-summary').textContent = model.total ? `${model.offset + 1}–${Math.min(model.offset + PAGE_SIZE, model.total)} / ${model.total}` : '0 条记录';
    $('previous-page').disabled = model.offset === 0;
    $('next-page').disabled = model.offset + PAGE_SIZE >= model.total;
  }
  function resetReports(cancel = true) {
    model.filterVersion += 1;
    if (cancel && model.pollController) model.pollController.abort();
    $('report-summary').textContent = '筛选已更新，正在加载…';
    poll();
  }
  function selectionChanged() {
    model.offset = 0;
    resetReports();
  }
  async function poll() {
    if (model.requestInFlight || model.adminBusy || document.hidden) return;
    const controller = new AbortController();
    model.pollController = controller;
    const versionAtStart = model.filterVersion;
    const work = (async () => {
      try {
        const data = await request('/api/state', {}, controller);
        if (controller.signal.aborted) return;
        renderState(data);
        setConnection(true);
        const version = model.filterVersion;
        try {
          const selected = selectedSessionId();
          if (selected && selected !== data.session.id) {
            const historical = await request(`/api/state?${new URLSearchParams({session_id: selected})}`, {}, controller);
            if (version !== model.filterVersion) return;
            renderSelectedTeams(historical.teams);
          }
          await fetchReports(version, controller);
        }
        catch (error) {
          if (!controller.signal.aborted || controller.timedOut) {
            const message = controller.timedOut ? '请求超时' : error.message;
            notice('report-error', `记录同步失败：${message}。保留旧记录并自动重试。`, true);
            setConnection(false, message);
          }
        }
      } catch (error) {
        if (!controller.signal.aborted) setConnection(false, error.message);
        else if (controller.timedOut) setConnection(false, '请求超时');
      }
    })();
    model.requestInFlight = work;
    try { await work; }
    finally {
      model.requestInFlight = null;
      model.pollController = null;
      if (model.filterVersion !== versionAtStart && !model.adminBusy) window.setTimeout(poll, 0);
    }
  }
  function openGallery(report, index) {
    model.gallery = report;
    model.galleryIndex = index;
    showGalleryImage();
    if (!$('image-dialog').open) $('image-dialog').showModal();
  }
  function showGalleryImage() {
    const report = model.gallery;
    if (!report) return;
    const image = report.images[model.galleryIndex];
    const url = safeImageURL(image.url);
    $('image-title').textContent = `${report.team} · ${report.task} · ${report.sub_id || '未指定点位'}`;
    $('image-error').hidden = Boolean(url);
    $('large-image').hidden = !url;
    if (url) $('large-image').src = url;
    else $('large-image').removeAttribute('src');
    $('large-image').alt = text(image.original_name) || '上报图片';
    $('image-position').textContent = `${model.galleryIndex + 1} / ${report.images.length}`;
    $('image-caption').textContent = `${text(image.original_name)} · ${text(image.media_type)} · ${image.width || '?'} × ${image.height || '?'} · ${localTime(report.received_at)}`;
    $('previous-image').disabled = model.galleryIndex === 0;
    $('next-image').disabled = model.galleryIndex + 1 >= report.images.length;
  }
  function moveGallery(delta) {
    if (!model.gallery) return;
    const next = model.galleryIndex + delta;
    if (next < 0 || next >= model.gallery.images.length) return;
    model.galleryIndex = next;
    showGalleryImage();
  }
  function updateAdminButtons() {
    for (const button of document.querySelectorAll('[data-admin]')) button.disabled = model.adminBusy || !model.state;
    if (!model.state || model.adminBusy) return;
    $('start-session').disabled = model.state.session.status !== 'ready';
    $('finish-session').disabled = model.state.session.status !== 'running';
    $('new-session').disabled = model.state.session.status === 'running';
    $('new-session').title = model.state.session.status === 'running' ? '请先结束当前场次，再新建场次' : '保留历史记录并切换新场次';
  }
  function authorization() {
    const token = model.token.trim();
    if (!token) throw new Error('请先输入管理令牌');
    return {Authorization: `Bearer ${token}`};
  }
  async function manage(action) {
    if (model.adminBusy) return;
    try { authorization(); }
    catch (error) { notice('admin-message', error.message, true); $('admin-token').focus(); return; }
    model.adminBusy = true;
    updateAdminButtons();
    notice('admin-message', '管理请求处理中…');
    try {
      // Serialize management with polling as well, never run two API calls in
      // parallel. Do not cancel a POST once it has reached the server.
      if (model.pollController) model.pollController.abort();
      if (model.requestInFlight) await model.requestInFlight;
      await action();
    } catch (error) { notice('admin-message', error.name === 'AbortError' ? '管理请求超时；结果可能已生效，请等待状态同步后再确认，不要重复提交。' : error.message, true); }
    finally { model.adminBusy = false; updateAdminButtons(); poll(); }
  }
  async function postAdmin(path, body, success) {
    const data = await request(`/api/admin/${path}`, {
      method: 'POST', headers: {...authorization(), 'Content-Type': 'application/json'}, body: JSON.stringify(body)
    });
    // The next state poll refreshes team and session lists. Immediately update
    // the session clock from the POST response so a finished clock freezes now.
    if (data.session && model.state) renderState({...model.state, session: data.session});
    notice('admin-message', success);
  }
  function confirmation(message) {
    if (!model.state) return false;
    return window.confirm(`${message}\n当前场次：${model.state.session.id}\n这是模拟管理操作，不涉及官方评分。`);
  }
  function exportFilename(disposition, fallback) {
    if (!disposition) return fallback;
    let name = '';
    const extended = disposition.match(/filename\*\s*=\s*UTF-8''([^;]+)/i);
    if (extended) {
      try { name = decodeURIComponent(extended[1].trim().replace(/^"|"$/g, '')); } catch (_) { /* fallback below */ }
    }
    if (!name) {
      const plain = disposition.match(/filename\s*=\s*(?:"([^"]+)"|([^;]+))/i);
      if (plain) name = plain[1] || plain[2].trim();
    }
    name = name.replace(/[\\/\x00-\x1f\x7f]/g, '_').trim();
    return name && name !== '.' && name !== '..' ? name : fallback;
  }
  function setMode(mode) {
    if (model.mode === mode) return;
    model.mode = mode;
    model.offset = 0;
    $('latest-mode').setAttribute('aria-pressed', String(mode === 'latest'));
    $('history-mode').setAttribute('aria-pressed', String(mode === 'history'));
    $('latest-mode').classList.toggle('secondary', mode !== 'latest');
    $('history-mode').classList.toggle('secondary', mode !== 'history');
    $('view-description').textContent = mode === 'latest' ? '最新：每个队伍 / 任务 / 点位的最后一次上报。' : '历史：按接收时间倒序，每页 100 条；新上报可能使分页位置变化。';
    $('pagination').hidden = mode !== 'history';
    resetReports();
  }

  buildCards();
  for (const id of ['session-filter', 'team-filter', 'task-filter']) $(id).addEventListener('change', selectionChanged);
  $('point-form').addEventListener('submit', event => {
    event.preventDefault();
    $('point-filter').dataset.applied = $('point-filter').value;
    selectionChanged();
  });
  $('latest-mode').addEventListener('click', () => setMode('latest'));
  $('history-mode').addEventListener('click', () => setMode('history'));
  $('previous-page').addEventListener('click', () => { model.offset = Math.max(0, model.offset - PAGE_SIZE); resetReports(); });
  $('next-page').addEventListener('click', () => { if (model.offset + PAGE_SIZE < model.total) { model.offset += PAGE_SIZE; resetReports(); } });
  $('token-form').addEventListener('submit', event => event.preventDefault());
  $('admin-token').value = '';
  $('admin-token').addEventListener('input', () => { model.token = $('admin-token').value; });
  $('clear-token').addEventListener('click', () => { model.token = ''; $('admin-token').value = ''; notice('admin-message', '管理令牌已清除'); });
  window.addEventListener('pagehide', () => { model.token = ''; $('admin-token').value = ''; });
  $('lock-team-select').addEventListener('change', () => { if ($('lock-team-select').value) $('lock-team-input').value = $('lock-team-select').value; });
  $('lock-form').addEventListener('submit', event => {
    event.preventDefault();
    const team = $('lock-team-input').value || $('lock-team-select').value;
    if (!team.trim()) { notice('admin-message', '请选择或输入要锁定的队名', true); return; }
    if (confirmation(`确认锁定队伍「${team}」？其他队伍的 hello / 上报将被拒绝。`)) manage(() => postAdmin('team-lock', {team}, `已锁定队伍：${team}`));
  });
  $('unlock-team').addEventListener('click', () => { if (confirmation('确认解除队伍锁定，允许所有队伍上报？')) manage(() => postAdmin('team-lock', {team: null}, '已解除队伍锁定')); });
  $('start-session').addEventListener('click', () => { if (confirmation('确认开始计时？hello / 测试上报不会自动开始计时。')) manage(() => postAdmin('session/start', {}, '当前场次已开始计时')); });
  $('finish-session').addEventListener('click', () => { if (confirmation('确认结束当前场次？计时将冻结，场次将拒绝新的上报。')) manage(() => postAdmin('session/finish', {}, '当前场次已结束，计时已冻结')); });
  $('new-session').addEventListener('click', () => { if (confirmation('确认新建场次？当前场次将被切换，历史记录保留。')) manage(async () => {
    await postAdmin('session/new', {}, '新场次已创建，历史记录保留');
    $('session-filter').value = 'current';
    model.offset = 0;
    resetReports(false);
  }); });
  $('export-session').addEventListener('click', () => {
    const id = selectedSessionId();
    if (!id) return;
    manage(async () => {
      const {response, blob} = await request(`/api/admin/export?${new URLSearchParams({session_id: id})}`, {headers: authorization()}, null, true);
      const filename = exportFilename(response.headers.get('Content-Disposition'), `mock-judge-${id}.zip`);
      const url = URL.createObjectURL(blob);
      const link = node('a');
      link.href = url;
      link.download = filename;
      link.hidden = true;
      document.body.append(link);
      link.click();
      link.remove();
      window.setTimeout(() => URL.revokeObjectURL(url), 60000);
      notice('admin-message', `已发起 ZIP 下载：${filename}`);
    });
  });
  $('close-image').addEventListener('click', () => $('image-dialog').close());
  $('image-dialog').addEventListener('close', () => { model.gallery = null; $('large-image').removeAttribute('src'); });
  $('image-dialog').addEventListener('click', event => { if (event.target === $('image-dialog')) {
    const rect = $('image-dialog').getBoundingClientRect();
    if (event.clientX < rect.left || event.clientX > rect.right || event.clientY < rect.top || event.clientY > rect.bottom) $('image-dialog').close();
  } });
  $('previous-image').addEventListener('click', () => moveGallery(-1));
  $('next-image').addEventListener('click', () => moveGallery(1));
  $('large-image').addEventListener('error', () => { if (model.gallery) { $('image-error').hidden = false; $('large-image').hidden = true; } });
  $('image-dialog').addEventListener('keydown', event => {
    if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') { event.preventDefault(); moveGallery(event.key === 'ArrowLeft' ? -1 : 1); }
  });
  document.addEventListener('visibilitychange', () => { if (!document.hidden) poll(); });
  window.addEventListener('online', poll);
  window.setInterval(poll, 1000);
  window.setInterval(updateClock, 250);
  poll();
})();
