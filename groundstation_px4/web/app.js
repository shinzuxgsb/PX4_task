'use strict';

const $ = (id) => document.getElementById(id);
const POLL_MS = 200;
const FACE_COLORS = { A: '#1769aa', B: '#18885c', C: '#a96800', D: '#7651a8' };

let field = null;
let state = null;
let cameraConfig = null;
let plannedRoute = null;
let offline = false;

async function api(path, options = {}) {
  const response = await fetch(path, { cache: 'no-store', ...options });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json();
}

async function post(path, body) {
  return api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });
}

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, (char) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  }[char]));
}

function statusClass(element, stateName) {
  element.classList.remove('ok', 'warn', 'bad');
  if (stateName) element.classList.add(stateName);
}

function missionLabel(value) {
  return ({
    idle: '待命',
    running: '执行中',
    finished: '已完成',
    estop: '已停止',
  }[value] || value || '待命');
}

function px4Label(value) {
  return ({
    WAIT_TOPICS: '等待数据',
    WAIT_STABLE: '定位稳定中',
    WAIT_OFFBOARD: '等待 OFFBOARD',
    WAIT_GO: '可以起飞',
    ARMING: '正在解锁',
    START_MISSION: '启动任务',
    FLIGHT: '任务飞行',
    LAND: '降落中',
    HANDOVER: '已交回',
    FINISHED: '已完成',
  }[value] || value || '未知');
}

function drawMap() {
  const canvas = $('map');
  const box = canvas.parentElement;
  const rect = box.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, rect.width);
  const height = Math.max(260, rect.height);
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  canvas.style.width = `${width}px`;
  canvas.style.height = `${height}px`;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  if (!field) return;
  const area = field.area;
  const pad = 28;
  const scale = Math.min(
    (width - pad * 2) / area.w,
    (height - pad * 2) / area.h,
  );
  const ox = (width - area.w * scale) / 2;
  const oy = (height - area.h * scale) / 2;
  const X = (x) => ox + x * scale;
  const Y = (y) => oy + (area.h - y) * scale;

  ctx.fillStyle = '#f7fafb';
  ctx.fillRect(X(0), Y(area.h), area.w * scale, area.h * scale);
  ctx.strokeStyle = '#d6e0e7';
  ctx.lineWidth = 1;
  ctx.strokeRect(X(0), Y(area.h), area.w * scale, area.h * scale);

  ctx.strokeStyle = '#e5ebef';
  ctx.fillStyle = '#9aa9b5';
  ctx.font = '10px monospace';
  for (let x = 0; x <= area.w; x += 50) {
    ctx.beginPath();
    ctx.moveTo(X(x), Y(0));
    ctx.lineTo(X(x), Y(area.h));
    ctx.stroke();
    if (x % 100 === 0) ctx.fillText(`${x}`, X(x) + 2, Y(0) + 12);
  }
  for (let y = 0; y <= area.h; y += 50) {
    ctx.beginPath();
    ctx.moveTo(X(0), Y(y));
    ctx.lineTo(X(area.w), Y(y));
    ctx.stroke();
    if (y % 100 === 0) ctx.fillText(`${y}`, X(0) - 22, Y(y) + 3);
  }

  field.racks.forEach((rack) => {
    const [x0, y0, x1, y1] = rack.rect;
    ctx.fillStyle = '#d7e0e6';
    ctx.strokeStyle = '#b8c7d2';
    ctx.fillRect(X(x0), Y(y1), Math.max(2, (x1 - x0) * scale), (y1 - y0) * scale);
    ctx.strokeRect(X(x0), Y(y1), Math.max(2, (x1 - x0) * scale), (y1 - y0) * scale);
  });

  field.faces.forEach((face) => {
    const color = FACE_COLORS[face.id] || '#6d7d8c';
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(X(face.from[0]), Y(face.from[1]));
    ctx.lineTo(X(face.to[0]), Y(face.to[1]));
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.font = 'bold 12px "Microsoft YaHei", sans-serif';
    ctx.fillText(`${face.id} 面`, X(face.label[0]) - 12, Y(face.label[1]));
  });

  const takeoff = field.takeoff;
  ctx.strokeStyle = '#526a7b';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(
    X(takeoff.x - takeoff.size / 2),
    Y(takeoff.y + takeoff.size / 2),
    takeoff.size * scale,
    takeoff.size * scale,
  );
  ctx.fillStyle = '#526a7b';
  ctx.font = '11px "Microsoft YaHei", sans-serif';
  ctx.fillText('起飞点', X(takeoff.x) - 17, Y(takeoff.y + takeoff.size / 2) - 5);

  const landing = field.landing;
  ctx.beginPath();
  ctx.arc(X(landing.x), Y(landing.y), landing.radius * scale, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(24,136,92,.10)';
  ctx.fill();
  ctx.strokeStyle = '#18885c';
  ctx.stroke();
  ctx.fillStyle = '#18885c';
  ctx.fillText('降落点', X(landing.x) - 17, Y(landing.y + landing.radius) - 5);

  const route = plannedRoute || state?.mission?.route;
  if (route?.polyline?.length > 1) {
    ctx.save();
    ctx.strokeStyle = '#1769aa';
    ctx.lineWidth = 1.7;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    route.polyline.forEach(([x, y], index) => {
      if (index === 0) ctx.moveTo(X(x), Y(y));
      else ctx.lineTo(X(x), Y(y));
    });
    ctx.stroke();
    ctx.restore();
  }

  if (state?.trail?.length > 1) {
    ctx.strokeStyle = 'rgba(169,104,0,.78)';
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    state.trail.forEach(([x, y], index) => {
      if (index === 0) ctx.moveTo(X(x), Y(y));
      else ctx.lineTo(X(x), Y(y));
    });
    ctx.stroke();
  }

  const mapping = state?.mapping || {};
  field.slots.forEach((slot) => {
    const px = slot.x + slot.normal[0] * 18;
    const py = slot.y + slot.normal[1] * 18;
    const known = mapping[slot.slot] !== undefined;
    ctx.beginPath();
    ctx.arc(X(px), Y(py), Math.max(7, 9 * scale), 0, Math.PI * 2);
    ctx.fillStyle = known ? '#18885c' : '#ffffff';
    ctx.fill();
    ctx.strokeStyle = known ? '#18885c' : '#9eb0bd';
    ctx.lineWidth = 1.2;
    ctx.stroke();
    ctx.fillStyle = known ? '#ffffff' : '#6d7d8c';
    ctx.font = 'bold 9px monospace';
    ctx.fillText(known ? String(mapping[slot.slot]) : slot.slot.slice(1), X(px) - 4, Y(py) + 3);
  });

  const telemetry = state?.telemetry;
  if (telemetry?.last_rx) {
    const cx = X(telemetry.x);
    const cy = Y(telemetry.y);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-Number(telemetry.yaw || 0) * Math.PI / 180);
    ctx.beginPath();
    ctx.moveTo(13, 0);
    ctx.lineTo(-9, 9);
    ctx.lineTo(-4, 0);
    ctx.lineTo(-9, -9);
    ctx.closePath();
    ctx.fillStyle = '#14202b';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.4;
    ctx.stroke();
    ctx.restore();
  }
}

function renderStatus() {
  const px4 = state?.px4 || {};
  const links = state?.links || {};
  const ws = links.ws || {};
  const wsConnected = Boolean(
    (ws.ok && Number(ws.clients || 0) > 0) || links.mock?.ok,
  );

  statusClass($('linkState'), wsConnected ? 'ok' : 'bad');
  $('linkState').innerHTML = `<i></i>${wsConnected ? '机载已连接' : '机载未连接'}`;
  statusClass($('ekfState'), px4.ekf_ok ? 'ok' : 'warn');
  $('ekfState').innerHTML = `<i></i>${px4.ekf_ok ? 'EKF 正常' : 'EKF 等待'}`;

  const armed = Boolean(px4.armed);
  statusClass($('flightState'), armed ? 'warn' : 'ok');
  $('flightState').innerHTML = `<i></i>${armed ? '已解锁' : '未解锁'}`;

  $('px4Phase').textContent = px4Label(px4.state);
  $('px4Message').textContent = px4.error || px4.message ||
    (px4.takeoff_available ? '预检已通过，可以执行盘点' : '等待机载预检完成');
  $('flightMode').textContent = px4.mode || '--';
  $('battery').textContent = px4.battery_pct == null ? '--' : `${px4.battery_pct}%`;

  const telemetry = state?.telemetry || {};
  $('altitude').textContent = telemetry.has_height === false || telemetry.z == null
    ? '--' : `${Number(telemetry.z).toFixed(0)} cm`;
  $('position').textContent = telemetry.x == null
    ? '--' : `${Number(telemetry.x).toFixed(0)} / ${Number(telemetry.y).toFixed(0)}`;

  const mission = state?.mission || {};
  $('missionState').textContent = mission.phase || missionLabel(mission.state);
  $('progress').textContent = `${(state?.records || []).length} / ${state?.total_slots || 0}`;

  const available = Boolean(px4.takeoff_available);
  $('btnStart').disabled = !available;
  $('startHint').textContent = available
    ? '预检通过，点击后自动解锁'
    : wsConnected ? '等待 WAIT_GO 预检状态' : '等待机载端连接';

  const route = plannedRoute || mission.route;
  if (route) {
    $('routeTitle').textContent = route.mode === 'search' ? '搜索航线' : '遍历航线';
    $('routeMeta').textContent =
      `航程 ${(Number(route.length_cm) / 100).toFixed(1)} m · ` +
      `预计 ${Number(route.eta_s).toFixed(0)} s · ` +
      `${route.waypoints?.length || 0} 个点`;
  }

  const footer = [
    `任务 ${missionLabel(mission.state)}`,
    `用时 ${Number(mission.elapsed || 0).toFixed(1)}s`,
    `运行 ${Number(state?.uptime || 0).toFixed(0)}s`,
  ];
  if (offline) footer.unshift('地面站服务连接中断');
  $('footerInfo').textContent = footer.join(' · ');
}

function renderResults() {
  const records = (state?.records || []).slice().reverse();
  $('records').querySelector('tbody').innerHTML = records.map((record) => `
    <tr>
      <td><strong>${escapeHtml(record.no)}</strong></td>
      <td>${escapeHtml(record.slot)}</td>
      <td>${Number(record.z).toFixed(0)} cm</td>
    </tr>
  `).join('') || '<tr><td colspan="3" style="color:#8a99a5">尚无盘点结果</td></tr>';

  const mapping = state?.mapping || {};
  $('slotGrid').innerHTML = (field?.slots || []).map((slot) => {
    const value = mapping[slot.slot];
    const done = value !== undefined;
    return `<div class="slot ${done ? 'done' : ''}">
      <span>${escapeHtml(slot.slot)}</span>
      <strong>${done ? escapeHtml(value) : '·'}</strong>
    </div>`;
  }).join('');

  const query = state?.query;
  if (query?.no !== undefined) {
    $('queryResult').innerHTML = query.found
      ? `货物 <strong>${escapeHtml(query.no)}</strong> 位于 <strong>${escapeHtml(query.slot)}</strong>，
        坐标 ${Number(query.record.x).toFixed(0)}, ${Number(query.record.y).toFixed(0)},
        ${Number(query.record.z).toFixed(0)} cm`
      : `货物 ${escapeHtml(query.no)} 尚未盘点`;
  }
}

function renderEvents() {
  const log = $('log');
  const events = state?.events || [];
  const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 24;
  log.innerHTML = events.map((event) => {
    const date = new Date(Number(event.t) * 1000);
    const time = [date.getHours(), date.getMinutes(), date.getSeconds()]
      .map((value) => String(value).padStart(2, '0')).join(':');
    return `<div class="event-row">
      <span class="event-time">${time}</span>
      <span class="event-${escapeHtml(event.level || 'info')}">${escapeHtml(event.text)}</span>
    </div>`;
  }).join('') || '<div class="event-info">暂无事件</div>';
  if (atBottom) log.scrollTop = log.scrollHeight;
}

function render() {
  if (!state || !field) return;
  renderStatus();
  renderResults();
  renderEvents();
  drawMap();
}

function connectCamera() {
  if (!cameraConfig?.enabled) {
    $('cameraHint').textContent = '地面站配置未启用相机';
    return;
  }
  const image = $('cameraFeed');
  const separator = cameraConfig.url.includes('?') ? '&' : '?';
  image.src = `${cameraConfig.url}${separator}t=${Date.now()}`;
  $('cameraStatus').textContent = '连接中';
  $('cameraStatus').className = 'camera-badge';
}

async function sendCommand(command, extra = {}) {
  try {
    const response = await post('/api/cmd', { cmd: command, ...extra });
    if (response?.route) plannedRoute = response.route;
    if (response?.ok === false) throw new Error(response.error || '指令被拒绝');
    return response;
  } catch (error) {
    alert(`指令发送失败：${error.message}`);
    return null;
  }
}

async function refreshRoute(mode = 'sweep', no = null) {
  const query = new URLSearchParams({ mode });
  if (no != null) query.set('no', String(no));
  plannedRoute = await api(`/api/route?${query}`);
  render();
}

function bindActions() {
  $('btnStart').addEventListener('click', async () => {
    if (!confirm('确认飞机位于起飞点、人员撤离，并起飞执行遍历盘点？')) return;
    $('btnStart').disabled = true;
    $('startHint').textContent = '已发送起飞请求';
    await sendCommand('start_sweep');
    render();
  });

  $('btnTarget').addEventListener('click', async () => {
    const no = Number($('targetNo').value || 0);
    if (!no) {
      alert('请输入目标货物编号');
      return;
    }
    if (!confirm(`确认起飞并定向盘点货物 ${no}？`)) return;
    await sendCommand('start_target', { no });
    render();
  });

  $('btnLand').addEventListener('click', () => sendCommand('land'));
  $('btnEstop').addEventListener('click', () => {
    if (confirm('确认立即请求 PX4 降落？')) sendCommand('estop');
  });
  $('btnPlanSweep').addEventListener('click', () => refreshRoute('sweep'));
  $('btnReset').addEventListener('click', async () => {
    if (!confirm('确认清空本次盘点结果？')) return;
    plannedRoute = null;
    await sendCommand('reset');
    await refreshRoute('sweep');
  });
  $('btnSave').addEventListener('click', async () => {
    const response = await sendCommand('save_session');
    if (response?.path) alert(`已保存到：${response.path}`);
  });
  $('btnQuery').addEventListener('click', async () => {
    const no = Number($('queryNo').value || 0);
    if (no) await sendCommand('query', { no });
  });
  $('queryNo').addEventListener('keydown', (event) => {
    if (event.key === 'Enter') $('btnQuery').click();
  });
  $('btnCameraReload').addEventListener('click', connectCamera);
  window.addEventListener('resize', drawMap);

  $('cameraFeed').addEventListener('load', () => {
    const image = $('cameraFeed');
    if (!image.naturalWidth) return;
    $('cameraEmpty').style.display = 'none';
    $('cameraStatus').textContent = '实时';
    $('cameraStatus').className = 'camera-badge ok';
  });
  $('cameraFeed').addEventListener('error', () => {
    $('cameraEmpty').style.display = 'flex';
    $('cameraHint').textContent = '检查机载预检是否运行、相机和网络是否正常';
    $('cameraStatus').textContent = '连接失败';
    $('cameraStatus').className = 'camera-badge bad';
  });
}

async function poll() {
  try {
    state = await api('/api/state');
    offline = false;
    render();
  } catch (error) {
    offline = true;
    $('footerInfo').textContent = '地面站服务连接中断';
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

async function boot() {
  const [fieldResponse, configResponse] = await Promise.all([
    api('/api/field'),
    api('/api/config'),
  ]);
  field = fieldResponse.field;
  cameraConfig = configResponse.runtime.camera || { enabled: false };
  $('fieldName').textContent =
    `${field.name} · ${field.area.w} × ${field.area.h} cm · ${field.total_slots} 个点位`;
  $('cameraLabel').textContent = cameraConfig.label || '机载摄像头';
  $('cameraHint').textContent = cameraConfig.url || '未配置相机地址';
  bindActions();
  connectCamera();
  await refreshRoute('sweep');
  await poll();
}

boot().catch((error) => {
  document.body.innerHTML =
    `<div style="padding:40px;color:#b5353f;font-family:sans-serif">` +
    `地面站界面加载失败：${escapeHtml(error.message)}</div>`;
});
