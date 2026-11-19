'use strict';
const $ = id => document.getElementById(id);
const letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
const criteria = [
	['barcode_detected', 'Detect a barcode'], ['barcode_decoded', 'Decode the barcode'],
	['hump_measured', 'Measure hump height'], ['line_following', 'Follow the line'],
	['instruction_followed', 'Follow barcode instruction'], ['obstacle_detected', 'Detect an obstacle'],
	['width_measured', 'Measure obstacle width'], ['obstacle_rejoined', 'Avoid obstacle & rejoin line'],
	['mqtt', 'Send MQTT telemetry'], ['handling_stopped', 'Handling / collision motor stop']
];
const defaults = { left: 'Z', right: 'A', straight: 'X', uturn: 'Y' };
const names = { left: 'LEFT', right: 'RIGHT', straight: 'STRAIGHT', uturn: 'U-TURN' };
let saved = null, busy = false, initialized = false, lastFrame = '', samples = [], boot = null;
for (const key of Object.keys(defaults)) {
	const select = $('rule-' + key);
	for (const letter of ['—', ...letters]) { const o = document.createElement('option'); o.value = letter === '—' ? '' : letter; o.textContent = letter; select.append(o); }
	select.value = defaults[key];
}
for (let i = 0; i < criteria.length; i++) {
	const row = document.createElement('div'); row.className = 'criterion'; row.id = 'criterion-' + criteria[i][0];
	const number = document.createElement('span'); number.className = 'number'; number.textContent = String(i + 1).padStart(2, '0');
	const title = document.createElement('span'); title.textContent = criteria[i][1];
	const status = document.createElement('span'); status.className = 'status'; status.textContent = 'WAITING';
	row.append(number, title, status); $('criteria').append(row);
}
const text = (id, value) => { $(id).textContent = value === null || value === undefined ? '—' : String(value); };
const cm = mm => mm === null || mm === undefined ? '—' : (mm / 10).toFixed(1) + ' cm';
const fmt = (n, suffix) => n === null || n === undefined ? '—' : n + suffix;
function status(message, error = false) { text('map-result', message); $('map-result').classList.toggle('error', error); }
function chips(id, values) {
	$(id).replaceChildren();
	for (const value of values) { const node = document.createElement('span'); node.className = 'chip'; node.textContent = value; $(id).append(node); }
}
function loadChoices(rules) {
	if (!rules) return status('No saved choices yet.');
	for (const [key, name] of Object.entries(names)) $('rule-' + key).value = [...letters].find(l => rules[l] === name) || '';
	status('Saved choices loaded. Apply to send them to the robot.');
}
async function api(path, body) {
	const response = await fetch('/csc14/api/' + path, body === undefined ? {} : {
		method: 'POST', headers: { 'content-type': 'application/json', 'x-csc14-dashboard': '1' }, body: JSON.stringify(body)
	});
	const data = await response.json();
	if (!response.ok) throw new Error(data.error || 'Dashboard service unavailable.');
	return data;
}
function lock(value) {
	busy = value;
	for (const id of ['apply-rules', 'read-rules', 'restore-rules']) $(id).disabled = value;
	for (const key of Object.keys(defaults)) $('rule-' + key).disabled = value;
	for (const button of document.querySelectorAll('[data-command]'))
		if (button.dataset.command !== 'STOP') button.disabled = value;
}
function showMap(rules) {
	const active = Object.entries(rules || {}).filter(([, a]) => a !== 'NONE').map(([l, a]) => l + ' → ' + a);
	chips('confirmed-rules', active.length ? active : ['No active rules confirmed']);
}
$('apply-rules').onclick = async () => {
	const rules = Object.fromEntries([...letters].map(l => [l, 'NONE']));
	for (const [key, name] of Object.entries(names)) {
		const l = $('rule-' + key).value;
		if (!l) continue;
		if (rules[l] !== 'NONE') return status('Choose a different letter for each instruction.', true);
		rules[l] = name;
	}
	lock(true); status('Sending 26 letter assignments and waiting for robot acknowledgements…');
	try { const r = await api('rules', { action: 'apply', rules }); saved = rules; showMap(r.confirmed); status('All 26 assignments acknowledged by the robot. Ready to resume.'); }
	catch (e) { status(e.message + ' Some rules may already be applied; read robot rules before retrying.', true); }
	finally { lock(false); }
};
$('read-rules').onclick = async () => {
	lock(true); status('Reading all 26 assignments from the robot…');
	try { const r = await api('rules', { action: 'read' }); showMap(r.confirmed); loadChoices(r.confirmed); status('Read all 26 assignments from the robot.'); }
	catch (e) { status(e.message, true); } finally { lock(false); }
};
$('restore-rules').onclick = () => loadChoices(saved);
for (const button of document.querySelectorAll('[data-command]')) button.onclick = async () => {
	const command = button.dataset.command;
	button.disabled = true;
	try { status(command + ': waiting for robot acknowledgement…'); await api('command', { command }); status(command + ' acknowledged by the robot.'); }
	catch (e) { status(e.message, true); } finally { button.disabled = false; }
};
function clearSession() {
 samples = []; lastFrame = '';
 for (const id of ['mode','speed','barcode','barcode-action','hump','hump-max','width','range','safety','shock','line-status','ir-state','wheel-left','wheel-right','steer','distance','pitch','roll','hump-count','peak','avoid-side','avoid-outcome','avoid-status','last-frame']) text(id, '—');
 for (const name of ['left','centre','right']) $('sensor-' + name).setAttribute('fill', '#fff');
 for (const name of ['left','right']) $('chart-' + name).setAttribute('points', '');
 $('pitch-bar').style.width = '0%'; $('roll-bar').style.width = '0%';
 chips('faults', ['Waiting for new boot telemetry']); showMap({});
 status('New robot session. Read or reapply saved barcode choices after stopping the robot.');
}
function render(s) {
 if (boot !== s.session_id) { clearSession(); boot = s.session_id; }
	const t = s.latest;
	text('robot-id', s.robot); text('clock', new Date().toLocaleTimeString('en-SG', { hour: '2-digit', minute: '2-digit' }));
	text('link-pill', s.live ? '● ROBOT ONLINE' : t ? '● ROBOT OFFLINE' : '● NO ROBOT DATA');
	$('link-pill').className = 'pill ' + (s.live ? 'live' : 'offline');
	$('side-dot').className = 'dot' + (s.live ? ' live' : ''); text('side-link', s.live ? 'Receiving live telemetry' : 'Waiting for robot');
	$('stale-banner').style.display = s.live ? 'none' : 'block';
	text('stale-banner', t ? 'Robot data is stale (' + Math.round(s.age_s) + ' seconds old). Measurements below are the last received values.' : 'No live telemetry received yet. Start the broker, enable the hotspot and connect the Pico.');
	saved = s.mapping.desired;
	if (!initialized && saved) { loadChoices(saved); initialized = true; }
	showMap(s.mapping.confirmed);
	if (t) {
		text('mode', t.mode_label === 'INIT' ? 'WAITING FOR BUDDIES' : t.mode_label); text('speed', t.speed_l_mms === null || t.speed_r_mms === null ? '—' : Math.round((t.speed_l_mms + t.speed_r_mms) / 2));
		const decoded = ['VALID','DUPLICATE'].includes(t.bc_result_label);
		text('barcode', decoded ? t.bc_symbol : t.bc_result_label === 'INVALID' ? 'INVALID' : '—');
		text('barcode-action', decoded ? (t.nav_label || 'Decoded · no assigned instruction') : t.bc_state_label);
		text('hump', cm(t.hump_last_mm)); text('hump-max', 'Highest this boot: ' + cm(t.hump_max_boot_mm));
		text('width', cm(t.obst_width_mm)); text('range', t.us_no_echo ? 'No echo' : cm(t.us_dist_mm)); text('range-context', t.us_context || 'sensor to surface');
		text('safety', t.collision ? 'LATCHED' : 'MONITORING'); text('shock', 'Peak shock: ' + fmt(t.shock_g, ' g'));
		text('line-status', t.line_state_label); text('ir-state', (t.ir_centre ? 'BLACK' : 'WHITE') + ' / ' + (t.ir_right ? 'BLACK' : 'WHITE'));
		for (const [name, value] of [['left', t.ir_left], ['centre', t.ir_centre], ['right', t.ir_right]]) $('sensor-' + name).setAttribute('fill', value ? '#264b3e' : '#fff');
		text('wheel-left', fmt(t.speed_l_mms, ' mm/s')); text('wheel-right', fmt(t.speed_r_mms, ' mm/s')); text('steer', t.steer); text('distance', fmt(t.odometer_cm === null ? null : (t.odometer_cm / 100).toFixed(2), ' m'));
		text('pitch', fmt(t.pitch_deg, '°')); text('roll', fmt(t.roll_deg, '°')); $('pitch-bar').style.width = Math.min(100, Math.abs(t.pitch_deg || 0) / 90 * 100) + '%'; $('roll-bar').style.width = Math.min(100, Math.abs(t.roll_deg || 0) / 90 * 100) + '%';
		text('hump-count', t.hump_count); text('peak', cm(t.hump_max_boot_mm)); text('avoid-side', t.avoid_side_label); text('avoid-outcome', t.avoid_outcome_label); text('avoid-status', t.avoid_state_label);
		chips('faults', t.faults && t.faults.length ? t.faults.map(f => typeof f === 'string' ? f : f.name || String(f)) : ['No active faults']);
		text('last-frame', 'Last frame · ' + new Date(t.received_at).toLocaleTimeString() + ' · ' + s.robot);
		if (s.live && lastFrame !== t.received_at) { samples.push([t.speed_l_mms, t.speed_r_mms]); samples = samples.slice(-60); lastFrame = t.received_at; }
		const max = Math.max(70, ...samples.flat().filter(Number.isFinite).map(Math.abs));
		for (let i = 0; i < 2; i++) {
			const points = samples.map((p, n) => [(n / Math.max(59, samples.length - 1) * 590 + 5).toFixed(1), (50 - (p[i] || 0) / max * 43).toFixed(1)].join(',')).join(' ');
			$('chart-' + (i ? 'right' : 'left')).setAttribute('points', points);
		}
	}
	let count = 0;
	for (const [key] of criteria) {
		const seen = key === 'mqtt' ? !!(s.evidence.frames > 0) : !!s.evidence[key];
		const row = $('criterion-' + key); row.classList.toggle('seen', seen); row.querySelector('.status').textContent = seen ? 'OBSERVED' : 'WAITING'; if (seen) count++;
	}
	text('evidence-count', count + ' / 10 observed'); text('event-count', s.events.length + ' recent');
	$('events').replaceChildren();
	if (!s.events.length) { const e = document.createElement('div'); e.className = 'empty'; e.textContent = 'No robot events received yet.'; $('events').append(e); }
	for (const event of s.events) {
		const row = document.createElement('div'); row.className = 'event ' + event.severity;
		const time = document.createElement('time'); time.textContent = new Date(event.received_at).toLocaleTimeString('en-SG', { hour12: false });
		const dot = document.createElement('i'); const message = document.createElement('p'); message.textContent = event.message;
		row.append(time, dot, message); $('events').append(row);
	}
}
async function poll() {
	try { render(await api('snapshot')); }
	catch (e) { text('link-pill', 'SERVICE UNAVAILABLE'); $('link-pill').className = 'pill offline'; $('stale-banner').style.display = 'block'; text('stale-banner', e.message); }
	setTimeout(poll, 1200);
}
poll();
