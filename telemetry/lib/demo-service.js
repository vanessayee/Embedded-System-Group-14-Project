'use strict';

const fs = require('fs');
const path = require('path');
const mqtt = require('mqtt');
const { Pool } = require('pg');
const { codec } = require('./protocol');
const ACTIONS = ['LEFT', 'RIGHT', 'STRAIGHT', 'U-TURN', 'NONE'];
const LETTERS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';

function mapArg(letter, action) {
	if (!LETTERS.includes(letter) || letter.length !== 1 || !ACTIONS.includes(action))
		throw new Error('Choose an A-Z letter and a supported action.');
	return (letter.charCodeAt(0) - 55) | (ACTIONS.indexOf(action) << 6);
}

function validateRules(rules) {
	if (!rules || typeof rules !== 'object' || Array.isArray(rules)) throw new Error('Rules must be an A-Z object.');
	const clean = {};
	for (const letter of LETTERS) {
		mapArg(letter, rules[letter]);
		clean[letter] = rules[letter];
	}
	if (Object.keys(rules).length !== 26) throw new Error('Provide exactly the 26 A-Z rules.');
	return clean;
}

class DemoService {
	constructor(cfg, deps = {}) {
		this.cfg = cfg;
		this.transport = deps.mqtt || mqtt;
		this.ackTimeoutMs = deps.ackTimeoutMs || 6500;
		this.db = deps.db || new Pool({ ...cfg.pg, max: 3, application_name: 'csc14-demo', connectionTimeoutMillis: 4000 });
		this.db.on('error', e => console.error('[demo database] ' + e.message));
		this.tail = Promise.resolve();
		this.confirmed = {};
		this.mappingBusy = false;
		this.boot = null;
		this.rulesFile = path.join(cfg.runtimeDir, 'barcode-rules.json');
		this.desired = null;
		try { this.desired = validateRules(JSON.parse(fs.readFileSync(this.rulesFile, 'utf8'))); } catch (_) {}
	}

	async snapshot() {
		const args = [this.cfg.robotId];
		const session = (await this.db.query("SELECT boot_id FROM csc14.boot_session WHERE robot_id=$1 AND source='live' ORDER BY first_rx_at DESC LIMIT 1", args)).rows[0] || null;
        const sessionArgs = [this.cfg.robotId, session ? session.boot_id : null];
        const latest = (await this.db.query("SELECT * FROM csc14.v_telemetry WHERE robot_id=$1 AND source='live' AND boot_id=$2 ORDER BY received_at DESC LIMIT 1", sessionArgs)).rows[0] || null;
		if (this.boot !== (session && session.boot_id)) {
			this.boot = session && session.boot_id;
			this.confirmed = {}; // never present a previous boot's map as confirmed
		}
		const events = (await this.db.query("SELECT received_at,kind,severity,message,data FROM csc14.event WHERE robot_id=$1 AND source='live' AND boot_id=$2 ORDER BY received_at DESC LIMIT 30", sessionArgs)).rows;
		const evidence = latest ? (await this.db.query(`
			SELECT count(*)::int AS frames,
			 bool_or(bc_state=1 OR bc_valid_mod64>0 OR bc_invalid_mod64>0) AS barcode_detected,
			 bool_or(bc_result=1 AND bc_symbol IS NOT NULL) AS barcode_decoded,
			 bool_or(hump_count>0 AND hump_last_mm IS NOT NULL) AS hump_measured,
			 bool_or(mode=1 AND line_state=0 AND (speed_l_mms>5 OR speed_r_mms>5)) AS line_following,
			 bool_or(avoid_state NOT IN (0) OR avoid_outcome IN (1,2)) AS obstacle_detected,
			 bool_or(obst_width_mm IS NOT NULL AND obst_width_mm>0) AS width_measured,
			 bool_or(avoid_outcome=1 AND mode=1) AS obstacle_rejoined,
			 bool_or(collision AND mode=4 AND abs(speed_l_mms)<=5 AND abs(speed_r_mms)<=5) AS handling_stopped
			FROM csc14.telemetry WHERE robot_id=$1 AND source='live' AND boot_id=$2`,
			[this.cfg.robotId, latest.boot_id])).rows[0] : {};
		const instruction = latest ? (await this.db.query(`
			SELECT EXISTS(SELECT 1 FROM csc14.event WHERE robot_id=$1 AND source='live'
			 AND boot_id=$2 AND kind='MODE' AND data->>'old_mode'='2'
			 AND data->>'new_mode'='1') AS done`, [this.cfg.robotId, latest.boot_id])).rows[0].done : false;
		evidence.instruction_followed = instruction;
		const age = latest ? (Date.now() - new Date(latest.received_at)) / 1000 : null;
		return { robot: this.cfg.robotId, session_id: session && session.boot_id, latest, age_s: age,
			live: age !== null && age <= 5, evidence, events,
			mapping: { confirmed: this.confirmed, desired: this.desired } };
	}

	// Normal configuration commands serialize; STOP bypasses the batch queue.
	command(name, arg = 0) {
		if (!['STOP', 'RESUME', 'CLEAR', 'REQ_STATUS', 'SET_NAV_MAP', 'GET_NAV_MAP'].includes(name))
			return Promise.reject(new Error('Unsupported dashboard command.'));
		if (name === 'STOP') return this.request(name, arg);
		if (this.mappingBusy && ['RESUME', 'CLEAR'].includes(name))
			return Promise.reject(new Error('Wait for barcode configuration to finish before clearing or resuming.'));
		const job = this.tail.then(() => this.request(name, arg));
		this.tail = job.catch(() => {});
		return job;
	}

	async request(name, arg) {
		if (['RESUME', 'CLEAR'].includes(name) && process.env.CSC14_ALLOW_MOTION_COMMANDS !== '1')
			throw new Error('Motion commands are disabled in the server configuration.');
		// Validate before allocating an ID or connecting.
		codec.encodeCommand(name, arg, 0, 0);
		const c = await this.db.connect();
		let row, frame, cmdId, seq;
		try {
			await c.query('BEGIN');
			await c.query("SELECT pg_advisory_xact_lock(hashtext('csc14-demo-command'))");
			const last = (await c.query("SELECT cmd_id,seq FROM csc14.command WHERE robot_id=$1 AND source='live' ORDER BY id DESC LIMIT 1", [this.cfg.robotId])).rows[0];
			cmdId = last ? (last.cmd_id + 1) % 256 : Math.floor(Math.random() * 256);
			seq = last ? (last.seq + 1) % 256 : 0;
			frame = codec.encodeCommand(name, arg, cmdId, seq);
			row = (await c.query(`INSERT INTO csc14.command(robot_id,source,topic,cmd,arg,cmd_id,seq,frame,requested_by)
			 VALUES($1,'live',$2,$3,$4,$5,$6,$7,'demo dashboard') RETURNING id`,
			[this.cfg.robotId, this.cfg.topics.live + '/cmd', name, arg, cmdId, seq, frame])).rows[0];
			await c.query('COMMIT');
		} catch (e) { await c.query('ROLLBACK'); throw e; } finally { c.release(); }
		let ack;
		try {
			ack = await new Promise((resolve, reject) => {
				const client = this.transport.connect({ host: this.cfg.mqtt.host, port: this.cfg.mqtt.port,
					clientId: 'csc14-demo-' + process.pid + '-' + cmdId, reconnectPeriod: 0, connectTimeout: 4000 });
				let done = false;
				const finish = (err, value) => {
					if (done) return;
					done = true; clearTimeout(timer); client.end(true);
					err ? reject(err) : resolve(value);
				};
				const timer = setTimeout(() => finish(new Error('No robot acknowledgement. Check Pico Wi-Fi, broker and firmware.')), this.ackTimeoutMs);
				client.on('error', e => finish(e));
				client.on('connect', () => client.subscribe(this.cfg.topics.live + '/ack', { qos: 0 }, err => {
					if (err) return finish(err);
					client.publish(this.cfg.topics.live + '/cmd', frame, { qos: 0, retain: false }, e => e && finish(e));
				}));
				client.on('message', (_topic, payload) => {
					const a = codec.decode(payload);
					if (!a.error && !a.issues.length && a.kind === 'ack' && a.cmd_id === cmdId && a.cmd === name) finish(null, a);
				});
			});
		} catch (e) {
			await this.db.query("UPDATE csc14.command SET status='send_failed' WHERE id=$1 AND status='pending'", [row.id]);
			throw e;
		}
		await this.db.query("UPDATE csc14.command SET status=$2,ack_result=$3,ack_detail=$4,acked_at=now() WHERE id=$1",
			[row.id, ack.result === 'OK' ? 'accepted' : 'rejected', ack.result, String(ack.detail)]);
		if (ack.result !== 'OK') throw new Error('Robot rejected ' + name + ': ' + ack.result + '. Stop the car and clear any queued barcode first.');
		if (name === 'SET_NAV_MAP' || name === 'GET_NAV_MAP') {
			const action = ACTIONS[Number(ack.detail)];
			if (!action) throw new Error('Robot returned an unsupported map response. Reflash the current UF2.');
			const letter = String.fromCharCode((arg & 63) + 55);
			this.confirmed[letter] = action;
		}
		return { accepted: true, command: name, detail: ack.detail, cmd_id: cmdId };
	}

	async transferMap(rules) {
		for (const letter of LETTERS) {
			await this.command(rules ? 'SET_NAV_MAP' : 'GET_NAV_MAP',
				rules ? mapArg(letter, rules[letter]) : letter.charCodeAt(0) - 55);
			await new Promise(r => setTimeout(r, 220));
		}
		return { confirmed: { ...this.confirmed } };
	}

	async syncMap() {
		if (this.mappingBusy) throw new Error('Another barcode configuration request is running.');
		const s = await this.snapshot();
		if (!s.live) throw new Error('Robot telemetry is offline or stale. No map has been confirmed.');
		if (this.mappingBusy) throw new Error('Another barcode configuration request is running.');
		this.mappingBusy = true;
		try {
			return await this.transferMap();
		} finally { this.mappingBusy = false; }
	}

	async applyRules(input) {
		if (this.mappingBusy) throw new Error('Another barcode configuration request is running.');
		const rules = validateRules(input);
		const s = await this.snapshot();
		if (!s.live || !s.latest || !['SAFE_STOP', 'INIT'].includes(s.latest.mode_label))
			throw new Error('Stop the robot before applying barcode rules.');
		if (this.mappingBusy) throw new Error('Another barcode configuration request is running.');
		this.mappingBusy = true;
		try {
			this.desired = rules;
			fs.mkdirSync(path.dirname(this.rulesFile), { recursive: true });
			fs.writeFileSync(this.rulesFile + '.tmp', JSON.stringify(rules, null, 2));
			fs.renameSync(this.rulesFile + '.tmp', this.rulesFile);
			const result = await this.transferMap(rules);
			return { accepted: true, ...result, note: 'Robot map is RAM-only. Reapply after power cycling.' };
		} finally { this.mappingBusy = false; }
	}

	async close() { await this.db.end(); }
}

module.exports = { DemoService, mapArg, validateRules, ACTIONS, LETTERS };
