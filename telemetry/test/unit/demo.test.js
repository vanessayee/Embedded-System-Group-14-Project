'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { DemoService, mapArg, validateRules, ACTIONS, LETTERS } = require('../../lib/demo-service');
const { codec } = require('../../lib/protocol');
const { flows } = require('../../nodered/build-flows');

test('all 26 letters and five actions round-trip through the wire codec', () => {
	for (const l of LETTERS) for (const a of ACTIONS) {
		const arg = mapArg(l, a);
		const d = codec.decode(codec.encodeCommand('SET_NAV_MAP', arg, 7, 8));
		assert.deepEqual(d.issues, []); assert.equal(d.arg & 63, l.charCodeAt(0) - 55);
		assert.equal(d.arg >> 6, ACTIONS.indexOf(a));
		const q = codec.decode(codec.encodeCommand('GET_NAV_MAP', l.charCodeAt(0) - 55, 8, 9));
		assert.deepEqual(q.issues, []);
	}
	for (const [l, a] of [['a','LEFT'],['*','LEFT'],['AA','RIGHT'],['Z','FASTER']])
		assert.throws(() => mapArg(l,a));
	assert.throws(() => codec.encodeCommand('SET_NAV_MAP', 10 | 5 << 6, 1, 1));
	assert.throws(() => codec.encodeCommand('GET_NAV_MAP', 36, 1, 1));
});

test('complete mapping validation rejects incomplete, extra and unsupported assignments', () => {
	const rules = Object.fromEntries([...LETTERS].map(l => [l, 'NONE']));
	rules.Z = 'LEFT'; rules.A = 'RIGHT';
	assert.deepEqual(validateRules(rules), rules);
	assert.throws(() => validateRules({ A: 'RIGHT' }));
	assert.throws(() => validateRules({ ...rules, a: 'LEFT' }));
	assert.throws(() => validateRules({ ...rules, B: 'FASTER' }));
});

function fixture(result = 'OK', silent = false) {
	const writes = [];
	let last = null;
	const db = new EventEmitter();
	db.query = async (sql, args) => {
		writes.push({ sql, args });
		if (sql.startsWith('SELECT cmd_id')) return { rows: last ? [last] : [] };
		if (sql.startsWith('INSERT INTO csc14.command')) {
			last = { cmd_id: args[4], seq: args[5] }; return { rows: [{ id: String(writes.length) }] };
		}
		return { rows: [] };
	};
	db.connect = async () => ({ query: db.query, release() {} }); db.end = async () => {};
	const transport = { connect() {
		const c = new EventEmitter(); c.end = () => {};
		c.subscribe = (topic, _qos, cb) => { assert.equal(topic, 'test/csc14/ack'); cb(); };
		c.publish = (topic, frame, opts, cb) => {
			assert.equal(topic, 'test/csc14/cmd'); assert.equal(opts.retain, false); assert.equal(opts.qos, 0);
			const d = codec.decode(frame); assert.deepEqual(d.issues, []);
			cb();
			if (silent) return;
			const ack = (id, cmd) => codec.encodeFrame(codec.TYPE.ACK, 0, {
				cmd_id: id, cmd, result: codec.ENUMS.ack_result.indexOf(result), detail: d.cmd === 'RESET' ? 31 : d.arg >> 6
			});
			// An unrelated ACK must never confirm this assignment.
			c.emit('message', 'test/csc14/ack', ack((d.cmd_id + 1) % 256, d.raw.cmd));
			c.emit('message', 'test/csc14/ack', ack(d.cmd_id, d.raw.cmd));
		};
		queueMicrotask(() => c.emit('connect')); return c;
	} };
	const runtimeDir = fs.mkdtempSync(path.join(os.tmpdir(), 'csc14-demo-test-'));
	const service = new DemoService({ pg: {}, robotId: 'test-car', topics: { live: 'test/csc14' },
		mqtt: { host: '127.0.0.1', port: 1 }, runtimeDir }, { db, mqtt: transport, ackTimeoutMs: 30 });
	return { service, writes, runtimeDir };
}

test('mapping is confirmed only by matching robot ACK, with non-retained binary publishing and audit row', async () => {
	const f = fixture();
	try {
		const r = await f.service.command('SET_NAV_MAP', mapArg('Z','LEFT'));
		assert.equal(r.accepted, true); assert.equal(f.service.confirmed.Z, 'LEFT');
		assert.ok(f.writes.some(w => w.sql.startsWith('INSERT INTO csc14.command')));
		assert.ok(f.writes.some(w => w.args && w.args[1] === 'accepted'));
	} finally { await f.service.close(); fs.rmSync(f.runtimeDir, { recursive: true }); }
});

test('rejected and missing acknowledgements cannot appear as applied mappings', async () => {
	for (const [result, silent] of [['REJ_STATE',false],['OK',true]]) {
		const f = fixture(result, silent);
		try {
			await assert.rejects(f.service.command('SET_NAV_MAP', mapArg('W','LEFT')));
			assert.equal(f.service.confirmed.W, undefined);
		} finally { await f.service.close(); fs.rmSync(f.runtimeDir, { recursive: true }); }
	}
});

test('configuration blocks resume and overlapping changes while STOP bypasses the batch', async () => {
	const f = fixture();
	try {
		f.service.mappingBusy = true;
		await assert.rejects(f.service.command('RESUME'), /configuration/);
		await assert.rejects(f.service.applyRules({}), /configuration/);
		const stopped = await f.service.command('STOP');
		assert.equal(stopped.accepted, true);
	} finally { await f.service.close(); fs.rmSync(f.runtimeDir, { recursive: true }); }
});

test('HTTP controls reject cross-origin or missing-header requests before any command', async () => {
	const body = flows.find(f => f.id === 'demo_http_4').func;
	const AsyncFunction = Object.getPrototypeOf(async function(){}).constructor;
	const handle = new AsyncFunction('msg','node','global', body);
	for (const headers of [{ host:'127.0.0.1:1880' },
		{ host:'127.0.0.1:1880', origin:'http://unrelated.example', 'x-csc14-dashboard':'1' }]) {
		let called = false;
		const r = await handle({ req:{ method:'POST',path:'/csc14/api/command',protocol:'http',headers,body:{command:'STOP'} } },
			{}, { get:()=>({command(){called=true;}}) });
		assert.equal(r.statusCode,403); assert.equal(called,false);
	}
});

test('physical reset selects new boot before telemetry arrives and scopes all current-run data', async () => {
 const f = fixture();
 try {
  let session = {boot_id:'old',first_rx_at:new Date(1000)};
  f.service.db.query = async (sql,args) => {
   f.writes.push({sql,args});
   if(sql.startsWith('SELECT boot_id FROM')) return {rows:[session]};
   if(sql.startsWith('SELECT * FROM csc14.v_telemetry')) {
    assert.equal(args[1],session.boot_id); assert.match(sql,/boot_id=\$2/);
    return {rows:[]};
   }
   if(sql.includes('FROM csc14.event')) {assert.match(sql,/boot_id=\$2/);assert.equal(args[1],session.boot_id);}
   return {rows:[]};
  };
  await f.service.snapshot(); f.service.confirmed.A='RIGHT';
  session = {boot_id:'new',first_rx_at:new Date(2000)};
  const s = await f.service.snapshot();
  assert.equal(s.session_id,'new'); assert.equal(s.latest,null); assert.deepEqual(s.evidence,{instruction_followed:false});
  assert.deepEqual(s.mapping.confirmed,{}); assert.deepEqual(s.events,[]);
  assert.ok(!f.writes.some(w => w.sql.includes('FROM csc14.command')), 'snapshot does not query unused command history');
 } finally {await f.service.close(); fs.rmSync(f.runtimeDir,{recursive:true});}
});


test('read and apply mapping batches cover all 26 letters and release the batch lock', async () => {
 const f = fixture();
 try {
  f.service.snapshot = async () => ({live:true,latest:{mode_label:'SAFE_STOP'}});
  const sent = [];
  f.service.command = async (name,arg) => {
   assert.equal(f.service.mappingBusy,true);
   sent.push([name,arg]);
   f.service.confirmed[String.fromCharCode((arg & 63)+55)] = name === 'SET_NAV_MAP' ? ACTIONS[arg >> 6] : 'NONE';
  };
  const read = await f.service.syncMap();
  assert.deepEqual(sent,[...LETTERS].map(l=>['GET_NAV_MAP',l.charCodeAt(0)-55]));
  assert.equal(Object.keys(read.confirmed).length,26);
  assert.equal(f.service.mappingBusy,false);
  sent.length=0;
  const rules=Object.fromEntries([...LETTERS].map((l,i)=>[l,ACTIONS[i % ACTIONS.length]]));
  const applied=await f.service.applyRules(rules);
  assert.deepEqual(sent,[...LETTERS].map(l=>['SET_NAV_MAP',mapArg(l,rules[l])]));
  assert.deepEqual(applied.confirmed,rules);
  assert.deepEqual(JSON.parse(fs.readFileSync(f.service.rulesFile,'utf8')),rules);
  assert.equal(f.service.mappingBusy,false);
  f.service.command=async()=>{throw new Error('transfer interrupted');};
  await assert.rejects(f.service.syncMap(),/transfer interrupted/);
  assert.equal(f.service.mappingBusy,false);
 } finally {await f.service.close();fs.rmSync(f.runtimeDir,{recursive:true});}
});


test('RESET bypasses a configuration batch, cancels queued commands and clears confirmation', async () => {
    const before = process.env.CSC14_ALLOW_MOTION_COMMANDS;
    process.env.CSC14_ALLOW_MOTION_COMMANDS = '1';
    const f = fixture();
    try {
        f.service.mappingBusy = true;
        f.service.confirmed.A = 'RIGHT';
        const cancelled = assert.rejects(f.service.command('REQ_STATUS'), /cancelled by reset/);
        const reset = await f.service.command('RESET');
        await cancelled;
        assert.equal(reset.accepted, true);
        assert.equal(reset.command, 'RESET');
        assert.equal(reset.detail, 31);
        assert.deepEqual(f.service.confirmed, {});
    } finally {
        if (before === undefined) delete process.env.CSC14_ALLOW_MOTION_COMMANDS;
        else process.env.CSC14_ALLOW_MOTION_COMMANDS = before;
        await f.service.close(); fs.rmSync(f.runtimeDir,{recursive:true});
    }
});

test('reset generation interrupts an in-progress map transfer', async () => {
    const f = fixture();
    try {
        let calls = 0;
        f.service.command = async () => { calls++; f.service.controlGeneration++; };
        await assert.rejects(f.service.transferMap(), /cancelled by reset/);
        assert.equal(calls, 1);
    } finally {await f.service.close();fs.rmSync(f.runtimeDir,{recursive:true});}
});


test('mapping retries one missing ACK, only for the same idempotent letter operation', async () => {
    for (const rules of [undefined,Object.fromEntries([...LETTERS].map(l=>[l,'NONE']))]) {
        const f=fixture();
        try {
            const calls=[];
            f.service.command=async(name,arg)=>{
                calls.push([name,arg]);
                if(calls.length===1){const e=new Error('missing ACK');e.code='ACK_TIMEOUT';throw e;}
                if(calls.length===3)throw new Error('end scenario');
                return {accepted:true};
            };
            await assert.rejects(f.service.transferMap(rules),/end scenario/);
            assert.deepEqual(calls[0],calls[1]);
            assert.notDeepEqual(calls[1],calls[2]);
        }finally{await f.service.close();fs.rmSync(f.runtimeDir,{recursive:true});}
    }
});

test('mapping retry budget is bounded and a rejected command is never retried', async () => {
    for(const code of ['ACK_TIMEOUT','REJECTED']){
        const f=fixture();
        try{
            let calls=0;
            f.service.command=async()=>{calls++;const e=new Error('failed');e.code=code;throw e;};
            await assert.rejects(f.service.transferMap(),/failed/);
            assert.equal(calls,code==='ACK_TIMEOUT'?2:1);
        }finally{await f.service.close();fs.rmSync(f.runtimeDir,{recursive:true});}
    }
});
