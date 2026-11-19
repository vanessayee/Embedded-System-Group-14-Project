/*
 * Active artifacts: generated Node-RED flows compile, ingestion cannot
 * publish, and the sole Grafana dashboard embeds the live mission page.
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const flowsGen = require('../../nodered/build-flows.js');

test('flows.json is generated from the current source', () => {
	assert.equal(fs.readFileSync(path.join(ROOT, 'nodered', 'flows.json'), 'utf8'), flowsGen.text,
		'run: node nodered/build-flows.js');
});

test('flow: subscribes as Buffer, never publishes, every Function node compiles', () => {
	const flows = flowsGen.flows;
	const ins = flows.filter((n) => n.type === 'mqtt in');
	assert.equal(ins.length, 2);
	for (const n of ins) {
		assert.equal(n.datatype, 'buffer');
		assert.match(n.topic, /^\$\{CSC14_SUB_(LIVE|SIM)\}$/);
	}
	assert.equal(flows.filter((n) => n.type === 'mqtt out').length, 0, 'the ingestion flow must not publish');
	const ids = new Set(flows.map((n) => n.id));
	for (const n of flows) {
		for (const w of n.wires || []) { for (const id of w) { assert.ok(ids.has(id), `${n.id} -> ${id}`); } }
		if (n.type === 'function') {
			for (const body of [n.func, n.finalize]) {
				if (body) { new vm.Script(`(async function (msg, node, global) {\n${body}\n})`); }
			}
			assert.equal(n.outputs, (n.wires || []).length);
		}
	}
	assert.deepEqual(flows.find((n) => n.type === 'status').scope.sort(), ins.map((n) => n.id).sort());
});

test('main Grafana embeds the live controls, with no sample dashboard provisioned', () => {
 const main=JSON.parse(fs.readFileSync(path.join(ROOT,'grafana/dashboards/csc14-robot.json'),'utf8'));
 assert.equal(main.uid,'csc14-robot');assert.equal(main.panels.length,1);
 assert.match(main.panels[0].options.content,/src="http:\/\/127\.0\.0\.1:1880\/csc14\/dashboard"/);
 assert.doesNotMatch(main.panels[0].options.content,/preview=grafana/);
 assert.deepEqual(fs.readdirSync(path.join(ROOT,'grafana/dashboards')).filter(x=>x.endsWith('.json')),['csc14-robot.json']);
});
