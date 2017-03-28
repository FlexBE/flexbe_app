ROS.ActionClient = function(topic, action_type) {
	var that = this;
	var os = require('os');
	var sys = require('sys');
	var spawn = require('child_process').spawn;

	var feedback_cb = undefined;
	var result_cb = undefined;
	var timer = undefined;

	const TYPE_FEEDBACK = "feedback";
	const TYPE_RESULT = "result";
////////////////////////////////
// BEGIN Python implementation
	var impl = `
import rospy
import sys
import importlib
import actionlib
import json
import genpy
import yaml

def feedback_cb(msg):
	comm = dict()
	comm['msg'] = yaml.load(genpy.message.strify_message(msg))
	comm['type'] = "`+TYPE_FEEDBACK+`"
	sys.stdout.write(json.dumps(comm))
	sys.stdout.flush()

def result_cb(state, msg):
	comm = dict()
	comm['msg'] = yaml.load(genpy.message.strify_message(msg))
	comm['state'] = str(state)
	comm['type'] = "`+TYPE_RESULT+`"
	sys.stdout.write(json.dumps(comm))
	sys.stdout.flush()

topic = sys.argv[1]
msg_def = sys.argv[2].split('/')
msg_pkg = msg_def[0]
msg_action_name = msg_def[1] + 'Action'
msg_goal_name = msg_def[1] + 'Goal'

rospy.init_node('flexbe_app_act_%s' % topic.replace('/', '_'))

msg_module = importlib.import_module('%s.msg' % msg_pkg)
msg_action_class = getattr(msg_module, msg_action_name)
msg_goal_class = getattr(msg_module, msg_goal_name)

client = actionlib.SimpleActionClient(topic, msg_action_class)

while not rospy.is_shutdown():
	json_str = sys.stdin.readline()
	try:
		msg_dict = json.loads(json_str)
		msg = msg_goal_class()
		genpy.message.fill_message_args(msg, [msg_dict])
		client.send_goal(msg, done_cb=result_cb, feedback_cb=feedback_cb)
	except Exception as e:
		sys.stderr.write("ignoring goal %s> %s" % (json_str, str(e)))
		sys.stderr.flush();
	`;
// END Python implementation
//////////////////////////////

	var client = spawn('python', ['-c', impl, topic, action_type]);

	var buffer = "";

	client.stdout.on('data', (data) => {
		buffer += data;
		var try_parse = true;
		while (try_parse) {
			try_parse = false;
			try {
				var [obj, idx] = json_parse_raw(buffer);
				if (obj == null) obj = undefined;
				if (idx != 0) {
					buffer = buffer.slice(idx);
					try_parse = true;
					if (obj['type'] == TYPE_FEEDBACK && feedback_cb != undefined) {
						var exec_cb = function(o) { process.nextTick(() => { feedback_cb(o); }); };
						exec_cb(obj['msg']);
					}
					if (obj['type'] == TYPE_RESULT) {
						if (timer != undefined) {
							clearTimeout(timer);
							timer = undefined;
						}
						callback = result_cb;
						feedback_cb = undefined;
						result_cb = undefined;
						if (callback != undefined) {
							var exec_cb = function(o,s) { process.nextTick(() => { callback(o,s); }); };
							exec_cb(obj['msg'], obj['state']);
						}
					}
				}
			} catch (err) {
				try_parse = false;
				console.log('[SUB:'+topic+'] Error:');
				console.log(err);
				if (err.hasOwnProperty('name') && err.name == "SyntaxError" && err.hasOwnProperty('at')) {
					buffer.slice(err.at);
					try_parse = true;
				}
			}
		}
	});

	client.stderr.on('data', (data) => {
		T.logWarn('[CLIENT:'+topic+'] ' + data);
		if (timer != undefined) {
			clearTimeout(timer);
			timer = undefined;
		}
		callback = result_cb;
		feedback_cb = undefined;
		result_cb = undefined;
		if (callback != undefined) {
			callback(undefined, undefined);
		}
	});

	client.on('close', (code) => {
		T.logInfo('[CLIENT:'+topic+'] EXIT');
	});

	that.send_goal = function(data, _result_cb, _feedback_cb, timeout, timeout_cb) {
		if (client == undefined) {
			T.logError('[CLIENT:'+topic+'] cannot send goal: already closed');
			return;
		}
		if (result_cb != undefined) {
			result_cb(undefined, undefined);
		}
		result_cb = _result_cb;
		feedback_cb = _feedback_cb;
		client.stdin.write(JSON.stringify(data) + os.EOL);
		if (timeout != undefined) {
			if (timer != undefined) clearTimeout(timer);
			timer = setTimeout(timeout_cb, timeout);
		}
		
	}

	that.close = function() {
		if (client == undefined) return;
		client.stdin.end();
		client.kill('SIGKILL');
		client = undefined;
	}

};