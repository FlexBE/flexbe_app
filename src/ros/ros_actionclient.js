ROS.ActionClient = function(topic, action_type) {
	var that = this;
	var os = require('os');
	var sys = require('sys');
	var spawn = require('child_process').spawn;
	var python = 'python' + (process.env.ROS_PYTHON_VERSION != undefined? process.env.ROS_PYTHON_VERSION : '');

	var feedback_cb = undefined;
	var result_cb = undefined;
	var timer = undefined;

	const TYPE_FEEDBACK = "feedback";
	const TYPE_RESULT = "result";
////////////////////////////////
// BEGIN Python implementation
	var impl = `
import rclpy
from rclpy.action import ActionClient
import sys
import importlib
import json
import yaml
import rosidl_runtime_py.convert
import rosidl_runtime_py.set_message

def feedback_cb(msg):
	comm = dict()
	comm['msg'] = yaml.load(rosidl_runtime_py.convert.message_to_yaml(msg), Loader=yaml.SafeLoader)
	comm['type'] = "`+TYPE_FEEDBACK+`"
	sys.stdout.write(json.dumps(comm))
	sys.stdout.flush()

def result_cb(future):
	goal_handle = future.result()
  self._get_result_future = goal_handle.get_result_async()
  self._get_result_future.add_done_callback(self.get_result)

def get_result(future):
	msg = future.result().result
	comm = dict()
	comm['msg'] = yaml.load(rosidl_runtime_py.convert.message_to_yaml(msg), Loader=yaml.SafeLoader)
	# comm['state'] = str(state)
	comm['type'] = "`+TYPE_RESULT+`"
	sys.stdout.write(json.dumps(comm))
	sys.stdout.flush()

topic = sys.argv[1]
msg_def = sys.argv[2].split('/')
msg_pkg = msg_def[0]
msg_action_name = msg_def[1] + 'Action'
msg_goal_name = msg_def[1] + 'Goal'

rclpy.init()
node = rclpy.create_node('flexbe_app_act_%s' % topic.replace('/', '_'))

msg_module = importlib.import_module('%s.msg' % msg_pkg)
msg_action_class = getattr(msg_module, msg_action_name)
msg_goal_class = getattr(msg_module, msg_goal_name)

client = ActionClient(node, msg_action_class, topic)

while rclpy.ok():
	json_str = sys.stdin.readline()
	try:
		msg_dict = json.loads(json_str)
		msg = msg_goal_class()
		rosidl_runtime_py.set_message.set_message_fields(msg, msg_dict)

		client.wait_for_server()
		future = client.send_goal_async(msg, feedback_callback=feedback_cb)
		future.add_done_callback(result_cb)
	except Exception as e:
		sys.stderr.write("ignoring goal %s> %s" % (json_str, str(e)))
		sys.stderr.flush();
	`;
// END Python implementation
//////////////////////////////

	var client = spawn(python, ['-c', impl, topic, action_type]);

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
