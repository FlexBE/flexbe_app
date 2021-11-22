ROS.Subscriber = function(topic, msg_type, callback) {
	var that = this;
	var sys = require('sys');
	var spawn = require('child_process').spawn;
	var python = 'python' + (process.env.ROS_PYTHON_VERSION != undefined? process.env.ROS_PYTHON_VERSION : '');

////////////////////////////////
// BEGIN Python implementation
	var impl = `
import rclpy
import sys
import importlib
import json
import rosidl_runtime_py.convert
import yaml

def callback(msg):
	sys.stdout.write(json.dumps(yaml.load(rosidl_runtime_py.convert.message_to_yaml(msg))))
	sys.stdout.flush()

topic = sys.argv[1]
msg_def = sys.argv[2].split('/')
msg_pkg = msg_def[0]
msg_name = msg_def[1]

rclpy.init()
node = rclpy.create_node('flexbe_app_sub_%s' % topic.replace('/', '_'))

msg_module = importlib.import_module('%s.msg' % msg_pkg)
msg_class = getattr(msg_module, msg_name)

sub = node.create_subscription(msg_class, topic, callback, 10)

rclpy.spin(node)
	`;
// END Python implementation
//////////////////////////////

	var sub = spawn(python, ['-c', impl, topic, msg_type]);

	var buffer = "";

	sub.stdout.on('data', (data) => {
		buffer += data;
		var try_parse = true;
		while (try_parse) {
			try_parse = false;
			try {
				var [obj, idx] = json_parse_raw(buffer);
				if (obj == null) obj = undefined;
				if (idx != 0) {
					var exec_cb = function(o) { process.nextTick(() => { callback(o); }); };
					buffer = buffer.slice(idx);
					try_parse = true;
					exec_cb(obj);
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

	sub.stderr.on('data', (data) => {
		T.logWarn('[SUB:'+topic+'] ' + data);
	});

	sub.on('close', (code) => {
		console.log('[SUB:'+topic+'] EXIT');
	});

	that.close = function() {
		if (sub == undefined) return;
		sub.kill('SIGKILL');
		sub = undefined;
	}

};
