ROS.Publisher = function(topic, msg_type, latched=false) {
	var that = this;
	var os = require('os');
	var sys = require('sys');
	var spawn = require('child_process').spawn;

	var LATCHED = "latched";
////////////////////////////////
// BEGIN Python implementation
	var impl = `
import rospy
import sys
import importlib
import json
import genpy
import yaml

topic = sys.argv[1]
msg_def = sys.argv[2].split('/')
msg_pkg = msg_def[0]
msg_name = msg_def[1]
latched = sys.argv[3] == "`+LATCHED+`" if len(sys.argv) > 3 else False

rospy.init_node('flexbe_app_pub_%s' % topic.replace('/', '_'))

msg_module = importlib.import_module('%s.msg' % msg_pkg)
msg_class = getattr(msg_module, msg_name)

pub = rospy.Publisher(topic, msg_class, queue_size=10, latch=latched)

while not rospy.is_shutdown():
	json_str = sys.stdin.readline()
	try:
		msg_dict = json.loads(json_str)
		msg = msg_class()
		genpy.message.fill_message_args(msg, [msg_dict])
		pub.publish(msg)
	except Exception as e:
		if json_str != '':
			sys.stderr.write("ignoring input %s> %s" % (json_str, str(e)))
			sys.stderr.flush();
	`;
// END Python implementation
//////////////////////////////

	var pub = spawn('python', ['-c', impl, topic, msg_type, latched? LATCHED : "_"]);

	pub.stdout.on('data', (data) => {
		T.logInfo('[PUB:'+topic+'] ' + data);
	});

	pub.stderr.on('data', (data) => {
		T.logWarn('[PUB:'+topic+'] ' + data);
	});

	pub.on('close', (code) => {
		console.log('[PUB:'+topic+'] EXIT');
	});

	that.publish = function(data) {
		if (pub == undefined) {
			T.logError('[PUB:'+topic+'] cannot publish: already closed');
			return;
		}
		data = data || {};
		pub.stdin.write(JSON.stringify(data) + os.EOL);
	}

	that.close = function() {
		if (pub == undefined) return;
		pub.stdin.end();
		pub.kill('SIGKILL');
		pub = undefined;
	}

};