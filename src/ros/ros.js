ROS = new (function() {
	var that = this;

	var os = require('os');
	var sys = require('sys');
	var spawn = require('child_process').spawn;
	var python = 'python' + (process.env.ROS_PYTHON_VERSION != undefined? process.env.ROS_PYTHON_VERSION : '');

////////////////////////////////
// BEGIN Python implementation
	var init_impl = `
import rospy
import sys

rospy.init_node('flexbe_app')

sys.stdout.flush()
sys.stdout.write(':'+rospy.get_namespace()+':connected')
sys.stdout.flush()

rospy.spin()
	`;
// END Python implementation
//////////////////////////////
	
	var ros_proc = undefined;

	that.init = function(callback) {
		ros_proc = spawn(python, ['-c', init_impl]);
		ros_proc.stdout.on('data', data => {
			data = String(data);
			if (data.endsWith("connected")) {
				var data_segments = data.split(':');
				var ros_namespace = data_segments[data_segments.length-2];
				callback(ros_namespace);
			}
		});
		ros_proc.stderr.on('data', data => {
			T.logError("ROS connection error: "+data);
			ros_proc.kill('SIGKILL');
			callback(undefined);
		});
	}

	that.shutdown = function() {
		if (ros_proc != undefined) {
			ros_proc.kill('SIGKILL');
		}
	}

	var package_cache = undefined;
	that.getPackageList = function(callback) {
		if (package_cache == undefined) {
			var proc = spawn('rospack', ['list']);

			var pkg_data = '';
			proc.stdout.on('data', data => {
				pkg_data += data;
				
			});
			proc.on('close', (code) => {
				package_cache = pkg_data.split(os.EOL);
				if (package_cache.length > 0) package_cache = package_cache.slice(0,-1);
				for (var i=0; i<package_cache.length; i++) {
					package_cache[i] = package_cache[i].split(" ");
					package_cache[i] = {
						'name': package_cache[i][0],
						'path': package_cache[i][1],
						'python_path': undefined
					}
				}
				callback(package_cache.clone());
			});
		} else {
			process.nextTick(() => {
				callback(package_cache.clone());
			});
		}
	}

	that.getPackagePath = function(package_name, callback) {
		that.getPackageList((package_cache) => {
			var package_path = undefined;
			for (var i=0; i<package_cache.length; i++) {
				if (package_cache[i]['name'] == package_name) {
					package_path = package_cache[i]['path'];
					break;
				}
			}
			callback(package_path);
		});
	}

	that.getPackagePythonPath = function(package_name, callback) {
		var python_path = undefined;
		that.getPackageList((package_cache) => {
			for (var i=0; i<package_cache.length; i++) {
				if (package_cache[i]['name'] == package_name) {
					python_path = package_cache[i]['python_path'];
					break;
				}
			}
			if (python_path !== undefined) {
				process.nextTick(() => {
					callback(python_path);
				});
			} else {
				var proc = spawn(python, ['-c', `import importlib; print(importlib.import_module('` + package_name + `').__path__[-1])`]);
				var path_data = '';
				proc.stdout.on('data', data => {
					path_data += data;
				});
				proc.stderr.on('data', data => {
					console.log(package_name+" failed to import: "+data);
				});
				proc.on('close', (code) => {
					if (path_data != "") {
						python_path = path_data.replace(/\n/g, '');
						for (var i=0; i<package_cache.length; i++) {
							if (package_cache[i]['name'] == package_name) {
								package_cache[i]['python_path'] = python_path;
								break;
							}
						}
						callback(python_path);
					} else {
						callback(undefined);
					}
				});
			}
		});
	}

	// that.getParam = function(name, callback) {
	// 	var proc = spawn('rosparam', ['get', name]);
	// 	proc.stdout.on('data', data => {
	// 		proc.kill('SIGKILL');
	// 		if (String(data).startsWith('ERROR')) {
	// 			callback(undefined);
	// 		} else {
	// 			callback(JSON.parse(data));
	// 		}
	// 	});
	// }

}) ();