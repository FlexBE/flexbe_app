ROS = new (function() {
	var that = this;

	var os = require('os');
	var sys = require('sys');
	var spawn = require('child_process').spawn;

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
		ros_proc = spawn('python', ['-c', init_impl]);
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
						'path': package_cache[i][1]
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

    var python_package_cache = {};
	that.getPackagePythonPath = function(package_name, callback) {
    	if (python_package_cache[package_name] !== undefined) {
            process.nextTick(() => {
                callback(python_package_cache[package_name]);
            });
    	} else {
            var proc = spawn('python', ['-c', `import importlib; print(importlib.import_module('` + package_name + `').__path__[-1])`]);

            var pkg_data = '';
            proc.stdout.on('data', data => {
                pkg_data += data;
            });
            proc.on('close', (code) => {
                lines = pkg_data.split(os.EOL);
                if (pkg_data.length > 0 && lines.length > 0) {
                    python_package_cache[package_name] = lines[0];
                }
                callback(python_package_cache[package_name]);
            });
        }
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