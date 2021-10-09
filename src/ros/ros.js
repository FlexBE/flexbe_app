ROS = new (function() {
	var that = this;

	var os = require('os');
	var sys = require('sys');
	var {spawn, exec, execSync} = require('child_process')
	// var spawn = require('child_process').spawn;
	var python = 'python' + (process.env.ROS_PYTHON_VERSION != undefined? process.env.ROS_PYTHON_VERSION : '');

////////////////////////////////
// BEGIN Python implementation
	var init_impl = `
import rclpy
import sys

rclpy.init()
node = rclpy.create_node('flexbe_app')

sys.stdout.flush()
sys.stdout.write(':'+node.get_namespace()+':connected')
sys.stdout.flush()

rclpy.spin(node)
	`;

var new_line = "\n"
	var get_package_paths = `
import subprocess
import os
import sys
import json
from ament_index_python.packages import get_package_share_directory

pkg_list = subprocess.check_output(["ros2", "pkg", "list"]).decode('utf-8')
packages = []

pkg_list = pkg_list.split(${new_line})

for pkg in pkg_list:
	path = get_package_share_directory(pkg)
	if "/opt/ros/foxy" not in path:
		if "share" in path:
			path = path.split("/share")[0]

		package = {'name': pkg, 'path': path, 'python_path': None}
		packages.append(package)

sys.stdout.write(json.dumps(packages))
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

		ros_proc = exec(python + " -c " + init_impl, function(err, stdout, stderr) {
		  if (err || stderr) {
				T.logError("ROS connection error: "+data);
				ros_proc.kill('SIGKILL');
				callback(undefined);
		  }
		  console.log(stdout);
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
			var get_pkg = undefined;
			let packages = []

			get_pkg = spawn(python, ['-c', get_package_paths])

			get_pkg.stdout.on('data', data => {
				package_cache = JSON.parse(data);

				for (let i = 0; i < package_cache.length; i++) {
					package_cache[i].python_path = undefined
				}
				callback(package_cache.clone())
			});

			get_pkg.stderr.on('data', data => {
				T.logError(data);
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

	that.getParam = function(name, callback) {
		var proc = spawn('ros2', ['param', 'get', name]);
		proc.stdout.on('data', data => {
			proc.kill('SIGKILL');
			if (String(data).startsWith('ERROR')) {
				callback(undefined);
			} else {
				callback(JSON.parse(data));
			}
		});
	}

}) ();
