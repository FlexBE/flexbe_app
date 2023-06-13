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

	var get_package_paths = `
import json
import os
import pathlib
import subprocess
import sys

from ament_index_python.packages import get_packages_with_prefixes, get_package_share_directory
import xml.etree.ElementTree as ET

def check_for_relevance(pkg_name, pkg_share_path):

    package_xml_path = os.path.join(pkg_share_path, "share", pkg_name, "package.xml")

    if os.path.exists(package_xml_path):
        try:
            xml_tree = ET.parse(package_xml_path)
            root = xml_tree.getroot()
            pkg_export = root.find("export")
            if pkg_export:
                has_states = pkg_export.find("flexbe_states") is not None
                has_behaviors = pkg_export.find("flexbe_behaviors") is not None
                return has_states, has_behaviors
        except Exception as exc:
            pass

    return False, False

def find_flexbe_packages():

    pkg_list = get_packages_with_prefixes()

    flexbe_packages = []

    for pkg_name, pkg_path in pkg_list.items():
        has_states, has_behaviors = check_for_relevance(pkg_name, pkg_path)

        if has_states or has_behaviors:
            package = {"name": pkg_name, "path": pkg_path, "python_path": None}
            flexbe_packages.append(package)

    return flexbe_packages

if __name__ == "__main__":
    flexbe_packages = find_flexbe_packages()
    #-----------------------------------------------------------
    dump_path = os.path.join(str(pathlib.Path.home()), ".ros", "flexbe_package_list.txt")
    with open(dump_path, "wt") as dump:
            dump.write(json.dumps(flexbe_packages, sort_keys=True, indent=2))
    #-----------------------------------------------------------

    sys.stdout.write(json.dumps(flexbe_packages))
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
			let package_data = '';
			let packages = []

			T.logInfo("  Request FlexBE compatible package list from ROS ...");
			get_pkg = spawn(python, ['-c', get_package_paths])

			get_pkg.stdout.on('data', data => {
				package_data += data;
			});

			get_pkg.stderr.on('data', data => {
				T.logError(data);
			});
			get_pkg.on('close', (code) => {
				T.logInfo(" Processing FlexBE compatible package list ...");
				try {
					package_cache = JSON.parse(package_data);
				} catch (err) {
					T.logError(err.toString());
					T.logError(" ros.js:: JSON error -->\n" + data);
					throw err;
				}

				T.logInfo(" Found " + package_cache.length + " FlexBE compatible packages.");
				for (let i = 0; i < package_cache.length; i++) {
					package_cache[i].python_path = undefined
					T.logInfo("  " + package_cache[i].name + " -->" + package_cache[i].path)
				}
				callback(package_cache.clone())
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
		var temp_package_path = undefined;
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
