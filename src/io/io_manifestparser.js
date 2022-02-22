IO.ManifestParser = new (function() {
	var that = this;
	var spawn = require('child_process').spawn;
	var python = 'python' + (process.env.ROS_PYTHON_VERSION != undefined? process.env.ROS_PYTHON_VERSION : '');

	this.parseManifest = function(content, file_path, python_path, callback) {
		var file_path_parts = file_path.split("/")
		var python_file = file_path_parts[file_path_parts.length - 1].substring(0, file_path_parts[file_path_parts.length - 1].indexOf(".py"))

		var py_module = file_path_parts[file_path_parts.length - 3] + "." + file_path_parts[file_path_parts.length - 2] + "." + python_file
		var manifest_file = file_path_parts[file_path_parts.length - 1]
		var manifest_dict = manifest_file.substring(0, manifest_file.indexOf("_manifest"))

		var json_data = undefined
		var impl = `
import json
from ${py_module} import ${manifest_dict}
print(json.dumps(${manifest_dict}))`;
		// var impl = `from ` + py_module + ` import ` + manifest_dict + `\nprint(` + manifest_dict + `)`
		var manifest = undefined;
		var manifest_json = undefined;

		manifest = spawn(python, ['-c', impl]);
		manifest.stdout.on('data', data => {
			try {
				json_data = JSON.parse(data)

				var package_path_parts = json_data["executable"]["package_path"].split(".")
				var contains_elements = json_data["contained_behaviors"]

				var contains_list = [];
				for (var i = 0; i < contains_elements.length; i++) {
					contains_list.push(contains_elements[i]["name"]);
				}

				var manifest_json = {
					name: 			    json_data["name"],
					description: 	  json_data["description"],
					tags: 			    json_data["tagstring"],
					author: 		    json_data["author"],
					date: 			    json_data["date"],
					rosnode_name: 	package_path_parts[0],
					codefile_name: 	package_path_parts[1] + ".py",
					codefile_path: 	python_path,
					class_name: 	  json_data["executable"]["class"],
					params: 		    json_data["params"],
					contains: 		  contains_list,
					file_path: 		  file_path
				};

				callback(manifest_json)
			} catch (err) {
				try_parse = false;
				console.log('Manifest Parser Error:');
				console.log(err);
				if (err.hasOwnProperty('name') && err.name == "SyntaxError" && err.hasOwnProperty('at')) {
					buffer.slice(err.at);
					try_parse = true;
				}
			}
		});
		manifest.stderr.on('data', (data) => {
			T.logWarn("[Manifest parse] " + data);
		});
	}

}) ();
