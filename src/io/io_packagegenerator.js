IO.PackageGenerator = new (function() {
	var that = this;

	var path = require('path');
	var fs = require('fs');

	var generateSetupPy = function(pkg_name) {
		return "#!/usr/bin/env python\n" +
			"\n" +
			"from distutils.core import setup\n" +
			"from catkin_pkg.python_setup import generate_distutils_setup\n" +
			"\n" +
			"d = generate_distutils_setup(\n" +
			"    packages = ['" + pkg_name + "'],\n" +
			"    package_dir = {'': 'src'}\n" +
			")\n" +
			"\n" +
			"setup(**d)";
	}

	var convertBehaviors = function(folder, pkg_name, pkg_path, callback, timer) {
		if (timer == undefined) timer = setTimeout(callback, 500);
		var ignore_path = path.join(pkg_path, "manifest");
		IO.Filesystem.getFolderContent(folder, function(files) {
			files.sort().forEach(function(entry, i) {
				if(IO.Filesystem.isFolder(entry)) {
					convertBehaviors(entry, pkg_name, pkg_path, callback, timer);
				} else {
					if (path.extname(entry) != ".xml" || path.basename(entry)[0] == '#' || path.dirname(entry) == ignore_path) return;
					IO.Filesystem.readFile(entry, (content) => {
						IO.ManifestParser.parseManifest(content, entry, (manifest) => {
							if (manifest != undefined) {// && manifest.rosnode_name != pkg_name) {
								ROS.getPackagePath(manifest.rosnode_name, (be_pkg_path) => {
									var src_file_path = path.join(be_pkg_path, 'src', manifest.rosnode_name, manifest.codefile_name);
									var dst_file_path = path.join(pkg_path, 'src', pkg_name, manifest.codefile_name);
									var src_file = fs.readFileSync(src_file_path);
									fs.writeFileSync(dst_file_path, src_file);
									var dst_manifest_path = path.join(ignore_path, manifest["name"]toLowerCase().replace(/ /g, "_") + '.py');
									var new_content = content.replace('package_path="'+manifest.rosnode_name+'.', 'package_path="'+pkg_name+'.');
									fs.writeFileSync(dst_manifest_path, new_content);
									clearTimeout(timer);
									timer = setTimeout(callback, 500);
								});
							}
						})
					});
				}
			});
		});
	}

	this.initializeBehaviorPackage = function(pkg_name, convert_behaviors, callback) {
		ROS.getPackagePath(pkg_name, (pkg_path) => {
			IO.Filesystem.getFileContent(pkg_path, "package.xml", (content) => {
				if (content.indexOf("</export>") != -1) {
					content = content.replace("</export>", "  <flexbe_behaviors />\n  </export>");
				} else {
					content = content.replace("</package>", "  <export>\n    <flexbe_behaviors />\n  </export>\n\n</package>");
				}
				IO.Filesystem.createFile(pkg_path, "package.xml", content, () => {
					IO.Filesystem.getFileContent(pkg_path, "CMakeLists.txt", (content) => {
						if (content.indexOf("#catkin_python_setup()") != -1) {
							content = content.replace("#catkin_python_setup()", "catkin_python_setup()");
						} else if (content.indexOf("catkin_python_setup()") == -1) {
							content += "\ncatkin_python_setup()";
						}
						IO.Filesystem.createFile(pkg_path, "CMakeLists.txt", content, () => {
							var src_folder_path = path.join(pkg_path, "src");
							var module_folder_path = path.join(pkg_path, "src", pkg_name);
							IO.Filesystem.createFolder(pkg_path, "manifest", () => {
								IO.Filesystem.createFile(pkg_path, "setup.py", generateSetupPy(pkg_name), () => {
									IO.Filesystem.createFolder(pkg_path, "src", () => {
										IO.Filesystem.createFolder(src_folder_path, pkg_name, () => {
											IO.Filesystem.createFile(module_folder_path, "__init__.py", "", () => {
												if (convert_behaviors) {
													convertBehaviors(pkg_path, pkg_name, pkg_path, callback);
												}
											});
										});
									});
								});
							});
						});
					});
				});
			});
		});
	}

}) ();
