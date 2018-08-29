IO.PackageParser = new (function() {
	var that = this;

	var fs = require('fs');
	var path = require('path');

	var dom_parser = new DOMParser();
	var watched_states = {};

	this.discover = function(pkg_cache, callback) {
		var add_states = [];
		var add_behaviors = [];

		ROS.getPackageList((pkg_list) => {
			pkg_cache.forEach(entry => { pkg_list.remove(entry); });
			var pkg_count = pkg_list.length;
			if (pkg_count > 0) {
				//T.logInfo("Checking "+pkg_count+" newly detected ROS packages for states and behaviors...");
				T.logInfo("Checking "+pkg_count+" ROS packages for states and behaviors...");
			} else {
				T.logInfo("No new ROS packages detected ("+pkg_cache.length+" in cache).");
			}
			var processEntry = function(idx) {
				var entry = pkg_list[idx];
				if (idx >= pkg_count) {
					callback(pkg_list, add_states, add_behaviors);
				} else {
					checkForRelevance(entry['path'], (has_states, has_behaviors) => {
						if (has_states || has_behaviors) {
							var add_package = function(python_path) {
								if (python_path != undefined) {
									entry['python_path'] = python_path;
									console.log(entry);
									if (has_states) add_states.push(entry);
									if (has_behaviors) add_behaviors.push(entry);
								}
								processEntry(idx+1);
							}
							python_path = entry['python_path'];
							if (python_path == undefined) {
								ROS.getPackagePythonPath(entry['name'], add_package);
							} else {
								add_package(python_path);
							}
						} else {
							processEntry(idx+1);
						}
					});
				}
			};
			processEntry(0);
		});
	}

	this.stopWatching = function() {
		for (var state in watched_states) {
			watched_states[state].close();
		}
	}

	var checkForRelevance = function(pkg_path, callback) {
		var data = fs.readFileSync(path.join(pkg_path, 'package.xml'));
		var xml = dom_parser.parseFromString(String(data), "text/xml");
		var pkg_xml = xml.getElementsByTagName("package")[0];
		var pkg_export = pkg_xml.getElementsByTagName("export")[0];
		var hasStates = pkg_export && pkg_export.getElementsByTagName("flexbe_states").length > 0;
		var hasBehaviors = pkg_export && pkg_export.getElementsByTagName("flexbe_behaviors").length > 0;
		callback(hasStates, hasBehaviors);
	}

	var watchStateFolder = function(folder_path, import_path) {
		if (watched_states[folder_path] != undefined) return;

		watched_states[folder_path] = fs.watch(folder_path,
			{persistent: false},
			(eventType, filename) => {
				if(RC.Controller.isReadonly()) {
					T.logWarn("A state definition source file changed while in read-only mode, ignoring the change for now!");
					return;
				}
				if (filename.endsWith(".py")) {
					var entry = path.join(folder_path, filename);
					IO.Filesystem.readFile(entry, (content) => {
						var imports = entry.replace(import_path+"/", "").replace(/.py$/i, "").replace(/[\/]/g, ".");
						var state_def = IO.StateParser.parseState(content, imports);
						if (state_def != undefined) {
							state_def.setFilePath(entry);
							WS.Statelib.updateDef(state_def);
							T.logInfo("Updating changed definition for state: " + state_def.getStateClass());
							var update_states = Behavior.getStatemachine().traverseStates(function(state) {
								return state.getStateClass() == state_def.getStateClass();
							});
							update_states.forEach(function (state) {
								state.updateStateDefinition(state_def);
								if (state.getContainer() == UI.Statemachine.getDisplayedSM() 
									&& UI.Panels.StateProperties.isCurrentState(state)) {
									UI.Panels.StateProperties.hide();
									UI.Panels.StateProperties.displayStateProperties(state);
								}
							});
							if (UI.Menu.isPageStatemachine()) {
								UI.Statemachine.refreshView();
							}
							RC.Controller.signalChanged();
						}
					});
				}
			}
		);
	}

	this.parseStateFolder = function(folder, import_path) {
		IO.Filesystem.checkFileExists(folder, "__init__.py", function(exists) {
			if (exists) {
				import_path = import_path || path.dirname(folder);
			}
			IO.Filesystem.getFolderContent(folder, function(files) {
				files.sort().forEach(function(entry, i) {
					if(IO.Filesystem.isFolder(entry)) {
						that.parseStateFolder(entry, import_path);
					} else if (import_path != undefined) {
						if (path.extname(entry) != ".py") return;
						IO.Filesystem.readFile(entry, (content) => {
							var imports = entry.replace(import_path+"/", "").replace(/.py$/i, "").replace(/[\/]/g, ".");
							var state_def = IO.StateParser.parseState(content, imports);
							if (state_def != undefined) {
								state_def.setFilePath(entry);
								WS.Statelib.addToLib(state_def);
								watchStateFolder(folder, import_path);
							}
						});
					}
				});
			});
		});
	}

	this.parseBehaviorFolder = function(folder, pkg_name, python_path) {
		IO.Filesystem.getFolderContent(folder, function(files) {
			files.sort().forEach(function(entry, i) {
				if(IO.Filesystem.isFolder(entry)) {
					that.parseBehaviorFolder(entry, pkg_name, python_path);
				} else {
					if (path.extname(entry) != ".xml" || path.basename(entry)[0] == '#') return;
					IO.Filesystem.readFile(entry, (content) => {
						var manifest = IO.ManifestParser.parseManifest(content, entry, python_path);
						if (manifest != undefined) {
							if (manifest.rosnode_name != pkg_name) {
								T.logWarn("Ignoring behavior " + manifest.name + ": Manifest and code need to be in the same ROS package.");
								return;
							}
							IO.BehaviorLoader.loadBehaviorInterface(manifest, function(ifc) {
								WS.Behaviorlib.addToLib(new WS.BehaviorStateDefinition(manifest, ifc.smi_outcomes, ifc.smi_input, ifc.smi_output));
							});
						}
					});
				}
			});
		});
	}

}) ();