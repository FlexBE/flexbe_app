IO.PackageParser = new (function() {
	var that = this;

	var fs = require('fs');
	var path = require('path');
	var { exec } = require('child_process')


	var dom_parser = new DOMParser();
	var watched_states = {};

	this.discover = function(pkg_cache, callback) {
		var add_states = [];
		var add_behaviors = [];

		ROS.getPackageList((pkg_list) => {
			// remove missing packages from cache
			pkg_cache = pkg_cache.filter(cached => pkg_list.findElement(entry => cached.name == entry.name) != undefined);
			// remove cached packages from detected ones
			pkg_list = pkg_list.filter(entry => pkg_cache.findElement(cached => cached.name == entry.name) == undefined);
			// all remaining packages are new and should be parsed
			var pkg_count = pkg_list.length;
			if (pkg_count > 0) {
				T.logInfo("Checking "+pkg_count+" ROS packages for states and behaviors  ("+pkg_cache.length+" in cache)...");
			} else {
				T.logInfo("No new ROS packages detected ("+pkg_cache.length+" in cache).");
			}
			pkg_cache = pkg_cache.concat(pkg_list);

			var processEntry = function(idx) {
				var entry = pkg_list[idx];
				if (idx >= pkg_count) {
					callback(pkg_cache, add_states, add_behaviors);
				} else {
					checkForRelevance(entry['path'], entry['name'], (has_states, has_behaviors) => {
						if (has_states || has_behaviors) {
							var add_package = function(python_path) {
								if (python_path != undefined) {
									entry['python_path'] = python_path;
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

	var checkForRelevance = function(pkg_path, pkg_name, callback) {
		var package_xml_path = path.join(pkg_path, 'share', pkg_name, 'package.xml');

		try {
			if (fs.existsSync(package_xml_path)) {
				var data = fs.readFileSync(package_xml_path);
				var xml = dom_parser.parseFromString(String(data), "text/xml");
				var pkg_xml = xml.getElementsByTagName("package")[0];
				var pkg_export = pkg_xml.getElementsByTagName("export")[0];
				var hasStates = pkg_export && pkg_export.getElementsByTagName("flexbe_states").length > 0;
				var hasBehaviors = pkg_export && pkg_export.getElementsByTagName("flexbe_behaviors").length > 0;
				callback(hasStates, hasBehaviors);
			} else {
				callback(undefined, undefined);
			}
		} catch (e) {
			T.logWarn("Skipping package "+pkg_path+" due to a malformed package.xml");
			callback(undefined, undefined);
		}
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
						IO.StateParser.parseState(content, imports, state_def => {
							if (state_def != undefined) {
								state_def.setFilePath(entry);
								WS.Statelib.updateDef(state_def);
								T.logInfo("Updating changed definition for state: " + state_def.getStateType());
								var update_states = Behavior.getStatemachine().traverseStates(function(state) {
									return state.getStateType() == state_def.getStateType();
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
					});
				}
			}
		);
	}

	this.parseStates = function(pkg, progress_cb, done_cb) {
		parseStateFolder(pkg['python_path'], undefined, progress_cb, done_cb);
	}

	var parseStateFolder = function(folder, import_path, progress_cb, done_cb) {
		var state_defs = [];
		IO.Filesystem.checkFileExists(folder, "__init__.py", function(exists) {
			if (exists) {
				import_path = import_path || path.dirname(folder);
			}
			IO.Filesystem.getFolderContent(folder, function(files) {
				files = files.sort();
				var processEntry = function(idx) {
					if (idx >= files.length) {
						progress_cb(1);
						done_cb(state_defs);
						return;
					} else {
						progress_cb(idx / files.length);
					}
					var entry = files[idx];
					if(IO.Filesystem.isFolder(entry)) {
						parseStateFolder(entry, import_path, progress_cb, new_state_defs => {
							state_defs = state_defs.concat(new_state_defs);
							processEntry(idx + 1);
						});
					} else if (import_path != undefined) {
						if (path.extname(entry) == ".py" && path.basename(entry) != "__init__.py") {
							IO.Filesystem.readFile(entry, (content) => {
								try {
									var imports = entry.replace(import_path+"/", "").replace(/.py$/i, "").replace(/[\/]/g, ".");
									IO.StateParser.parseState(content, imports, state_def => {
										try {
											if (state_def != undefined) {
												state_def.setFilePath(entry);
												if (WS.Statelib.getFromLib(state_def.getStateType())) {
													WS.Statelib.updateDef(state_def);
												} else {
													WS.Statelib.addToLib(state_def);
												}
												state_defs.push(state_def);
												watchStateFolder(folder, import_path);
											}
										} catch (error) {
											console.error(error);
										}
										processEntry(idx + 1);
									});
								} catch (error) {
									console.error(error);
									processEntry(idx + 1);
								}
							});
						} else {
							processEntry(idx + 1);
						}
					} else {
						processEntry(idx + 1);
					}
				};
				processEntry(0);
			});
		});
	}

	this.parseBehaviors = function(pkg, progress_cb, done_cb) {
		parseBehaviorFolder(pkg['path'], pkg['name'], pkg['python_path'], progress_cb, done_cb);
	}

	var parseBehaviorFolder = function(folder, pkg_name, python_path, progress_cb, done_cb) {
		var behavior_defs = [];
		IO.Filesystem.getFolderContent(folder, function(files) {
			files = files.sort();
			var processEntry = function(idx) {
				if (idx >= files.length) {
					progress_cb(1);
					done_cb(behavior_defs);
					return;
				} else {
					progress_cb(idx / files.length);
				}
				var entry = files[idx];
				if(IO.Filesystem.isFolder(entry)) {
					parseBehaviorFolder(entry, pkg_name, python_path, progress_cb, new_behavior_defs => {
						behavior_defs = behavior_defs.concat(new_behavior_defs);
						processEntry(idx + 1);
					});
				} else {
					if (path.extname(entry) == ".xml" && path.basename(entry)[0] != '#') {
						IO.Filesystem.readFile(entry, (content) => {
							var manifest = IO.ManifestParser.parseManifest(content, entry, python_path);
							if (manifest != undefined) {
								if (manifest.rosnode_name != pkg_name) {
									T.logWarn("Ignoring behavior " + manifest.name + ": Manifest and code need to be in the same ROS package.");
									processEntry(idx + 1);
									return;
								}
								IO.BehaviorLoader.loadBehaviorInterface(manifest, function(ifc) {
									var behavior_def = new WS.BehaviorStateDefinition(manifest, ifc.smi_outcomes, ifc.smi_input, ifc.smi_output);
									WS.Behaviorlib.addToLib(behavior_def);
									behavior_defs.push(behavior_def);
									processEntry(idx + 1);
								});
							} else {
								processEntry(idx + 1);
							}
						});
					} else {
						processEntry(idx + 1);
					}
				}
			};
			processEntry(0);
		});
	}

}) ();
