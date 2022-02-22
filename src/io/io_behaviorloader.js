IO.BehaviorLoader = new (function() {
	var that = this;

	var path = require('path');


	var parseCode = function(file_content, manifest_data, callback) {
		callback = callback || console.error;
		var parsingResult;
		try {
			parsingResult = IO.CodeParser.parseCode(file_content);
		} catch (err) {
			var error_string = "Code parsing failed: " + err;
			T.logError(error_string);
			callback(error_string);
			return;
		}
		try {
			applyParsingResult(parsingResult, manifest_data);
			T.logInfo("Behavior " + parsingResult.behavior_name + " loaded.");
		} catch (err) {
			var error_string = "Code parsing failed: " + err;
			T.logError(error_string);
			callback(error_string);
			return;
		}

		var error_string = Checking.checkBehavior();
		if (error_string != undefined) {
			T.logError("The loaded behavior contains errors! Please fix and save:");
			T.logError(error_string);
			RC.Controller.signalChanged();
		}
		callback(error_string);
	}

	var applyParsingResult = function(result, manifest) {
		IO.ModelGenerator.generateBehaviorAttributes(result, manifest);

		T.logInfo("Building behavior state machine...");
		var sm = IO.ModelGenerator.buildStateMachine("", result.root_sm_name, result.sm_defs, result.sm_states);
		Behavior.setStatemachine(sm);
		UI.Statemachine.resetStatemachine();
		T.logInfo("Behavior state machine built.");

		ActivityTracer.resetActivities();

		ROS.getPackagePath(manifest.rosnode_name, (package_path) => {
			ROS.getPackagePythonPath(manifest.rosnode_name, (python_path) => {
				if (!python_path.startsWith(package_path)) {
					Behavior.setReadonly(true);
				}
				UI.Statemachine.refreshView();
			});
		});
	}

	var resetEditor = function() {
		Behavior.resetBehavior();
		UI.Dashboard.resetAllFields();
		UI.Statemachine.resetStatemachine();

		// make sure a new behavior always starts at the dashboard
		UI.Menu.toDashboardClicked();
		UI.Panels.setActivePanel(UI.Panels.NO_PANEL);
	}

	this.loadBehavior = function(manifest, callback) {
		T.clearLog();
		UI.Panels.Terminal.show();

		resetEditor();

		var file_path = path.join(manifest.codefile_path, manifest.codefile_name);
		IO.Filesystem.readFile(file_path, (content) => {
			T.logInfo("Parsing sourcecode...");
			parseCode(content, manifest, callback);
		});
	}

	this.loadBehaviorInterface = function(manifest, callback) {
		var file_path = path.join(manifest.codefile_path, manifest.codefile_name);
		IO.Filesystem.readFile(file_path, (content) => {
			try {
				var parsingResult = IO.CodeParser.parseSMInterface(content);
				callback(parsingResult);
			} catch (err) {
				T.logError("Failed to parse behavior interface of " + manifest.name + ": " + err);
				return;
			}
		});
	}

	this.updateManualSections = function(callback) {
		var names = Behavior.createNames();
		var package_name = names.rosnode_name;
		ROS.getPackagePythonPath(package_name, (folder_path) => {
			if (folder_path == undefined) {
				return;
			}
			var file_path = path.join(folder_path, names.file_name);
			IO.Filesystem.checkFileExists(folder_path, names.file_name, (exists) => {
				if (exists) {
					IO.Filesystem.readFile(file_path, (content) => {
						var extract_result = IO.CodeParser.extractManual(content);
						Behavior.setManualCodeImport(extract_result.manual_import);
						Behavior.setManualCodeInit(extract_result.manual_init);
						Behavior.setManualCodeCreate(extract_result.manual_create);
						Behavior.setManualCodeFunc(extract_result.manual_func);
						callback();
					});
				} else {
					process.nextTick(() => {
						callback();
					});
				}
			});
		});
	}

	this.parseBehaviorSM = function(manifest, callback) {
		var file_path = path.join(manifest.codefile_path, manifest.codefile_name);
		IO.Filesystem.readFile(file_path, (content) => {
			console.log("Preparing sourcecode of behavior " + manifest.name + "...");
			try {
				parsingResult = IO.CodeParser.parseCode(content);
			} catch (err) {
				console.log("Code parsing failed: " + err);
				return;
			}
			callback({
				container_name: "",
				container_sm_var_name: parsingResult.root_sm_name,
				sm_defs: parsingResult.sm_defs,
				sm_states: parsingResult.sm_states,
				default_userdata: parsingResult.default_userdata
			});
		});
	}

	this.loadBehaviorDependencies = function(manifest, ignore_list) {
		manifest.contains.forEach(function(be_name) {
			if (!ignore_list.contains(be_name)) {
				var lib_entry = WS.Behaviorlib.getByName(be_name);
				WS.Behaviorlib.updateEntry(lib_entry);
				ignore_list.push(be_name);
				ignore_list = that.loadBehaviorDependencies(lib_entry.getBehaviorManifest(), ignore_list);
			}
		});
		return ignore_list;
	}

}) ();
