UI.Settings = new (function() {
	var that = this;

	var App = require('nw.gui').App;

	var ros_pkg_cache;
	var state_pkg_cache;
	var behavior_pkg_cache;

	var runtime_timeout;
	var stop_behaviors;
	var collapse_info;
	var collapse_warn;
	var collapse_error;
	var collapse_hint;

	var default_package;
	var code_indentation;
	var editor_command;
	var save_on_robot;
	var ssh_hostname;
	var ssh_username;

	var transition_mode;
	var gridsize;
	var commands_enabled;
	var commands_key;

	var synthesis_enabled;
	var synthesis_topic;
	var synthesis_type;
	var synthesis_system;

	var storeSettings = function() {
		chrome.storage.local.set({
			'ros_pkg_cache': [],
			'state_pkg_cache': [],
			'behavior_pkg_cache': [],
			'runtime_timeout': runtime_timeout,
			'stop_behaviors': stop_behaviors,
			'collapse_info': collapse_info,
			'collapse_warn': collapse_warn,
			'collapse_error': collapse_error,
			'collapse_hint': collapse_hint,
			'default_package': default_package,
			'code_indentation': code_indentation,
			'editor_command': editor_command,
			'save_on_robot': save_on_robot,
			'ssh_username' : ssh_username,
			'ssh_hostname' : ssh_hostname,
			'transition_mode': transition_mode,
			'gridsize': gridsize,
			'commands_enabled': commands_enabled,
			'commands_key': commands_key,
			'synthesis_enabled': synthesis_enabled,
			'synthesis_topic': synthesis_topic,
			'synthesis_type': synthesis_type,
			'synthesis_system': synthesis_system
		});
		displaySettingsHints();
	}


	this.restoreSettings = function(restored_callback) {
		chrome.storage.local.get({
			'ros_pkg_cache': [],
			'state_pkg_cache': [],
			'behavior_pkg_cache': [],
			'runtime_timeout': 10,
			'stop_behaviors': false,
			'collapse_info': true,
			'collapse_warn': true,
			'collapse_error': false,
			'collapse_hint': false,
			'default_package': 'flexbe_behaviors',
			'code_indentation': 0,
			'editor_command': 'gedit --new-window $FILE +$LINE',
			'save_on_robot' : false,
			'ssh_username': "",
			'ssh_hostname' : "",
			'transition_mode': 1,
			'gridsize': 50,
			'commands_enabled': false,
			'commands_key': '',
			'synthesis_enabled': false,
			'synthesis_topic': '',
			'synthesis_type': 'flexbe_msgs/BehaviorSynthesisAction',
			'synthesis_system': ''
		}, function(items) {
			ros_pkg_cache = items.ros_pkg_cache;
			state_pkg_cache = items.state_pkg_cache;
			behavior_pkg_cache = items.behavior_pkg_cache;

			runtime_timeout = items.runtime_timeout;
			document.getElementById("input_runtime_timeout").value = items.runtime_timeout;
			stop_behaviors = items.stop_behaviors;
			document.getElementById("cb_stop_behaviors").checked = items.stop_behaviors;
			collapse_info = items.collapse_info;
			document.getElementById("cb_collapse_info").checked = items.collapse_info;
			collapse_warn = items.collapse_warn;
			document.getElementById("cb_collapse_warn").checked = items.collapse_warn;
			collapse_error = items.collapse_error;
			document.getElementById("cb_collapse_error").checked = items.collapse_error;
			collapse_hint = items.collapse_hint;
			document.getElementById("cb_collapse_hint").checked = items.collapse_hint;
			
			default_package = items.default_package;
			that.createBehaviorPackageSelect(document.getElementById("select_default_package"));
			code_indentation = items.code_indentation;
			document.getElementById("select_code_indentation").selectedIndex = items.code_indentation;
			editor_command = items.editor_command;
			document.getElementById("input_editor_command").value = items.editor_command;
			save_on_robot = items.save_on_robot;
            document.getElementById("cb_save_on_robot").checked = items.save_on_robot;
            ssh_username = items.ssh_username;
            document.getElementById("input_ssh_user_name").value = items.ssh_username;
            ssh_hostname = items.ssh_hostname;
            document.getElementById("input_ssh_host_name").value = items.ssh_hostname;
            document.getElementById("input_ssh_user_name").disabled = !items.save_on_robot;
            document.getElementById("input_ssh_host_name").disabled = !items.save_on_robot;



			transition_mode = items.transition_mode;
			document.getElementById("select_transition_mode").selectedIndex = items.transition_mode;
			gridsize = items.gridsize;
			document.getElementById("input_gridsize").value = items.gridsize;
			commands_enabled = items.commands_enabled;
			document.getElementById("cb_commands_enabled").checked = items.commands_enabled;
			commands_key = items.commands_key;
			document.getElementById("input_commands_key").value = items.commands_key;

			synthesis_enabled = items.synthesis_enabled;
			document.getElementById("cb_synthesis_enabled").checked = items.synthesis_enabled;
			synthesis_topic = items.synthesis_topic;
			document.getElementById("input_synthesis_topic").value = items.synthesis_topic;
			synthesis_type = items.synthesis_type;
			document.getElementById("input_synthesis_type").value = items.synthesis_type;
			synthesis_system = items.synthesis_system;
			document.getElementById("input_synthesis_system").value = items.synthesis_system;
			updateSynthesisInterface();

			IO.PackageParser.discover(ros_pkg_cache, (new_packages, added_states, added_behaviors) => {
				ros_pkg_cache = ros_pkg_cache.concat(new_packages);
				state_pkg_cache = state_pkg_cache.concat(added_states);
				behavior_pkg_cache = behavior_pkg_cache.concat(added_behaviors);
				storeSettings(); // to update cache
				updateWorkspaceDisplay();

				that.createBehaviorPackageSelect(document.getElementById("select_default_package"));
				that.createBehaviorPackageSelect(document.getElementById("select_behavior_package"));
				Behavior.setBehaviorPackage(document.getElementById('select_behavior_package').value);

				// async
				that.updateStatelib();
				that.updateBehaviorlib();
			});

			that.setRosProperties('');
		});
	}

	this.updateStatelib = function() {
		WS.Statelib.resetLib();
		state_pkg_cache.forEach(state_pkg => {
			IO.PackageParser.parseStateFolder(state_pkg['path'], state_pkg['path'], false);
		});
	}

	this.updateBehaviorlib = function() {
		WS.Behaviorlib.resetLib();
		behavior_pkg_cache.forEach(behavior_pkg => {
			IO.PackageParser.parseBehaviorFolder(behavior_pkg['path'], behavior_pkg['name']);
		});
	}

	this.createBehaviorPackageSelect = function(select_el, add_all_option) {
		select_el.innerHTML = "";
		if (behavior_pkg_cache == undefined || behavior_pkg_cache.length == 0) {
			return;
		}
		if (add_all_option) {
			var option = document.createElement("option");
			option.setAttribute("value", "ALL");
			option.innerText = "ALL";
			select_el.appendChild(option);
		}
		for (var i=0; i<behavior_pkg_cache.length; i++) {
			var option = document.createElement("option");
			option.setAttribute("value", behavior_pkg_cache[i]["name"]);
			option.innerText = behavior_pkg_cache[i]["name"];
			select_el.appendChild(option);
		}
		if (!add_all_option) {
			var default_package_index = behavior_pkg_cache.map((pkg) => { return pkg['name']; }).indexOf(default_package);
			if (default_package_index < 0) {
				default_package_index = 0;
				default_package = behavior_pkg_cache[0]['name'];
			}
			select_el.selectedIndex = default_package_index;
		}
	}

	this.createStatePackageSelect = function(select_el, add_all_option) {
		select_el.innerHTML = "";
		if (state_pkg_cache == undefined || state_pkg_cache.length == 0) {
			return;
		}
		if (add_all_option) {
			var option = document.createElement("option");
			option.setAttribute("value", "ALL");
			option.innerText = "ALL";
			select_el.appendChild(option);
		}
		for (var i=0; i<state_pkg_cache.length; i++) {
			var option = document.createElement("option");
			option.setAttribute("value", state_pkg_cache[i]["name"]);
			option.innerText = state_pkg_cache[i]["name"];
			select_el.appendChild(option);
		}
	}

	var displaySettingsHints = function() {
		if (behavior_pkg_cache == undefined || behavior_pkg_cache.length == 0) {
			var action_div = document.createElement("div");

			var pkg_select = document.createElement("select");
			pkg_select.setAttribute("style", "width:100%; margin: 5px 0;");
			var pkg_select_update_title = function() {
				pkg_select.setAttribute("title", pkg_select.options[pkg_select.selectedIndex].getAttribute("title"));
			};
			pkg_select.addEventListener('change', pkg_select_update_title);
			var packages = ros_pkg_cache.filter((pkg) => { return !pkg['path'].startsWith("/opt/ros"); });
			for (var i=0; i<packages.length; i++) {
				var option = document.createElement("option");
				option.setAttribute("value", packages[i]["name"]);
				option.setAttribute("title", packages[i]["path"]);
				option.innerText = packages[i]["name"];
				pkg_select.appendChild(option);
			}

			var suggestion = packages.findElement((pkg) => { return pkg['name'] == "flexbe_behaviors"; });
			suggestion = suggestion || packages.findElement((pkg) => { return pkg['name'].indexOf("flexbe_behaviors") != -1; });
			if (suggestion != undefined) {
				pkg_select.selectedIndex = packages.indexOf(suggestion);
			}
			pkg_select_update_title();

			var pkg_convert_cb = document.createElement("input");
			pkg_convert_cb.setAttribute("id", "pkg_convert_cb");
			pkg_convert_cb.setAttribute("type", "checkbox");
			if (suggestion != undefined) pkg_convert_cb.setAttribute("checked", "checked");
			var pkg_convert_label = document.createElement("label");
			pkg_convert_label.setAttribute("for", "pkg_convert_cb");
			pkg_convert_label.innerText = "Convert existing behaviors";
			var pkg_convert_group = document.createElement("div");
			pkg_convert_group.setAttribute("title", "If this package already contains behaviors in the old format, import them into this package. You can remove the old behavior packages afterwards.");
			pkg_convert_group.setAttribute("style", "vertical-align:middle; margin: 0 0 5px 0;");
			pkg_convert_group.appendChild(pkg_convert_cb);
			pkg_convert_group.appendChild(pkg_convert_label);

			var pkg_init_button = document.createElement("input");
			pkg_init_button.setAttribute("value", "Initialize");
			pkg_init_button.setAttribute("type", "button");
			pkg_init_button.addEventListener("click", function() {
				T.clearLog();
				T.logInfo("Initializing package " + pkg_select.value + "...");
				T.show();
				var pkg_name = pkg_select.value;
				var convert = pkg_convert_cb.checked;
				IO.PackageGenerator.initializeBehaviorPackage(pkg_name, convert, () => {
					that.restoreSettings();
					T.logInfo("Initialization done!");
					if (convert) {
						T.logInfo("You can now remove the old behavior packages and the /behaviors folder in "+pkg_name+".");
					}
				});
			});

			action_div.appendChild(pkg_select);
			action_div.appendChild(pkg_convert_group);
			action_div.appendChild(pkg_init_button);
			UI.Feed.displayCustomMessage('msg_no_behavior_packages', 1, 'No Behavior Packages',
				'There are no behavior packages available. Please initialize a ROS package for this purpose or prepare one manually.',
				action_div
			);
		} else {
			var msg = document.getElementById('msg_no_behavior_packages');
			if (msg != undefined) msg.parentNode.removeChild(msg);
		}

		if (state_pkg_cache == undefined || state_pkg_cache.length == 0) {
			UI.Feed.displayCustomMessage('msg_no_state_packages', 1, 'No State Packages',
				'The list of available states is empty. You can find available states <a href="https://github.com/FlexBE" target="_blank">on Github</a>.'
			);
		} else {
			var msg = document.getElementById('msg_no_state_packages');
			if (msg != undefined) msg.parentNode.removeChild(msg);
		}
	}

	var updateWorkspaceDisplay = function() {
		var createEntry = function(pkg) {
			var entry = document.createElement("div");
			entry.setAttribute("class", "tag");
			entry.setAttribute("title", pkg['path']);
			entry.innerText = pkg['name'];
			return entry;
		}
		var behavior_el = document.getElementById("workspace_behavior_packages");
		behavior_el.innerHTML = "";
		behavior_pkg_cache.forEach((behavior_pkg) => {
			var entry = createEntry(behavior_pkg);
			behavior_el.appendChild(entry);
		});
		var state_el = document.getElementById("workspace_state_packages");
		state_el.innerHTML = "";
		state_pkg_cache.forEach((state_pkg) => {
			var entry = createEntry(state_pkg);
			state_el.appendChild(entry);
		});
	}

	this.importConfiguration = function() {
		chrome.fileSystem.chooseEntry({
			type: 'openFile',
			accepts: [{
				description: 'Configuration (.json)',
				extensions: ['json']
			}]
		}, function(entry) {
			if (chrome.runtime.lastError) {
				return;
			}
			UI.Panels.setActivePanel(UI.Panels.NO_PANEL);
			entry.file(function(file) {
				var reader = new FileReader();
				reader.onload = function(event) {
					var config = JSON.parse(event.target.result);
					chrome.storage.local.set(config, function() {
						that.restoreSettings();
					});
				};
				reader.readAsText(file);
			});
		});
	}

	this.exportConfiguration = function() {
		chrome.fileSystem.chooseEntry({
			type: 'saveFile',
			suggestedName: 'flexbe_config.json',
			accepts: [{
				description: 'Configuration (.json)',
				extensions: ['json']
			}]
		}, function(entry) {
			if (chrome.runtime.lastError) {
				return;
			}
			chrome.storage.local.get(null, function(config) {
				var truncated = false;
				var content = JSON.stringify(config);
				entry.createWriter(function(writer) {
					writer.onerror = function(error) { T.logError("Error when exporting configuration: " + error); };
					writer.onwriteend = function() { 
						if (!truncated) {
							truncated = true;
							this.truncate(this.position);
						}
					};
					writer.write(new Blob([content], {type: 'text/plain'}));
				}, function(e) { T.logError("Error when exporting configuration: " + error); });
			});
		});
	}


	this.getVersion = function() {
		return App.manifest.version;
	}


	// Runtime
	//=========

	this.runtimeTimeoutChanged = function() {
		runtime_timeout = document.getElementById("input_runtime_timeout").value;
		RC.Controller.onboardTimeout = runtime_timeout;
		storeSettings();
	}

	this.stopBehaviorsClicked = function(evt) {
		stop_behaviors = evt.target.checked;
		storeSettings();
	}

	this.collapseInfoClicked = function(evt) {
		collapse_info = evt.target.checked;
		storeSettings();
	}

	this.collapseWarnClicked = function(evt) {
		collapse_warn = evt.target.checked;
		storeSettings();
	}

	this.collapseErrorClicked = function(evt) {
		collapse_error = evt.target.checked;
		storeSettings();
	}

	this.collapseHintClicked = function(evt) {
		collapse_hint = evt.target.checked;
		storeSettings();
	}

	this.isStopBehaviors = function() {
		return stop_behaviors;
	}

	this.isCollapseInfo = function() { return collapse_info; }
	this.isCollapseWarn = function() { return collapse_warn; }
	this.isCollapseError = function() { return collapse_error; }
	this.isCollapseHint = function() { return collapse_hint; }


	// Code
	//======

	this.defaultPackageChanged = function() {
		var el = document.getElementById('select_default_package');
		default_package = el.value;
		storeSettings();
	}

	this.codeIndentationChanged = function() {
		var el = document.getElementById('select_code_indentation');
		code_indentation = el.selectedIndex;
		storeSettings();
	}

	this.editorCommandChanged = function() {
		var el = document.getElementById('input_editor_command');
		editor_command = el.value;
		storeSettings();
	}

	this.getDefaultPackage = function() {
		return default_package;
	}

	this.getCodeIndentation = function() {
		var chars = ['\t', '  ', '    ', '        '];
		return chars[code_indentation];
	}

	this.getEditorCommand = function(file_path, line_number) {
		if (line_number == undefined) line_number = 0;
		return editor_command.replace("$LINE", line_number).replace("$FILE", file_path);
	}

    this.isSaveOnRobotEnabled = function() { return save_on_robot; }

    this.saveOnRobotClicked = function(evt) {
        save_on_robot = evt.target.checked;
        storeSettings();
        document.getElementById("input_ssh_user_name").disabled = !save_on_robot;
        document.getElementById("input_ssh_host_name").disabled = !save_on_robot;
    }

    this.sshUsernameChanged = function(evt) {
        ssh_username = document.getElementById('input_ssh_user_name').value;
        storeSettings();
    }

    this.sshHostnameChanged = function(evt) {
        ssh_hostname = document.getElementById('input_ssh_host_name').value;
        storeSettings();
    }

    this.getRobotHostname = function() {
        return document.getElementById('input_ssh_host_name').value;
    }

	this.getRobotUsername = function() {
	    return document.getElementById('input_ssh_user_name').value;
	}


	// Editor
	//========

	this.transitionEndpointsChanged = function() {
		var el = document.getElementById('select_transition_mode');
		transition_mode = el.selectedIndex;
		storeSettings();
	}

	this.gridsizeChanged = function() {
		var el = document.getElementById('input_gridsize');
		gridsize = parseInt(el.value);
		storeSettings();
	}

	this.commandsEnabledClicked = function(evt) {
		commands_enabled = evt.target.checked;
		storeSettings();
	}

	this.commandsKeyChanged = function() {
		var el = document.getElementById('input_commands_key');
		commands_key = el.value;
		storeSettings();
	}

	this.isTransitionModeCentered = function() {
		return transition_mode == 0;
	}

	this.isTransitionModeCombined = function() {
		return transition_mode == 2;
	}

	this.getGridsize = function() {
		return gridsize;
	}

	this.isCommandsEnabled = function() {
		return commands_enabled;
	}

	this.getCommandsKey = function() {
		return commands_key;
	}


	// ROS Properties
	//================

	this.setRosProperties = function(ns) {
		if (ns == '')  ns = '/';
		if (ns != '/') {
			document.getElementById('label_editor_title').innerHTML = ns.slice(1,-1);
		}
		document.getElementById('ros_prop_namespace').value = ns;
		var status_disp = document.getElementById('ros_prop_status');
		var connect_button = document.getElementById('button_ros_connect');
		if (RC.ROS.isConnected()) {
			status_disp.value = "Connected to master!";
			status_disp.style.color = "#090";
			connect_button.value = "Disconnect";
		} else if (RC.ROS.isTrying()) {
			status_disp.value = "Waiting for master...";
			status_disp.style.color = "#900";
			connect_button.value = "Disconnect";
		} else {
			status_disp.value = "Not connecting.";
			status_disp.style.color = "#999";
			connect_button.value = "Connect";
		}
	}

	this.rosConnectClicked = function() {
		if (RC.ROS.isConnected()) {
			RC.ROS.closeConnection();
		} else if (RC.ROS.isTrying()) {
			RC.ROS.closeConnection();
		} else {
			RC.ROS.trySetupConnection();
		}
	}


	// Workspace
	//===========

	this.forceDiscoverClicked = function() {
		ros_pkg_cache = [];
		state_pkg_cache = [];
		behavior_pkg_cache = [];
		storeSettings();

	}


	// Synthesis
	//===========

	this.synthesisEnabledClicked = function(evt) {
		synthesis_enabled = evt.target.checked;
		storeSettings();
		updateSynthesisInterface();
	}

	this.synthesisTopicChanged = function() {
		var el = document.getElementById('input_synthesis_topic');
		synthesis_topic = el.value;
		storeSettings();
	}

	this.synthesisTypeChanged = function() {
		var el = document.getElementById('input_synthesis_type');
		synthesis_type = el.value;
		storeSettings();
	}

	this.synthesisSystemChanged = function() {
		var el = document.getElementById('input_synthesis_system');
		synthesis_system = el.value;
		storeSettings();
	}

	this.isSynthesisEnabled = function() {
		return synthesis_enabled;
	}

	this.getSynthesisTopic = function() {
		return synthesis_topic;
	}

	this.getSynthesisType = function() {
		return synthesis_type;
	}

	this.getSynthesisSystem = function() {
		return synthesis_system;
	}

	var updateSynthesisInterface = function() {
		if (synthesis_enabled) {
			document.getElementById('synthesis_display_option').style.display = "inline";
			if (RC.ROS.isConnected()) {
				RC.PubSub.initializeSynthesisAction();
			}
		} else {
			document.getElementById('synthesis_display_option').style.display = "none";
		}
	}

}) ();