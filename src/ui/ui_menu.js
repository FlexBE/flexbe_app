UI.Menu = new (function() {
	var that = this;

	var os = require('os');
	var sys = require('sys');
	var spawn = require('child_process').spawn;
	var path = require('path');

	var keys = ["db", "sm", "rc", "se"];

	var current_page = "db";

	var setFocus = function(target) {
		for (var i=0; i<keys.length; ++i) {
			key = keys[i];
			active = (key == target)? "_active" : "";
			document.getElementById("button_to_" + key).setAttribute("class", "category_button" + active);
			document.getElementById("button_to_" + key).children[0].setAttribute("src", "img/" + key + active + ".png");
		}
		current_page = target;
	}

	var button_config_db = [
		[
			["New Behavior", "file_new", function() { UI.Menu.newBehaviorClicked(); }],
			["Load Behavior", "file_load", function() { UI.Menu.loadBehaviorClicked(); }],
			["Save Behavior", "file_save", function() { UI.Menu.saveBehaviorClicked(); }]
		],
		[
			["Edit Code", "page_edit", function() { UI.Menu.scEditClicked(); }, "ctrl+e"]
		],
		[
			["Check Behavior", "check", function() { UI.Menu.checkBehaviorClicked(); }]
		]
	];
	var button_config_sm = [
		[
			["Add State", "add", function() { UI.Menu.addStateClicked(); }, "ctrl+1"],
			["Add Behavior", "add", function() { UI.Menu.addBehaviorClicked(); }, "ctrl+2"],
			["Add Container", "add", function() { UI.Menu.addStatemachineClicked(); }, "ctrl+3"]
		],
		[
			["Data Flow Graph", "dataflow", function() { UI.Statemachine.toggleDataflow(); }, "ctrl+d"],
			["Check Behavior", "check", function() { UI.Menu.checkBehaviorClicked(); }],
			["Save Behavior", "file_save", function() { UI.Menu.saveBehaviorClicked(); }]
		],
		[
			["Undo", "undo", function() { ActivityTracer.undo(); }, undefined],
			["Redo", "redo", function() { ActivityTracer.redo(); }, undefined],
			["Reset", "cross", function() { ActivityTracer.resetToSave(); }]
		],
		[
			["Hide Comments", "note", function() { UI.Statemachine.toggleComments(); }, "ctrl+h"],
			["Write Comment", "note_add", function() { UI.Menu.addCommentClicked(); }, "ctrl+4"]
		],
		[
			["Fade Outcomes", "outcome", function() { UI.Statemachine.toggleOutcomes(); }, "ctrl+f"],
			["Auto-Connect", "autoconnect", function() { Tools.autoconnect(); }, "ctrl+a"],
			["Group Selection", "group_selection", function() { Tools.groupSelection(); }, "ctrl+g"]
		]
	];
	var button_config_rc = [
		[
			["Show Terminal", "title_terminal", function() { UI.Menu.terminalClicked(); }, undefined]
		]
	];
	var button_config_se = [
		[
			["Show Terminal", "title_terminal", function() { UI.Menu.terminalClicked(); }, undefined]
		],
		[
			["Import Configuration", "settings_import", function() { UI.Settings.importConfiguration(); }],
			["Export Configuration", "settings_export", function() { UI.Settings.exportConfiguration(); }]
		]
	];

	var setMenuButtons = function(config) {
		panel = document.getElementById("title_button_panel");
		panel.innerHTML = "";
		for (var c=0; c<config.length; ++c) {
			column = document.createElement("div");
			column.setAttribute("class", "tool_category");
			table = document.createElement("table");
			table.setAttribute("cellspacing", "0");
			table.setAttribute("cellpadding", "0");
			for (var r=0; r<config[c].length; ++r) {
				button = config[c][r];
				tr = document.createElement("tr");
				td = document.createElement("td");
				td.setAttribute("class", "tool_button");
				td.setAttribute("id", "tool_button " + button[0]);
				td.innerHTML = 
					'<table cellpadding="0" cellspacing="0"><tr><td valign="middle">' +
						'<img src="img/' + button[1] + '.png" />' +
					'</td><td valign="middle" style="padding-left:5px">' +
						button[0] +
					'</td></tr></table>';
				td.addEventListener("click", button[2]);
				tr.appendChild(td);
				table.appendChild(tr);
			}
			column.appendChild(table);
			panel.appendChild(column);
		}
	}

	this.isPageDashboard = function() { return current_page == "db"; }
	this.isPageStatemachine = function() { return current_page == "sm"; }
	this.isPageControl = function() { return current_page == "rc"; }
	this.isPageSettings = function() { return current_page == "se"; }

	this.configureKeybindings = function() {
		[[button_config_db, that.isPageDashboard],
		 [button_config_sm, that.isPageStatemachine],
		 [button_config_rc, that.isPageControl],
		 [button_config_se, that.isPageSettings]
		].forEach(function(element) {
			element[0].forEach(function(column) {
				column.forEach(function(button) {
					if (button[3] == undefined) return;
					Mousetrap.bind(button[3], function() {
						if (!element[1]()) return;
						button[2]();
		}); }); }); });
	}

	this.toDashboardClicked = function() {
		document.getElementById("dashboard").style.left = "0px";
		document.getElementById("statemachine").style.left = "calc(100% + 50px)";
		document.getElementById("runtimecontrol").style.left = "calc((100% + 50px)*2)";
		document.getElementById("settings").style.left = "calc((100% + 50px)*3)";
		setFocus("db");
		setMenuButtons(button_config_db);
	}

	this.toStatemachineClicked = function() {
		document.getElementById("dashboard").style.left = "calc(-100% - 50px)";
		document.getElementById("statemachine").style.left = "0px";
		document.getElementById("runtimecontrol").style.left = "calc(100% + 50px)";
		document.getElementById("settings").style.left = "calc((100% + 50px)*2)";
		setFocus("sm");
		setMenuButtons(button_config_sm);
		UI.Statemachine.refreshView();
	}

	this.toControlClicked = function() {
		document.getElementById("dashboard").style.left = "calc((-100% - 50px)*2)";
		document.getElementById("statemachine").style.left = "calc(-100% - 50px)";
		document.getElementById("runtimecontrol").style.left = "0px";
		document.getElementById("settings").style.left = "calc(100% + 50px)";
		setFocus("rc");
		setMenuButtons(button_config_rc);
	}

	this.toSettingsClicked = function() {
		document.getElementById("dashboard").style.left = "calc((-100% - 50px)*3)";
		document.getElementById("statemachine").style.left = "calc((-100% - 50px)*2)";
		document.getElementById("runtimecontrol").style.left = "calc(-100% - 50px)";
		document.getElementById("settings").style.left = "0px";
		setFocus("se");
		setMenuButtons(button_config_se);
	}

	this.displayRuntimeStatus = function(status) {
		txt = document.getElementById("runtime_status_txt");
		txt.innerHTML = status;
		img = document.getElementById("runtime_status_img");
		color = "#aaa";
		switch(status) {
			case 'offline': img.setAttribute("src", "img/link_break.png"); color = "#aaa"; break;
			case 'disconnected': img.setAttribute("src", "img/cross.png"); color = "#d66"; break;
			case 'online': img.setAttribute("src", "img/link.png"); color = "#aaa"; break;
			case 'running': img.setAttribute("src", "img/tick.png"); color = "#3a3"; break;
			case 'locked': img.setAttribute("src", "img/lock.png"); color = "#b85"; break;
			case 'message': img.setAttribute("src", "img/information.png"); color = "#66b"; break;
			case 'warning': img.setAttribute("src", "img/warning.png"); color = "#b85"; break;
			case 'error': img.setAttribute("src", "img/error.png"); color = "#d66"; break;
		}
		txt.style.color = color;
	}

	this.scEditClicked = function() {
		var names = Behavior.createNames();
		var package_name = Behavior.getBehaviorPackage();
		ROS.getPackagePath(package_name, (package_path) => {
			try {
				var file_path = path.join(package_path, 'src', package_name, names.file_name);
				var command = UI.Settings.getEditorCommand(file_path).split(' ');
				var proc = spawn(command[0], command.slice(1));
				proc.stderr.on('data', (data) => {
					T.logWarn(data);
				});
			} catch (err) {
				T.logError("Unable to open editor. Make sure you already saved the behavior to generate code files.");
			}
		});
	}

	this.addStateClicked = function() {
		if (UI.Statemachine.isReadonly()) return;

		UI.Panels.AddState.show();
	}

	this.addBehaviorClicked = function() {
		if (UI.Statemachine.isReadonly()) return;

		UI.Panels.SelectBehavior.setSelectionCallback(function(manifest) {
			IO.BehaviorLoader.loadBehaviorInterface(manifest, function(smi) {
				if (smi.class_name != manifest.class_name) T.logWarn("Class names of behavior " + manifest.name + " do not match!");
				var be_def = WS.Behaviorlib.getByName(manifest.name);
				var be = new BehaviorState(manifest.name, be_def, []);
				be.setStateName(Tools.getUniqueName(UI.Statemachine.getDisplayedSM(), be.getStateName()));
				UI.Statemachine.getDisplayedSM().addState(be);
				UI.Statemachine.refreshView();
				UI.Panels.StateProperties.displayStateProperties(be);

				var be_name = manifest.name;
				var state_path = be.getStatePath();
				var container_path = be.getContainer().getStatePath();

				ActivityTracer.addActivity(ActivityTracer.ACT_STATE_ADD,
					"Added new state taken from behavior " + manifest.name,
					function() {
						var state = Behavior.getStatemachine().getStateByPath(state_path);
						state.getContainer().removeState(state);
						if (UI.Panels.StateProperties.isCurrentState(state)) {
							UI.Panels.StateProperties.hide();
						}
						UI.Statemachine.refreshView();
					},
					function() {
						var container = (container_path == "")? Behavior.getStatemachine() : Behavior.getStatemachine().getStateByPath(container_path);
						var redo_state = new BehaviorState(be_name, WS.Behaviorlib.getByName(be_name), []);
						container.addState(redo_state);
						UI.Statemachine.refreshView();
					}
				);
			});
		});
		UI.Panels.SelectBehavior.enableHover();
		UI.Panels.SelectBehavior.show();
	}

	this.addStatemachineClicked = function() {
		if (UI.Statemachine.isReadonly()) return;

		var sm_def = new WS.StateMachineDefinition(['finished', 'failed'], [], []);
		var state_name = Tools.getUniqueName(UI.Statemachine.getDisplayedSM(), "Container");
		var sm = new Statemachine(state_name, sm_def);
		UI.Statemachine.getDisplayedSM().addState(sm);
		UI.Statemachine.refreshView();
		UI.Panels.StateProperties.displayStateProperties(sm);

		var state_path = sm.getStatePath();
		var container_path = sm.getContainer().getStatePath();

		ActivityTracer.addActivity(ActivityTracer.ACT_STATE_ADD,
			"Added new container",
			function() {
				var state = Behavior.getStatemachine().getStateByPath(state_path);
				if (UI.Statemachine.getDisplayedSM().getStatePath() == state.getStatePath()) {
					UI.Statemachine.setDisplayedSM(state.getContainer());
				}
				state.getContainer().removeState(state);
				if (UI.Panels.StateProperties.isCurrentState(state)) {
					UI.Panels.StateProperties.hide();
				}
				UI.Statemachine.refreshView();
			},
			function() {
				var container = (container_path == "")? Behavior.getStatemachine() : Behavior.getStatemachine().getStateByPath(container_path);
				var redo_state = new Statemachine(state_name, new WS.StateMachineDefinition(['finished', 'failed'], [], []));
				container.addState(redo_state);
				UI.Statemachine.refreshView();
			}
		);

		return sm;
	}

	this.terminalClicked = function() {
		UI.Panels.Terminal.show();
	}

	this.saveBehaviorClicked = function() {
		var check_error_string = undefined;
		if (Behavior.isReadonly()) {
			check_error_string = "behavior has been loaded from a read-only file";
		} else {
			check_error_string = Checking.checkBehavior();
		}
		if (check_error_string != undefined) {
			T.clearLog();
			T.show();
			T.logError("Unable to save behavior: " + check_error_string);
			return;
		}
		var warnings = Checking.warnBehavior();
		IO.BehaviorSaver.saveStateMachine();
		warnings.forEach(function(w) {
			T.logWarn("Warning: " + w);
		});
		ActivityTracer.addSave();
	}

	this.newBehaviorClicked = function() {
		if (RC.Controller.isReadonly()) return;

		// abort behavior execution if running

		Behavior.resetBehavior();
		UI.Dashboard.resetAllFields();
		UI.Statemachine.resetStatemachine();

		// make sure a new behavior always starts at the dashboard
		UI.Menu.toDashboardClicked();
		UI.Panels.setActivePanel(UI.Panels.NO_PANEL);

		UI.Dashboard.addBehaviorOutcome('finished');
		UI.Dashboard.addBehaviorOutcome('failed');

		ActivityTracer.resetActivities();
	}

	this.loadBehaviorClicked = function() {
		if (RC.Controller.isReadonly()) return;
		
		UI.Panels.SelectBehavior.setSelectionCallback(function(manifest) {
			IO.BehaviorLoader.loadBehavior(manifest);
		});
		UI.Panels.SelectBehavior.show();
	}

	this.checkBehaviorClicked = function() {
		T.clearLog();
		T.show();
		T.logInfo("Performing behavior checks...");
		var error_string = Checking.checkBehavior();
		if (error_string != undefined) {
			T.logError("Found error: " + error_string);
		} else {
			// generate warnings
			var warnings = Checking.warnBehavior();
			warnings.forEach(function(w) {
				T.logWarn("Warning: " + w);
			});

			T.logInfo("Behavior is valid!");
		}
	}

	this.addCommentClicked = function() {
		if (UI.Statemachine.isReadonly()) return;
		if (Behavior.getCommentNotes().findElement(function(n) { return n.getContent() == ""; }) != undefined) return;

		var note = new Note("");
		note.setContainerPath(UI.Statemachine.getDisplayedSM().getStatePath());
		Behavior.addCommentNote(note);
		UI.Statemachine.refreshView();
	}

}) ();