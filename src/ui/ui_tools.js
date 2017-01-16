UI.Tools = new (function() {
	var that = this;

	var command_history = [];
	var command_history_idx = 0;
	var command_library = CommandLib.load();

	var last_ros_command = undefined; 

	Mousetrap.bind("ctrl+space", function() { UI.Tools.toggle(); }, 'keydown');
	Mousetrap.bind("ctrl+space", function() { UI.Tools.evaluate(); }, 'keyup');
	Mousetrap.bind("enter", function() { that.displayCommandInput(); }, 'keyup');
	var mouse = {x: 0, y: 0};
	document.addEventListener('mousemove', function(e){ 
		mouse.x = e.clientX || e.pageX; 
		mouse.y = e.clientY || e.pageY 
	}, false);
	var hover_mode = false;
	var closest_button = undefined;

	var id_list = ["dfg", "undo", "redo", "copy", "cut", "paste", "terminal", "save"];

	var hide_timeout = undefined;

	var getPanel = function() { return document.getElementById("tool_overlay"); }

	var placeButtonsCircular = function(radius, animate) {
		for (var i = 0; i < id_list.length; i++) {
			var d = document.createElement("div");
			d.setAttribute("class", "tool_overlay_button");
			getPanel().appendChild(d);
			var el = document.getElementById("tool_overlay_" + id_list[i]);
			var angle = -225 + i * 45;
			d.setAttribute("style", "transform: rotate(" + angle + "deg) translate(" + radius + "px) rotate(" + -angle + "deg);");
			var bb = d.getBoundingClientRect();
			var p = el.parentNode;
			el.setAttribute("style", "top: " + (bb.top - parseInt(p.style.top)) + "px; left: " + (bb.left - parseInt(p.style.left)) + "px;"
							+ (animate? "transition: all 0.2s ease;" : ""));
			p.removeChild(d);
		}
	}

	var getClosestButton = function() {
		var panel = getPanel();
		var angle = Math.atan2((mouse.y - (parseInt(panel.style.top) + 50)), (mouse.x - (parseInt(panel.style.left) + 50)));
		angle = angle * 180 / Math.PI;
		var snapped_angle = Math.round(angle / 45) * 45;
		var angle_index = (snapped_angle + 225) / 45;
		angle_index = angle_index % 8;
		return document.getElementById("tool_overlay_" + id_list[angle_index]);
	}

	var createHistoryContent = function() {
		var activities = ActivityTracer.getActivityList();
		var current = ActivityTracer.getCurrentIndex();
		var save = ActivityTracer.getLastSaveIndex();
		var changes = ActivityTracer.hasUnsavedChanges();

		var info = document.getElementById("history_info");
		info.innerHTML = "<b>" + Behavior.getBehaviorName() + "</b><br /><br />";
		if (!changes) {
			info.innerHTML += "Last saved: <i>no changes</i><br />";
		} else if (save == 0) {
			info.innerHTML += "Last saved: <i>never</i><br />";
		} else {
			var savetime = Math.round((Date.now() - activities[save].time) / 60000);
			info.innerHTML += "Last saved: <i>" + savetime + " min ago</i><br />";
		}

		var list = document.getElementById("history_list");
		list.innerHTML = "";
		for (var i = activities.length - 1; i >= 0; i--) {
			var a = (i > 0)? activities[i] : {description: "<i>no changes</i>"};
			var entry = document.createElement("div");
			entry.setAttribute("class", "history_element");
			entry.innerHTML = ((i == current)? "> " : "") + ((i == save)? "# " : "") + a.description;
			list.appendChild(entry);

			var addListener = function(index) {
				entry.addEventListener('click', function() {
					ActivityTracer.goToIndex(index);
					createHistoryContent();
				});
			};
			addListener(i);
		}
	}


	this.printAvailableCommands = function() {
		T.clearLog();
		T.show();
		T.logInfo("The following commands are available:");
		command_library.forEach(function(c) {
			T.logInfo(c.desc + '<font style="color: #999; font-style: italic;"> - ' + c.text + '</font>');
		});
	}

	this.display = function() {
		if (RC.Controller.isReadonly()) return;

		if (hide_timeout != undefined) clearTimeout(hide_timeout);
		var panel = getPanel();

		var position_top = Math.min(Math.max(mouse.y - 50, 120), window.innerHeight - 210);
		var position_left = Math.min(Math.max(mouse.x - 50, 120), window.innerWidth - 210);

		panel.style.display = "block";
		panel.style.top = position_top + "px";
		panel.style.left = position_left + "px";
		placeButtonsCircular(300, false);

		setTimeout(function() {
			placeButtonsCircular(120, true);
			panel.style.opacity = "1";
		}, 10);
	}

	this.hide = function() {
		getPanel().style.opacity = "0";
		hide_timeout = setTimeout(function() {
			getPanel().style.display = "none";
			document.getElementById("command_overlay").style.display = "none";
		}, 200);
		placeButtonsCircular(300, true);

		document.getElementById("tool_input_command").blur();
		document.getElementById("tool_input_command").value = "";

		document.getElementById("history_overlay").style.left = "-295px";
		document.getElementById("command_overlay").style.bottom = "0px";
		document.getElementById("command_overlay").style.opacity = "0";
		document.getElementById("command_overlay_suggestions").style.display = "none";
		document.getElementById("command_overlay_suggestions").style.opacity = "0";
	}

	this.toggle = function() {
		if (RC.Controller.isReadonly()) return;

		var panel = getPanel();
		if (!hover_mode) {
			if (panel.style.display != "block") {
				that.display();
				hover_mode = true;
			} else {
				that.hide();
			}
		} else {
			var mouse_distance = Math.sqrt(Math.pow(mouse.x - (parseInt(panel.style.left) + 50), 2) + Math.pow(mouse.y - (parseInt(panel.style.top) + 50), 2));
			if (mouse_distance > 170) {
				var new_closest_button = getClosestButton();
				if (closest_button != new_closest_button) {
					if (closest_button != undefined) 
						closest_button.style.backgroundColor = "";
					closest_button = new_closest_button;
					closest_button.style.backgroundColor = "rgba(0, 0, 0, 0.9)";
				}
			} else {
				if (closest_button != undefined) {
					closest_button.style.backgroundColor = "";
					closest_button = undefined;
				}
			}
		}
	}

	this.evaluate = function() {
		var panel = getPanel();
		if (hover_mode) {
			var will_stay = true;
			hover_mode = false;
			if (closest_button != undefined) {
				will_stay = false;
				closest_button.click();
				closest_button.style.backgroundColor = "";
				closest_button = undefined;
			} else {
				// triggering keydown again the first time is slow, so make sure we dont miss any selection
				var mouse_distance = Math.sqrt(Math.pow(mouse.x - (parseInt(panel.style.left) + 50), 2) + Math.pow(mouse.y - (parseInt(panel.style.top) + 50), 2));
				if (mouse_distance > 170) {
					getClosestButton().click();
					will_stay = false;
				}
			}
			if (will_stay) {
				// display panels


				document.getElementById("history_overlay").style.left = "-15px";
				ActivityTracer.setUpdateCallback(createHistoryContent);
				createHistoryContent();

				that.displayCommandInput();
			}
		}

	}

	this.displayCommandInput = function() {
		document.getElementById("command_overlay").style.display = "block";
		document.getElementById("command_overlay_suggestions").style.display = "block";
		setTimeout(function() {
			document.getElementById("command_overlay").style.bottom = "20px";
			document.getElementById("command_overlay").style.opacity = "1";
			document.getElementById("tool_input_command").focus();
		}, 10);
	}

	this.tryExecuteCommand = function(cmd) {
		try {
			command_history.push(cmd);
			command_history_idx = command_history.length;

			for (var i = 0; i < command_library.length; i++) {
				var c = command_library[i];
				var args = cmd.match(c.match);
				if (args != null) {
					c.impl(args);
					return true;
				}
			}

			if (cmd != "") {
				T.clearLog();
				T.show();
				T.logWarn("Command not recognized: " + cmd);
			}

		} catch (err) {
			T.logError(err.toString());
		}
		return false;
	}

	this.commandListener = function(event) {
		var hide = false;
		var cmd_input = document.getElementById("tool_input_command");
		var cmd = cmd_input.value;

		// process command
		if (event.keyCode == 13) { // enter
			hide = true;
			
			that.tryExecuteCommand(cmd);
		}

		// close overlay
		if (event.keyCode == 27 // esc
		 || event.keyCode == 32 && event.ctrlKey) { // ctrl+space
			hide = true;
		}

		// navigate history
		if (event.keyCode == 38) { // up arrow
			if (command_history_idx > 0) {
				command_history_idx -= 1;
				cmd = command_history[command_history_idx];
				cmd_input.value = cmd;
			}
		}
		if (event.keyCode == 40) { // down arrow
			if (command_history_idx < command_history.length) {
				command_history_idx += 1;
				cmd = (command_history_idx < command_history.length)? command_history[command_history_idx] : "";
				cmd_input.value = cmd;
			}
		}

		// add autocomplete
		var suggestions = (cmd == "")? [] : Autocomplete.generateCommandList(cmd, command_library);
		document.getElementById("command_overlay_suggestions").innerHTML = "";
		var displaySuggestions = function(s) {
			var div = document.createElement("div");
			div.setAttribute("class", "command_overlay_suggestion");
			div.innerText = s.text;
			document.getElementById("command_overlay_suggestions").appendChild(div);
		};
		suggestions.forEach(displaySuggestions);
		if (suggestions.length == 0) {
			document.getElementById("command_overlay_suggestions").style.opacity = "0";
		} else {
			document.getElementById("command_overlay_suggestions").style.opacity = "1";
		}
		if (event.keyCode == 39 // right arrow
			&& cmd_input.selectionStart == cmd.length
			&& suggestions.length > 0)
		{
			if (cmd.indexOf(" ") == -1) {
				cmd_input.value = cmd + Autocomplete.getSameFill(cmd, suggestions);
			} else {
				var suggested_cmd = command_library.findElement(function(el) { return el.desc == suggestions[0].text; });
				var count_spaces = cmd.split(" ").length - 1;
				var split_suggested_cmd = suggested_cmd.desc.split(" ");
				if (split_suggested_cmd[count_spaces].endsWith(":")) {
					cmd_input.value += split_suggested_cmd[count_spaces] + " ";
				} else {
					var arg_idx = count_spaces - cmd.split(":").length;
					var current_arg = cmd.substr(cmd.lastIndexOf(" ") + 1);
					if (suggested_cmd.completions != undefined && suggested_cmd.completions[arg_idx] != undefined) {
						var completions = suggested_cmd.completions[arg_idx]().filter(function(el) {
							return el.startsWith(current_arg);
						}).map(function(el) {
							return {text: el, hint: "", fill: el};
						});
						if (completions.length > 0) {
							var fill = Autocomplete.getSameFill(current_arg, completions);
							if (fill.length > 0 || completions.length == 1) {
								cmd_input.value = cmd.substr(0, cmd.lastIndexOf(" ") + 1) + current_arg + fill;
							} else {
								document.getElementById("command_overlay_suggestions").innerHTML = "";
								completions.sort(function(a,b) { return a.text.toLowerCase().localeCompare(b.text.toLowerCase()); });
								completions.forEach(displaySuggestions);
							}
						} else  {
							document.getElementById("command_overlay_suggestions").innerHTML = '<div class="command_overlay_suggestion"><i>no suggestions</i></div>';
						}
					} else  {
						document.getElementById("command_overlay_suggestions").innerHTML = '<div class="command_overlay_suggestion"><i>no suggestions</i></div>';
					}
				}
			}
		} 

		if (hide) {
			that.hide();
		}
	}

	this.undoClicked = function() {
		ActivityTracer.undo();
		that.hide();
	}

	this.redoClicked = function() {
		ActivityTracer.redo();
		that.hide();
	}

	this.copyClicked = function() {
		Tools.copy();
		that.hide();
	}

	this.pasteClicked = function() {
		Tools.paste();
		that.hide();
	}

	this.cutClicked = function() {
		Tools.cut();
		that.hide();
	}

	this.dfgClicked = function() {
		UI.Statemachine.toggleDataflow();
		that.hide();
	}

	this.terminalClicked = function() {
		UI.Panels.Terminal.show();
		that.hide();
	}

	this.saveClicked = function() {
		UI.Menu.saveBehaviorClicked();
		that.hide();
	}

	this.startRosCommand = function(cmd) {
		last_ros_command = cmd;
		that.tryExecuteCommand(cmd);
	}

	this.notifyRosCommand = function(cmd) {
		if (last_ros_command != undefined && last_ros_command.startsWith(cmd)) {
			RC.PubSub.sendRosNotification(last_ros_command);
			last_ros_command = undefined;
		}
	}

}) ();