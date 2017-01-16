UI.RuntimeControl = new (function() {
	var that = this;

	var R = undefined;
	var background = undefined;
	var previous_states = [];
	var current_states = [];
	var previous_state = undefined;
	var income_transition = undefined;
	var current_state = undefined;
	var current_level = 0;
	var next_states = [];
	var outcome_transitions = [];
	var drawings = [];
	var status_label = undefined;
	var outcome_request = { outcome: "", target: undefined };

	var param_keys = [];
	var param_vals = [];

	var locked_state_path = "";

	var pause_behavior_toggle = true;
	var sync_ext_toggle = false;

	var updateDrawing = function() {
		drawings.forEach(function(element, i) {
			element.drawing.remove();
		});
		drawings = [];

		if (current_state == undefined) return;

		// Path
		//------

		drawings.push(new Drawable.ContainerPath(current_state.getContainer(), R, smDisplayHandler));

		// Current
		//---------

		var is_locked = RC.Controller.isLocked() && RC.Controller.isOnLockedPath(current_state.getStatePath());
		var current_state_drawing = createDrawing(current_state, Drawable.State.Mode.MAPPING, true, is_locked);
		var bbox = current_state_drawing.drawing.getBBox()
		current_state_drawing.drawing.translate(-bbox.x + (R.width - bbox.width) / 2, -bbox.y + (R.height - bbox.height) / 2);
		drawings.push(current_state_drawing);

		// Previous
		//----------

		if (previous_state != undefined) {
			var previous_state_drawing = createDrawing(previous_state, Drawable.State.Mode.SIMPLE, false, false);
			var b = previous_state_drawing.drawing.getBBox();
			previous_state_drawing.drawing.translate(-b.x + 10, -b.y + (R.height - previous_state_drawing.drawing.getBBox().height) / 2);
			drawings.push(previous_state_drawing);
		}

		// Next
		//------

		var next_state_drawings = [];
		next_states.forEach(function(element, i) {
			next_state_drawings.push(createDrawing(element, Drawable.State.Mode.SIMPLE, false, false));
		});

		// vertical position
		var offset_top = 0;
		var space = (R.height - next_state_drawings.reduce(function(prev, cur, i, a) {
			return prev + cur.drawing.getBBox().height;
		}, 0)) / (next_state_drawings.length + 1);

		// horizontal position
		var offset_left = R.width - next_state_drawings.reduce(function(prev, cur, i, a) {
			return Math.max(prev, cur.drawing.getBBox().width);
		}, 0) - 10;

		next_state_drawings.forEach(function(element, i) {
			offset_top += space;
			var b = element.drawing.getBBox();
			element.drawing.translate(-b.x + offset_left, -b.y + offset_top);
			offset_top += b.height;
			drawings.push(element);
		});

		// Transitions
		//-------------

		if (previous_state != undefined)
			drawings.push(new Drawable.Transition(income_transition, R, true, drawings, false, false, Drawable.Transition.PATH_STRAIGHT));

		outcome_transitions.forEach(function(element, i) {
			var highlight = outcome_request.target != undefined && outcome_request.target == current_state.getStatePath() && outcome_request.outcome == element.getOutcome();
			if (highlight) drawStatusLabel("Onboard requested outcome: " + outcome_request.outcome);
			var transition = new Drawable.Transition(element, R, true, drawings.filter(function(d) { return d != previous_state_drawing; }), highlight, false, Drawable.Transition.PATH_CURVE);
			transition.drawing
				.attr({'cursor': 'pointer', 'title': "Click to force outcome " + element.getOutcome()})
				.data("transition", transition.obj)
				.click(function() { 
					if (RC.Sync.hasProcess("Transition")) return;
					var t = this.data("transition");
					RC.PubSub.sendOutcomeRequest(t.getFrom(), t.getOutcome());
					drawStatusLabel("Forcing outcome: " + t.getOutcome());
				});
			drawings.push(transition);
		});

		bgcolor = current_state.isInsideDifferentBehavior()? '#fff3f6' : '#fff';
		background.attr({fill: bgcolor}).toBack();
	}

	var drawStatusLabel = function(text) {
		if (status_label != undefined) status_label.remove();
		status_label = R.text(R.width / 2, R.height - 30, text)
			.attr({'font-size': 16, 'fill': 'gray'});
	}

	var createDrawing = function(state_obj, mode, active, locked) {
		if (state_obj instanceof Statemachine) {
			var drawable = new Drawable.Statemachine(state_obj, R, true, mode, active, locked);
			if (active) {
				drawable.drawing.data("sm_path", state_obj.getStatePath());
				drawable.drawing.dblclick(function() {
					var child_state = current_states[current_states.indexOf(current_state) + 1];
					current_state = child_state;
					updateStateDisplayDepth(child_state.getStatePath());
				});
			}
			return drawable;
		}

		if (state_obj instanceof BehaviorState) {
			var drawable = new Drawable.BehaviorState(state_obj, R, true, mode, active, locked);
			if (active) {
				drawable.drawing.data("new_path", state_obj.getStatePath());
				drawable.drawing.dblclick(function() {
					var child_state = current_states[current_states.indexOf(current_state) + 1];
					current_state = child_state;
					updateStateDisplayDepth(child_state.getStatePath());
				});
			}
			return drawable;
		}

		if (state_obj.getStateClass() == ":OUTCOME" || state_obj.getStateClass() == ":CONDITION")
			return new Drawable.Outcome(state_obj, R, true, false);

		return new Drawable.State(state_obj, R, true, mode, active, locked);
	}

	var smDisplayHandler = function() {
		var sm = this.data("statemachine");
		var current_relative_path = current_state.getStatePath().slice(sm.getStatePath().length + 1);
		var current_lower_name = current_relative_path.split("/")[0];
		var current_lower_path = sm.getStatePath() + "/" + current_lower_name;
		updateStateDisplayDepth(current_lower_path);
	}

	var updateStateDisplayDepth = function(state_path) {
		var path_segments = state_path.split("/");
		current_level = path_segments.length - 1;
		document.getElementById("selection_rc_lock_layer").selectedIndex = document.getElementById("selection_rc_lock_layer").length - current_level;

		updateStateDisplay();
	}

	var updateStateDisplay = function() {
		current_state = current_states[current_level];
		if (current_state == undefined) {
			updateDrawing();
			return;
		}

		previous_state = previous_states[current_level];
		if (previous_state != undefined) {
			income_transition = current_state.getContainer().getTransitions().findElement(function(element) {
				return element.getFrom().getStateName() == previous_state.getStateName() && element.getTo().getStateName() == current_state.getStateName();
			});
			if (income_transition == undefined) {
				previous_state = undefined;
			}
		}
		outcome_transitions = current_state.getContainer().getTransitions().filter(function(element) {
			return element.getFrom().getStateName() == current_state.getStateName();
		});
		next_states = [];
		outcome_transitions.forEach(function(element, i) {
			if (!next_states.contains(element.getTo()))
				next_states.push(element.getTo());
		});

		// documentation
		setDocumentation(current_state);

		updateDrawing();
	}

	var setDocumentation = function(state) {
		if (state == undefined) {
			document.getElementById("runtime_documentation_text").innerHTML = "";
			return;
		}

		var state_class = state.getStateClass();
		var doc = "";

		if (state instanceof Statemachine) {
			doc += "<b>Statemachine</b><br />";
		} else if (state instanceof BehaviorState) {
			doc += "<b>Behavior</b><br />";
		} else if (WS.Statelib.getFromLib(state_class) != undefined) {
			doc += "<b>" + state_class + "</b><br />";
			doc += WS.Statelib.getFromLib(state_class).getStateDesc() + "<br />";
			var pkeys = state.getParameters();
			var pvals = state.getParameterValues();
			if (pkeys.length > 0) {
				doc += "<br />";
				doc += "<b>Parameter Values:</b><br />";
			}
			for (var i = 0; i < pkeys.length; i++) {
				doc += "<div style='font-family: monospace;'><b>" + pkeys[i] + "</b> = ";
				doc += pvals[i];
				var resolved = VarSolver.resolveVar(pvals[i], false);
				if (resolved !== false && !(resolved instanceof Array) && resolved != pvals[i]) {
					doc += " (" + resolved + ")</div>";
				}
			}
		}

		document.getElementById("runtime_documentation_text").innerHTML = doc;
	}

	var resetStateDisplay = function() {
		current_level = 0;
		current_state = undefined
		current_states = [];
		previous_state = undefined;
		income_transition = undefined;
		next_states = [];
		outcome_transitions = [];
		setDocumentation(undefined);
	}

	var createParameterTable = function() {
		var table = document.getElementById("rc_parameter_table");
		table.innerHTML = "";

		var params = Behavior.getBehaviorParameters();
		var embedded_behaviors = helper_collectEmbeddedBehaviors(Behavior.getStatemachine());
		for (var i = 0; i < embedded_behaviors.length; i++) {
			if (embedded_behaviors[i] == undefined) continue;
			var b = embedded_behaviors[i];
			var ps = b.getBehaviorManifest().params.clone();
			params = params.concat(ps.filter(function(el) {
				return params.findElement(function(p) {
					return p.name == el.name;
				}) == undefined;
			}).map(function (el) {
				return {
					type: el.type,
					name: b.getStatePath().substr(1) + "/" + el.name,
					default: el.default,
					label: el.label,
					hint: el.hint,
					additional: el.additional
				};
			}));
		}

		if (params.length == 0) {
			table.innerHTML = "<i>The selected behavior supports no parameters.</i>";
			return;
		}

		for (var i=0; i<params.length; i++) {
			var name_td = document.createElement("td");
			name_td.setAttribute("width", "14%");
			name_td.setAttribute("height", "30");
			name_td.setAttribute("title", params[i].name);
			name_td.innerHTML = params[i].label + ":";

			var value_td = document.createElement("td");
			value_td.setAttribute("width", "60%");
			if (params[i].type == "enum") {
				var select = document.createElement("select");
				select.setAttribute("name", params[i].name);
				params[i].additional.forEach(function(opt) {
					select.innerHTML += '<option value="' + opt + '" ' + ((opt == params[i].default)? 'selected="selected"' : '') + '>' + opt + '</option>';
				});
				value_td.appendChild(select);
			} else if (params[i].type == "numeric") {
				var input = document.createElement("input");
				input.setAttribute("name", params[i].name);
				input.setAttribute("type", "number");
				input.setAttribute("value", params[i].default);
				input.setAttribute("min", params[i].additional.min);
				input.setAttribute("max", params[i].additional.max);
				input.setAttribute("step", (params[i].default.indexOf(".") != -1)? "0.1" : "1");
				value_td.appendChild(input);
			} else if (params[i].type == "boolean") {
				var input = document.createElement("input");
				input.setAttribute("name", params[i].name);
				input.setAttribute("type", "checkbox");
				if (params[i].default == "True") {
					input.setAttribute("checked", "checked");
				}
				value_td.appendChild(input);
			} else if (params[i].type == "text") {
				var input = document.createElement("input");
				input.setAttribute("name", params[i].name);
				input.setAttribute("type", "text");
				input.setAttribute("value", params[i].default);
				value_td.appendChild(input);
			} else if (params[i].type == "yaml") {
				var add_button = document.createElement("input");
				add_button.setAttribute("type", "button");
				add_button.setAttribute("value", "...");
				add_button.addEventListener('click', function() {
					var button = this;
					chrome.fileSystem.chooseEntry({type: 'openFile'}, function(entry) {
						button.parentNode.parentNode.children[0].children[0].setAttribute("value", entry.name);
					});
				});

				var file_input = document.createElement("input");
				file_input.setAttribute("type", "text");
				file_input.setAttribute("value", params[i].default);
				file_input.setAttribute("key", params[i].additional.key);
				file_input.setAttribute("style", "width: 300px");

				var file_td = document.createElement("td");
				file_td.appendChild(file_input);
				var add_td = document.createElement("td");
				add_td.appendChild(add_button);
				var key_label_td = document.createElement("td");
				key_label_td.innerHTML = (params[i].additional.key != "")? "(key: " + params[i].additional.key + ")" : "";

				var yaml_tr = document.createElement("tr");
				yaml_tr.appendChild(file_td);
				//yaml_tr.appendChild(add_td);
				yaml_tr.appendChild(key_label_td);
				var yaml_table = document.createElement("table");
				yaml_table.appendChild(yaml_tr);
				value_td.appendChild(yaml_table);
			}

			var hint_td = document.createElement("td");
			hint_td.setAttribute("width", "25%");
			var text = params[i].hint;
			if (params[i].name.indexOf("/") != -1) {
				text += " (/" + params[i].name.substr(0, params[i].name.lastIndexOf("/")) + ")";
			}
			hint_td.innerHTML = '<font style="color: gray">' + text + '</font>';

			var tr = document.createElement("tr");
			tr.setAttribute("name", params[i].name);
			tr.setAttribute("type", params[i].type);
			tr.appendChild(name_td);
			tr.appendChild(value_td);
			tr.appendChild(hint_td);
			table.appendChild(tr);
		}
	}

	var parseParameterConfig = function(callback) {
		var children = document.getElementById("rc_parameter_table").children;
		var tagsToGo = children.length;
		var result = [];

		var checkResult = function() {
			tagsToGo -= 1;
			if (tagsToGo == 0) callback(result);
		}

		for (var i = 0; i < children.length; i++) {
			var c = children[i];
			if (c.tagName != "TR") {
				checkResult();
				continue;
			}

			var name = c.getAttribute("name");
			var type = c.getAttribute("type");
			var value = "";
			if (type == "enum") {
				var select = c.children[1].children[0];
				value = select.options[select.selectedIndex].value;
				result.push({name: name, value: value});
				checkResult();
			} else if (type == "numeric") {
				value = c.children[1].children[0].value;
				result.push({name: name, value: value});
				checkResult();
			} else if (type == "boolean") {
				value = c.children[1].children[0].checked? "1" : "0";
				result.push({name: name, value: value});
				checkResult();
			} else if (type == "text") {
				value = c.children[1].children[0].value;
				result.push({name: name, value: value});
				checkResult();
			} else if (type == "yaml") {
				value = c.children[1].children[0].children[0].children[0].children[0].value;
				var key = c.children[1].children[0].children[0].children[0].children[0].getAttribute('key');
				result.push({name: "YAML:" + name, value: value + ":" + key});
				checkResult();
			}
		}
	}

	var initializeStateDisplay = function() {
		hideDisplays();
		R = Raphael("runtime_state_display");
		background = R.rect(0, 0, R.width, R.height)
			.attr({fill: '#FFF', stroke: '#FFF'}).toBack();
	}

	var hideDisplays = function() {
		document.getElementById("runtime_configuration_display").style.display = "none";
		document.getElementById("runtime_external_display").style.display = "none";
		document.getElementById("runtime_waiting_display").style.display = "none";
		document.getElementById("runtime_offline_display").style.display = "none";
		document.getElementById("runtime_no_behavior_display").style.display = "none";
		setDocumentation(undefined);
		if (R != undefined) {
			R.remove();
			R = undefined;
			drawings = [];
			status_label = undefined;
			that.displayLockBehavior();
		}
	}


	//
	//  Interface
	// ===========

	this.connectClicked = function() {
		if (!RC.ROS.isConnected() && !RC.ROS.isTrying()) {
			RC.ROS.trySetupConnection();
		}
	}

	this.resetRuntimeControl = function() {
		that.resetProgress();
	}

	this.recreateDrawingArea = function() {
		if (R != undefined) {
			R.remove();
			drawings = [];
			initializeStateDisplay();
			updateStateDisplay();
		}
	}

	this.refreshView = function() {
		if (R != undefined) {
			updateStateDisplay();
		}
	}

	this.setProgress = function(percent) {
		document.getElementById('progress_bar').style.transition = "all 0.5s linear";
		document.getElementById('progress_left').style.transition = "all 0.5s linear";
		document.getElementById('progress_bar').style.width = percent + "%";
		document.getElementById('progress_left').style.width = (100 - percent) + "%";
	}

	this.resetProgress = function() {
		document.getElementById('progress_bar').style.transition = "all 0s linear";
		document.getElementById('progress_left').style.transition = "all 0s linear";
		document.getElementById('progress_bar').style.width = "0%";
		document.getElementById('progress_left').style.width = "100%";
	}

	this.setProgressStatus = function(status) {
		var color = "";
		switch (status) {
			case RC.Sync.STATUS_WARN: color = "linear-gradient(#ff3, #dd2)"; break;
			case RC.Sync.STATUS_ERROR: color = "linear-gradient(#e86, #c64)"; break;
			default: color = "linear-gradient(#bf7, #9d5)"; break;
		}
		document.getElementById('progress_bar').style.background = color;
	}

	this.resetParameterTableClicked = function() {
		createParameterTable();
	}

	this.startBehaviorClicked = function() {
		console.log("Parsing parameter values...");
		parseParameterConfig(function (result) {
			console.log(result);
			console.log("Got parameter values, starting behavior...");
			param_keys = [];
			param_vals = [];
			result.forEach(function (r) {
				param_keys.push("/" + r.name);
				param_vals.push("" + r.value);
			});
			var selection_box = document.getElementById("selection_rc_autonomy");
			var autonomy_value = parseInt(selection_box.options[selection_box.selectedIndex].value);
			RC.PubSub.sendBehaviorStart(param_keys, param_vals, autonomy_value); 
		});
	}

	this.attachExternalClicked = function() {
		var selection_box = document.getElementById("selection_rc_autonomy");
		var autonomy_level = parseInt(selection_box.options[selection_box.selectedIndex].value);
		RC.PubSub.sendAttachBehavior(autonomy_level);

		UI.RuntimeControl.displayBehaviorFeedback(4, "Attaching to behavior...");
	}

	this.behaviorLockClicked = function() {
		if (!RC.Controller.isRunning()) return;

		if (RC.Controller.isActive()) {
			var selection_box = document.getElementById("selection_rc_lock_layer");
			locked_state_path = selection_box.options[selection_box.selectedIndex].getAttribute("path");
			RC.Controller.setLockedStatePath(locked_state_path);
			RC.PubSub.sendBehaviorLock(locked_state_path);
		} else if (RC.Controller.needSwitch()) {
			var selection_box = document.getElementById("selection_rc_autonomy");
			var autonomy_value = parseInt(selection_box.options[selection_box.selectedIndex].value);
			RC.PubSub.sendBehaviorUpdate(param_keys, param_vals, autonomy_value);
		} else {
			RC.PubSub.sendBehaviorUnlock(locked_state_path);
			RC.Sync.remove("Changes");
		}
	}

	this.updateAutonomySelectionBoxColor = function() {
		var selection_box = document.getElementById("selection_rc_autonomy");
		var value = parseInt(selection_box.options[selection_box.selectedIndex].value);
		switch (value) {
			case 0: selection_box.style.color = "black"; break;
			case 1: selection_box.style.color = "blue"; break;
			case 2: selection_box.style.color = "green"; break;
			case 3: selection_box.style.color = "red"; break;
		}
	}

	this.autonomySelectionChanged = function() {
		var selection_box = document.getElementById("selection_rc_autonomy");
		var value = parseInt(selection_box.options[selection_box.selectedIndex].value);
		this.updateAutonomySelectionBoxColor();
		if (RC.Controller.isConnected()) {
			RC.PubSub.sendAutonomyLevel(value);
		}
		//UI.RuntimeControl.displayBehaviorFeedback(4, "Changed autonomy to: " + ["No","Low","High","Full"][value]);
	}

	this.repeatBehaviorClicked = function() {
		if (!RC.Controller.isRunning()) return;

		RC.PubSub.sendRepeatBehavior();
	}

	this.pauseBehaviorClicked = function() {
		if (!RC.Controller.isRunning()) return;
		document.getElementById("button_behavior_pause").setAttribute("disabled", "disabled");

		if (pause_behavior_toggle) {
			RC.PubSub.sendPauseBehavior();
		} else {
			RC.PubSub.sendResumeBehavior();
		}
	}

	this.switchPauseButton = function() {
		pause_behavior_toggle = !pause_behavior_toggle;
		document.getElementById("button_behavior_pause").removeAttribute("disabled", "disabled");

		if (pause_behavior_toggle) {
			document.getElementById("button_behavior_pause").setAttribute("value", "Pause");
		} else {
			document.getElementById("button_behavior_pause").setAttribute("value", "Resume");
		}
	}

	this.resetPauseButton = function() {
		pause_behavior_toggle = true;
		document.getElementById("button_behavior_pause").removeAttribute("disabled", "disabled");
		document.getElementById("button_behavior_pause").setAttribute("value", "Pause");
	}

	this.preemptBehaviorClicked = function() {
		if (!RC.Controller.isConnected()) return;

		RC.PubSub.sendPreemptBehavior();
		UI.RuntimeControl.displayBehaviorFeedback(4, "Stopping behavior...");
		document.getElementById("cb_allow_preempt").checked = false;
		document.getElementById("button_behavior_preempt").setAttribute("disabled", "disabled");
		document.getElementById("button_behavior_preempt").style.color = "gray";
	}

	this.allowPreemptClicked = function(evt) {
		if(evt.target.checked) {
			document.getElementById("button_behavior_preempt").removeAttribute("disabled", "disabled");
			document.getElementById("button_behavior_preempt").style.color = "red";
		} else {
			document.getElementById("button_behavior_preempt").setAttribute("disabled", "disabled");
			document.getElementById("button_behavior_preempt").style.color = "gray";
		}
	}

	this.syncMirrorClicked = function() {
		if (!RC.Controller.isConnected()) return;

		RC.PubSub.sendSyncRequest();
		UI.RuntimeControl.displayBehaviorFeedback(4, "Requesting behavior sync...");
	}

	this.displaySyncExtension = function() {
		document.getElementById("sync_bar").style.marginBottom = "0";
		document.getElementById("sync_extension").style.display = "block";
		RC.Sync.setVisualizationCallback(function (sync_processes) {
			var processes = sync_processes.clone();
			if (processes.length == 0) {
				document.getElementById("sync_extension").innerHTML = '<div id="sync_empty" style="font-style: italic; color: gray;"> none active</div>';
				return;
			} else {
				var empt = document.getElementById("sync_empty");
				if (empt != undefined) document.getElementById("sync_extension").removeChild(empt);
			}
			var extension_divs = document.getElementById("sync_extension").childNodes;
			for (var i = 0; i < extension_divs.length; i++) {
				var d = extension_divs[i];
				var p = processes.findElement(function (el) {
					return el.key == d.getAttribute("key") && d.getAttribute("removing") == "false";
				});
				if (p == undefined) {
					if (d.getAttribute("removing") == "false") {
						d.setAttribute('removing', 'true');
						d.childNodes[1].style.color = 'gray';
						d.firstChild.firstChild.style.width = '100%';
						var success = d.firstChild.firstChild.style.backgroundColor == 'rgb(153, 221, 85)';
						d.firstChild.firstChild.style.opacity = '0.4';
						d.childNodes[1].innerText += success? ' - done!' : ' - failed!';
						var remove = function(el, timeout) {
							setTimeout(function() {
								document.getElementById("sync_extension").removeChild(el);
								document.getElementById("sync_extension").style.height = 
									Math.max(20, 20 * document.getElementById("sync_extension").childNodes.length) + "px";
							}, timeout);
						}
						remove(d, success? 2500 : 5000);
					}
					continue;
				}
				processes.remove(p);
				var color = (p.status == RC.Sync.STATUS_WARN)? '#dd2' :
							(p.status == RC.Sync.STATUS_ERROR)? '#c64' :
							'#9d5';
				d.firstChild.firstChild.style.backgroundColor = color;
				d.firstChild.firstChild.style.width = (p.fulfilled * 100) + '%';
				d.childNodes[1].innerText = p.key;
			}
			for (var i = 0; i < processes.length; i++) {
				var p = processes[i];
				var d = document.createElement("div");
				d.setAttribute('class', 'sync_entry');
				d.setAttribute('key', p.key);
				d.setAttribute('removing', 'false');
				d_content  = '<div class="sync_bar_border">';
				d_content += '<div class="sync_bar_content" style="width: ' + (p.fulfilled * 100) + '%; background-color: ' + color + ';"></div>';
				d_content += '</div><font>' + p.key + '</font>';
				d.innerHTML = d_content;
				document.getElementById("sync_extension").appendChild(d);
			}
			document.getElementById("sync_extension").style.height = Math.max(20, 20 * document.getElementById("sync_extension").childNodes.length) + "px";
		});
	}

	this.hideSyncExtension = function() {
		document.getElementById("sync_bar").style.marginBottom = "10px";
		document.getElementById("sync_extension").style.display = "none";
		RC.Sync.setVisualizationCallback(undefined);
	}

	this.toggleSyncExtension = function() {
		if (sync_ext_toggle) that.hideSyncExtension();
		else that.displaySyncExtension();
		sync_ext_toggle = !sync_ext_toggle;
	}

	this.displayBehaviorConfiguration = function() {
		hideDisplays();
		document.getElementById("runtime_configuration_display").style.display = "inline";
		createParameterTable();
	}

	this.displayWaitingForBehavior = function() {
		hideDisplays();
		document.getElementById("runtime_waiting_display").style.display = "inline";
	}

	this.displayExternalBehavior = function() {
		hideDisplays();
		document.getElementById("runtime_external_display").style.display = "inline";
	}

	this.displayEngineOffline = function() {
		hideDisplays();
		document.getElementById("runtime_offline_display").style.display = "inline";
	}

	this.displayNoBehavior = function() {
		hideDisplays();
		document.getElementById("runtime_no_behavior_display").style.display = "inline";
	}

	this.displayOutcomeRequest = function(outcome, target) {
		outcome_request.target = undefined;
		outcome_request.outcome = outcome;

		if (target != undefined) {
			outcome_request.target = target.getStatePath();
			drawStatusLabel("Onboard requested outcome: " + target.getStateName() + " > " + outcome);
			updateDrawing();
		}
	}

	this.displayLockBehavior = function() {
		var lock_button = document.getElementById("button_behavior_lock");
		lock_button.setAttribute("value", "Lock Behavior");

		var selection_box = document.createElement("select");
		selection_box.setAttribute("id", "selection_rc_lock_layer");
		if (R == undefined) {
			lock_button.setAttribute("disabled", "disabled");
			selection_box.innerHTML = '<option>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</option>';
			selection_box.setAttribute("disabled", "disabled");
		} else {
			lock_button.removeAttribute("disabled");
			// collect containers but skip top-level container (behavior)
			var options = [];
			for(var i=current_states.length-1; i>0; i--) {
				var option = document.createElement("option");
				if (i == current_level) {
					option.setAttribute("selected", "selected");
				}
				if (current_states[i] instanceof Statemachine && current_states[i].isConcurrent()) {
					options = [];
					option.setAttribute("selected", "selected");
				}
				option.setAttribute("path", current_states[i].getStatePath());
				var txt = current_states[i].getStateName();
				option.setAttribute("title", txt);
				option.text = ((txt.length > 18)? txt.slice(0,15) + "..." : txt);
				options.push(option);
			}
			options.forEach(function(option) {
				selection_box.add(option);
			});
		}

		var label_td = document.createElement("td");
		label_td.innerHTML = "At level: ";
		var selection_td = document.createElement("td");
		selection_td.appendChild(selection_box);

		var tr = document.getElementById("behavior_lock_display");
		tr.innerHTML = "";
		tr.appendChild(label_td);
		tr.appendChild(selection_td);
	}
	this.displayUnlockBehavior = function(have_changes) {
		var lock_button = document.getElementById("button_behavior_lock");
		lock_button.setAttribute("value", (have_changes)? "Go for it!" : "Unlock Behavior");
		lock_button.removeAttribute("disabled");

		var label_td = document.createElement("td");
		if (have_changes) {
			label_td.innerHTML = '<font style="color: gray">Do further changes or switch to the new version and continue its execution.</font>';
		} else {
			label_td.innerHTML = '<font style="color: gray">You may now change the behavior or unlock it to continue its execution.</font>';
		}

		var tr = document.getElementById("behavior_lock_display");
		tr.innerHTML = "";
		tr.appendChild(label_td);
	}
	this.displayBehaviorChanged = function() {
		var lock_button = document.getElementById("button_behavior_lock");
		lock_button.setAttribute("value", "Unlock Behavior");
		lock_button.setAttribute("disabled", "disabled");

		var button = document.createElement("input");
		button.setAttribute("id", "selection_rc_lock_layer");
		button.setAttribute("type", "button");
		button.setAttribute("value", "Reset Changes");
		button.setAttribute("style", "color: red; width: 70px; white-space: normal;");
		button.addEventListener("click", function() {
			ActivityTracer.resetToExecution();
			if (ActivityTracer.getLastSaveIndex() != ActivityTracer.getExecutionIndex()) {
				IO.BehaviorSaver.saveStateMachine();
				ActivityTracer.addSave();
			}
			RC.Controller.signalReset();
		});

		var label_td = document.createElement("td");
		label_td.innerHTML = '<font style="color: gray">Behavior changed, please save before unlock.</font>';
		var button_td = document.createElement("td");
		button_td.appendChild(button);

		var tr = document.getElementById("behavior_lock_display");
		tr.innerHTML = "";
		tr.appendChild(label_td);
		tr.appendChild(button_td);
	}

	this.displayState = function(state_path) {
		if (R == undefined) initializeStateDisplay();

		if (state_path == "") {
			resetStateDisplay();
			return;
		}

		// always update current states
		var path_segments = state_path.split("/");
		var path_recreate = "";
		for (var i=1; i<path_segments.length; i++) {
			path_recreate += "/" + path_segments[i];
			var new_current_state = Behavior.getStatemachine().getStateByPath(path_recreate);
			if (new_current_state != current_states[i]) {
				previous_states[i] = current_states[i];
			}
			current_states[i] = new_current_state;
		}
		current_states = current_states.slice(0, path_segments.length);

		// don't update display if it's only a child update
		if (current_state != undefined && current_level < path_segments.length 
			&& current_states[current_level].getStatePath() == current_state.getStatePath()) {

			if (!RC.Controller.isLocked()) {
				that.displayLockBehavior();
			}
			return;
		}

		current_level = path_segments.length - 1;

		if (!RC.Controller.isLocked()) {
			that.displayLockBehavior();
		}
		
		if (status_label != undefined) {
			status_label.remove();
			status_label = undefined;
		}

		updateStateDisplay();
	}

	this.displayBehaviorFeedback = function(level, text) {
		var color = "black";
		var collapse = UI.Settings.isCollapseInfo();
		switch(level) {
			case 1:
				color = "orange";
				collapse = UI.Settings.isCollapseWarn();
				break;
			case 2:
				color = "navy";
				collapse = UI.Settings.isCollapseHint();
				break;
			case 3:
				color = "red";
				collapse = UI.Settings.isCollapseError();
				break;
			case 4:
				color = "green";
				collapse = false;
				break;
		}
		var currentdate = new Date(); 
		var time = currentdate.toLocaleTimeString();

		var text_split = text.split("\n");
		var text_title = text_split[0];
		var text_body = "";
		for (var i = 1; i < text_split.length; i++) {
			text_body += "<br />&nbsp;&nbsp;&nbsp;&nbsp;";
			text_body += text_split[i].replace(/ /g, "&nbsp;").replace(/\t/g, "&nbsp;&nbsp;&nbsp;&nbsp;");
		}

		var panel = document.getElementById("runtime_feedback_text");

		var entry_time = document.createElement("font");
		entry_time.style.color = "gray";
		entry_time.innerHTML = "[" + time + "] ";

		var entry_title = document.createElement("font");
		entry_title.style.color = color;
		entry_title.style.fontWeight = level == 2? "bold" : "";
		entry_title.style.fontSize = "9pt";
		entry_title.innerHTML = text_title;

		var entry_body = undefined;
		var entry_toggle = undefined;
		if (text_body != "") {
			entry_body = document.createElement("font");
			entry_body.style.color = color;
			entry_body.style.opacity = "0.8";
			entry_body.innerHTML = text_body;
			entry_body.style.display = collapse? "none" : "";

			entry_toggle = document.createElement("font");
			entry_toggle.style.cursor = "pointer";
			entry_toggle.innerHTML = collapse? " [+]" : " [-]";
			entry_toggle.title = collapse? "show details" : "hide details";
			entry_toggle.addEventListener("click", function() {
				if (entry_toggle.innerHTML == " [-]") {
					entry_toggle.innerHTML = " [+]";
					entry_toggle.title = "show details";
					entry_body.style.display = "none";
				} else {
					entry_toggle.innerHTML = " [-]";
					entry_body.style.display = "";
					entry_toggle.title = "hide details";
					panel.scrollTop = entry_title.offsetTop - panel.offsetTop;
				}
			});
		}

		var entry = document.createElement("div");
		entry.style.whiteSpace = "nowrap";
		entry.appendChild(entry_time);
		entry.appendChild(entry_title);
		if (entry_body != undefined) {
			entry.appendChild(entry_toggle);
			entry.appendChild(entry_body);
		}

		var prev_entry = panel.lastChild
		if (prev_entry != undefined) {
			prev_entry.children[1].style.fontSize = "";
		}
		panel.appendChild(entry);

		panel.scrollTop = entry_title.offsetTop - panel.offsetTop;
	}

	var helper_collectEmbeddedBehaviors = function(sm) {
		if (sm == undefined) return;
		var states = sm.getStates();
		var result = [];
		for (var i = 0; i < states.length; i++) {
			if (states[i] instanceof Statemachine) {
				result = result.concat(helper_collectEmbeddedBehaviors(states[i]));
			}
			if (states[i] instanceof BehaviorState) {
				result.push(states[i]);
				result = result.concat(helper_collectEmbeddedBehaviors(states[i].getBehaviorStatemachine()));
			}
		}
		return result;
	}

}) ();