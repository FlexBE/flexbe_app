UI.Panels.AddState = new (function() {
	var that = this;

	var statelib = [];

	var addHoverDetails = function(el, state_def) {
		var details = "<div style='margin-bottom: 0.5em;'>Package: <i>" + state_def.getStatePackage() + "</i></div>";
		var params = state_def.getParameters();
		if (params.length > 0) {
			details += "<div style='margin-bottom: 0.5em;'>Parameters:";
			params.forEach(param => {
				details += "<br />&nbsp;&nbsp;- " + param;
				var doc = state_def.getParamDesc().findElement(desc => { return desc.name == param; });
				if (doc != undefined) details += "&nbsp;&nbsp;<i>" + doc.type + "</i>";
			});
			details += "</div>";
		}
		var input_keys = state_def.getInputKeys().filter(key => !key.startsWith("$"));
		if (input_keys.length > 0) {
			details += "<div style='margin-bottom: 0.5em;'>Input Keys:";
			input_keys.forEach(key => {
				details += "<br />&nbsp;&nbsp;- " + key;
				var doc = state_def.getInputDesc().findElement(desc => { return desc.name == key; });
				if (doc != undefined) details += "&nbsp;&nbsp;<i>" + doc.type + "</i>";
			});
			details += "</div>";
		}
		var output_keys = state_def.getOutputKeys().filter(key => !key.startsWith("$"));
		if (output_keys.length > 0) {
			details += "<div style='margin-bottom: 0.5em;'>Output Keys:";
			output_keys.forEach(key => {
				details += "<br />&nbsp;&nbsp;- " + key;
				var doc = state_def.getOutputDesc().findElement(desc => { return desc.name == key; });
				if (doc != undefined) details += "&nbsp;&nbsp;<i>" + doc.type + "</i>";
			});
			details += "</div>";
		}
		var outcomes = state_def.getOutcomes().filter(outcome => !outcome.startsWith("$"));
		if (outcomes.length > 0) {
			details += "<div style='margin-bottom: 0em;'>Outcomes:";
			outcomes.forEach(outcome => {
				details += "<br />&nbsp;&nbsp;- " + outcome;
			});
			details += "</div>";
		}

		el.addEventListener('mouseover', function() {
			var rect = this.getBoundingClientRect();
			var tt = document.createElement("div");
			tt.setAttribute("style", "right: 370px; top: " + rect.top + "px; display: block;");
			tt.setAttribute("class", "sidepanel_tooltip");
			tt.setAttribute("id", "add_state_tooltip");
			tt.innerHTML = details;
			document.getElementsByTagName("body")[0].appendChild(tt);
			if (tt.getBoundingClientRect().bottom >= window.innerHeight - 5) {
				tt.setAttribute("style", "right: 370px; bottom: 5px; display: block;");
			}
		});
		el.addEventListener('mouseout', removeHover);
	}

	var removeHover = function() {
		var tt = document.getElementById("add_state_tooltip");
		if (tt != undefined) {
			tt.parentNode.removeChild(tt);
		}
	}

	var filterClassList = function() {
		removeHover();
		document.getElementById('panel_class_select').innerHTML = "";
		var filter_exp = document.getElementById("input_class_filter").value.toLowerCase();
		var filter_pkg = document.getElementById("input_package_filter").value;

		var filtered_lib = (filter_pkg == "ALL")?
			statelib :
			statelib.filter(function(element) {
				return WS.Statelib.getFromLib(element).getStatePackage() == filter_pkg;
			});

		var begin_list = filtered_lib.filter(function(element) {
			return element.toLowerCase().indexOf(filter_exp) == 0;
		});
		var contain_list = filtered_lib.filter(function(element) {
			return element.toLowerCase().indexOf(filter_exp) > 0;
		});

		displayStateClasses(begin_list);
		displayStateClasses(contain_list);

		if (begin_list.length + contain_list.length == 1) {
			var selected_element;
			if (begin_list.length == 1)
				selected_element = begin_list[0];
			else
				selected_element = contain_list[0];
			document.getElementById("add_state_class").value = selected_element;
		}
	};

	var displayStateClasses = function(class_list) {
		var panel_class_select = document.getElementById('panel_class_select');

		class_list.sort();

		for (var i=0; i<class_list.length; ++i) {
			state_def = WS.Statelib.getFromLib(class_list[i]);

			class_div = document.createElement("div");
			class_div.setAttribute("id", "class_select_" + state_def.getStateClass());
			class_div.setAttribute("class", "panel_class_select_class");
			class_div.setAttribute("value", state_def.getStateClass());
			class_div.innerHTML =
				  '<b>' + state_def.getStateClass() + '</b><br>'
				+ '<i>' + state_def.getShortDesc() + '</i>';

			class_div.addEventListener('click', function() {
				document.getElementById('add_state_class').value = this.getAttribute("value");
			});
			addHoverDetails(class_div, state_def);

			panel_class_select.appendChild(class_div);
		}
	};


	this.show = function() {
		panel_class_select.innerHTML = "";
		statelib = WS.Statelib.getClassList();
		displayStateClasses(statelib);
		UI.Panels.setActivePanel(UI.Panels.ADD_STATE_PANEL);
		UI.Settings.createStatePackageSelect(document.getElementById("input_package_filter"), true);
	}

	this.hide = function() {
		UI.Panels.hidePanelIfActive(UI.Panels.ADD_STATE_PANEL);
		document.getElementById("input_class_filter").value = "";
		document.activeElement.blur();
		removeHover();
	}

	this.addStateConfirmClicked = function() {
		var state_name = document.getElementById("add_state_name").value;
		var state_class = document.getElementById("add_state_class").value;
		if (state_name == "" || state_class == "") return;
		if (UI.Statemachine.getDisplayedSM().getStateByName(state_name) != undefined) {
			T.logWarn("State name already in use!");
			return;
		}

		var state_def = WS.Statelib.getFromLib(state_class);
		var new_state = new State(state_name, state_def);
		var sm = UI.Statemachine.getDisplayedSM();
		sm.addState(new_state);

		document.getElementById("add_state_name").value = "";
		document.getElementById("add_state_class").value = "";
		document.getElementById("input_class_filter").value = "";
		UI.Panels.AddState.filterChanged();

		UI.Statemachine.refreshView();
		UI.Panels.StateProperties.displayStateProperties(new_state);

		var state_path = new_state.getStatePath();
		var container_path = new_state.getContainer().getStatePath();

		ActivityTracer.addActivity(ActivityTracer.ACT_STATE_ADD,
			"Added new state " + state_name,
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
				var redo_state = new State(state_name, WS.Statelib.getFromLib(state_class));
				container.addState(redo_state);
				UI.Statemachine.refreshView();
			}
		);
	}

	this.filterChanged = function() {
		filterClassList();
	}

	this.addStateCancelClicked = function() {
		UI.Panels.AddState.hide();
	}

}) ();