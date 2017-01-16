UI.Panels.AddState = new (function() {
	var that = this;

	var statelib = [];

	var filterClassList = function(filter_exp) {
		if (filter_exp == "") {
			displayStatelib();
			return;
		}

		var begin_list = statelib.filter(function(element) {
			return element.toLowerCase().indexOf(filter_exp) == 0;
		});
		var contain_list = statelib.filter(function(element) {
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

	var displayStatelib = function() {
		document.getElementById('panel_class_select').innerHTML = "";

		displayStateClasses(statelib);
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
				  '<b>' + state_def.getStateClass() + '</b><br />'
				+ '<i>' + state_def.getShortDesc() + '</i>';

			class_div.addEventListener('click', function() {
				document.getElementById('add_state_class').value = this.getAttribute("value");
			});

			panel_class_select.appendChild(class_div);
		}
	};


	this.show = function() {
		statelib = WS.Statelib.getClassList();
		displayStatelib();
		UI.Panels.setActivePanel(UI.Panels.ADD_STATE_PANEL);
	}

	this.hide = function() {
		UI.Panels.hidePanelIfActive(UI.Panels.ADD_STATE_PANEL);
		document.getElementById("input_class_filter").value = "";
		document.activeElement.blur();
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
		UI.Panels.AddState.classFilterChanged();

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

	this.classFilterChanged = function() {
		document.getElementById('panel_class_select').innerHTML = "";
		filterClassList(document.getElementById("input_class_filter").value.toLowerCase());
	}

	this.addStateCancelClicked = function() {
		UI.Panels.AddState.hide();
	}

}) ();