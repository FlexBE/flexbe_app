UI.Dashboard = new (function() {
	var that = this;

	var show_param_edit = false;

	//
	//  Private Configuration
	// =======================

	var _addPrivateVariable = function(new_key, new_value) {
		Behavior.getPrivateVariables().push({key: new_key, value: new_value});

		var tr = document.createElement("tr");

		var removeFunction = function(var_key) {
			tr.parentNode.removeChild(tr); 
			var element = Behavior.getPrivateVariables().findElement(function (element) { return element.key == var_key; });
			if (element != undefined) Behavior.getPrivateVariables().remove(element);
		};
		var addFunction = function(var_key, var_value) {
			document.getElementById("db_variable_table").appendChild(tr);
			Behavior.getPrivateVariables().push({key: var_key, value: var_value});
		};
		var changeKeyFunction = function(new_key, old_key, element) {
			var entry = Behavior.getPrivateVariables().findElement(function (el) { return el.key == old_key; });
			if (entry != undefined) entry.key = new_key;
			element.setAttribute("key", new_key);
			element.style.backgroundColor = Checking.isValidPythonVarname(new_key)? "initial" : "#fca";
			element.value = new_key;
		};
		var changeValueFunction = function(new_value, key, element) {
			var entry = Behavior.getPrivateVariables().findElement(function (el) { return el.key == key; });
			if (entry != undefined) entry.value = new_value;
			element.setAttribute("old_value", new_value);
			element.style.backgroundColor = Checking.isValidExpressionSyntax(new_value, true)? "initial" : "#fca";
			element.value = new_value;
		};

		var remove_button = document.createElement("img");
		remove_button.setAttribute("src", "img/table_row_delete.png");
		remove_button.setAttribute("title", "Remove this variable");
		remove_button.setAttribute("class", "img_button");
		remove_button.setAttribute("style", "margin-left: 10px;");
		remove_button.addEventListener("click", function() {
			var key = key_input_field.getAttribute("key");
			var value = value_input_field.getAttribute("old_value");
			removeFunction(key);

			ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_REMOVE,
				"Removed private variable " + key,
				function() { addFunction(key, value); },
				function() { removeFunction(key); }
			);
		});
		var key_input_field = document.createElement("input");
		key_input_field.setAttribute("value", new_key);
		key_input_field.setAttribute("key", new_key);
		key_input_field.setAttribute("class", "inline_text_edit");
		key_input_field.setAttribute("type", "text");
		key_input_field.style.backgroundColor = Checking.isValidPythonVarname(new_key)? "initial" : "#fca";
		key_input_field.addEventListener("blur", function() {
			var old_key = key_input_field.getAttribute("key");
			var new_key = key_input_field.value;
			if (old_key == new_key) return;
			changeKeyFunction(new_key, old_key, key_input_field);

			ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_CHANGE,
				"Renamed private variable " + old_key + " to " + new_key,
				function() { changeKeyFunction(old_key, new_key, key_input_field); },
				function() { changeKeyFunction(new_key, old_key, key_input_field); }
			);
		});
		var value_input_field = document.createElement("input");
		value_input_field.setAttribute("value", new_value);
		value_input_field.setAttribute("old_value", new_value);
		value_input_field.setAttribute("class", "inline_text_edit");
		value_input_field.setAttribute("type", "text");
		value_input_field.style.backgroundColor = Checking.isValidExpressionSyntax(new_value, true)? "initial" : "#fca";
		value_input_field.addEventListener("blur", function() {
			var key = key_input_field.getAttribute("key");
			var old_value = value_input_field.getAttribute("old_value");
			var new_value = value_input_field.value;
			if (old_value == new_value) return;
			changeValueFunction(new_value, key, value_input_field);

			ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_CHANGE,
				"Changed private variable " + key + " to " + new_value,
				function() { changeValueFunction(old_value, key, value_input_field); },
				function() { changeValueFunction(new_value, key, value_input_field); }
			);
		});

		var td_key_input_field = document.createElement("td");
		var td_label = document.createElement("td");
		var td_value_input_field = document.createElement("td");
		var td_remove_button = document.createElement("td");

		td_key_input_field.appendChild(key_input_field);
		td_label.innerHTML = "&nbsp;&nbsp;=&nbsp;";
		td_value_input_field.appendChild(value_input_field);
		td_value_input_field.setAttribute("width", "62%");
		td_remove_button.appendChild(remove_button);
		tr.appendChild(td_key_input_field);
		tr.appendChild(td_label);
		tr.appendChild(td_value_input_field);
		tr.appendChild(td_remove_button);
		document.getElementById("db_variable_table").appendChild(tr);

		ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_ADD,
			"Added private variable " + new_key + " = " + new_value,
			function() { removeFunction(new_key); },
			function() { addFunction(new_key, new_value); }
		);
	}


	//
	//  State Machine Userdata
	// ========================

	var _addDefaultUserdata = function(new_key, new_value) {
		Behavior.getDefaultUserdata().push({key: new_key, value: new_value});

		var tr = document.createElement("tr");

		var removeFunction = function(var_key) {
			tr.parentNode.removeChild(tr); 
			var element = Behavior.getDefaultUserdata().findElement(function (element) { return element.key == var_key; });
			if (element != undefined) Behavior.getDefaultUserdata().remove(element);
		};
		var addFunction = function(var_key, var_value) {
			document.getElementById("db_userdata_table").appendChild(tr);
			Behavior.getDefaultUserdata().push({key: var_key, value: var_value});
		};
		var changeKeyFunction = function(new_key, old_key, element) {
			var entry = Behavior.getDefaultUserdata().findElement(function (el) { return el.key == old_key; });
			if (entry != undefined) entry.key = new_key;
			element.setAttribute("key", new_key);
			element.style.backgroundColor = Checking.isValidPythonVarname(new_key)? "initial" : "#fca";
			element.value = new_key;
		};
		var changeValueFunction = function(new_value, key, element) {
			var entry = Behavior.getDefaultUserdata().findElement(function (el) { return el.key == key; });
			if (entry != undefined) entry.value = new_value;
			element.setAttribute("old_value", new_value);
			element.style.backgroundColor = Checking.isValidExpressionSyntax(new_value, true)? "initial" : "#fca";
			element.value = new_value;
		};

		var remove_button = document.createElement("img");
		remove_button.setAttribute("src", "img/table_row_delete.png");
		remove_button.setAttribute("title", "Remove this userdata");
		remove_button.setAttribute("class", "img_button");
		remove_button.setAttribute("style", "margin-left: 10px;");
		remove_button.addEventListener("click", function() {
			var key = key_input_field.getAttribute("key");
			var value = value_input_field.getAttribute("old_value");
			removeFunction(key);

			ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_REMOVE,
				"Removed userdata " + key,
				function() { addFunction(key, value); },
				function() { removeFunction(key); }
			);
		});
		var key_input_field = document.createElement("input");
		key_input_field.setAttribute("value", new_key);
		key_input_field.setAttribute("key", new_key);
		key_input_field.setAttribute("class", "inline_text_edit");
		key_input_field.setAttribute("type", "text");
		key_input_field.style.backgroundColor = Checking.isValidPythonVarname(new_key)? "initial" : "#fca";
		key_input_field.addEventListener("blur", function() {
			var old_key = key_input_field.getAttribute("key");
			var new_key = key_input_field.value;
			if (old_key == new_key) return;
			changeKeyFunction(new_key, old_key, key_input_field);

			ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_CHANGE,
				"Renamed userdata " + old_key + " to " + new_key,
				function() { changeKeyFunction(old_key, new_key, key_input_field); },
				function() { changeKeyFunction(new_key, old_key, key_input_field); }
			);
		});
		var value_input_field = document.createElement("input");
		value_input_field.setAttribute("value", new_value);
		value_input_field.setAttribute("old_value", new_value);
		value_input_field.setAttribute("class", "inline_text_edit");
		value_input_field.setAttribute("type", "text");
		value_input_field.style.backgroundColor = Checking.isValidExpressionSyntax(new_value, true)? "initial" : "#fca";
		value_input_field.addEventListener("blur", function() {
			var key = key_input_field.getAttribute("key");
			var old_value = value_input_field.getAttribute("old_value");
			var new_value = value_input_field.value;
			if (old_value == new_value) return;
			changeValueFunction(new_value, key, value_input_field);

			ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_CHANGE,
				"Changed userdata " + key + " to " + new_value,
				function() { changeValueFunction(old_value, key, value_input_field); },
				function() { changeValueFunction(new_value, key, value_input_field); }
			);
		});

		var td_key_input_field = document.createElement("td");
		var td_label = document.createElement("td");
		var td_value_input_field = document.createElement("td");
		var td_remove_button = document.createElement("td");

		td_key_input_field.appendChild(key_input_field);
		td_label.innerHTML = "&nbsp;&nbsp;=&nbsp;";
		td_value_input_field.appendChild(value_input_field);
		td_remove_button.appendChild(remove_button);
		tr.appendChild(td_key_input_field);
		tr.appendChild(td_label);
		tr.appendChild(td_value_input_field);
		tr.appendChild(td_remove_button);
		document.getElementById("db_userdata_table").appendChild(tr);

		ActivityTracer.addActivity(ActivityTracer.ACT_INTERNAL_CONFIG_ADD,
			"Added userdata " + new_key + " = " + new_value,
			function() { removeFunction(new_key); },
			function() { addFunction(new_key, new_value); }
		);
	}


	//
	//  Behavior Parameters
	// ===================

	var _addBehaviorParameter = function(new_type, new_name) {
		var daa = getDefaultAndAdditional(new_type);
		Behavior.getBehaviorParameters().push({
			type: new_type,
			name: new_name,
			default: daa.default,
			label: new_name,
			hint: "Sets the " + new_name,
			additional: daa.additional
		});

		var edit_button = document.createElement("img");
		edit_button.setAttribute("src", "img/pencil.png");
		edit_button.setAttribute("title", "Edit this parameter");
		edit_button.setAttribute("class", "img_button");
		edit_button.setAttribute("style", "margin-left: 10px;");
		edit_button.setAttribute("name", new_name);
		edit_button.addEventListener("click", function() {
			var name = this.getAttribute("name");
			
			createBehaviorParameterEdit(name);

			that.turnParameterClicked();
		});
		var remove_button = document.createElement("img");
		remove_button.setAttribute("src", "img/table_row_delete.png");
		remove_button.setAttribute("title", "Remove this parameter");
		remove_button.setAttribute("class", "img_button");
		remove_button.setAttribute("style", "margin-left: 10px;");
		remove_button.setAttribute("name", new_name);
		remove_button.addEventListener("click", function() {
			var name = this.getAttribute("name");

			Behavior.removeBehaviorParameter(name);
			
			var row = this.parentNode.parentNode;
			row.parentNode.removeChild(row);
		});
		var type_input_field = document.createElement("select");
		type_input_field.innerHTML = '<option value="enum"' + (new_type == "enum"? ' selected="selected"' : '') + '>Enum</option>' +
							'<option value="numeric"' + (new_type == "numeric"? ' selected="selected"' : '') + '>Numeric</option>' +
							'<option value="boolean"' + (new_type == "boolean"? ' selected="selected"' : '') + '>Boolean</option>' +
							'<option value="text"' + (new_type == "text"? ' selected="selected"' : '') + '>Text</option>' +
							'<option value="yaml"' + (new_type == "yaml"? ' selected="selected"' : '') + '>File</option>';
		type_input_field.setAttribute("name", new_name);
		type_input_field.setAttribute("class", "inline_text_edit");
		type_input_field.addEventListener("blur", function() {
			var name = this.getAttribute("name");
			var type = this.options[this.selectedIndex].value;
			var daa = getDefaultAndAdditional(type);
			Behavior.updateBehaviorParameter(name, type, "type");
			Behavior.updateBehaviorParameter(name, daa.default, "default");
			Behavior.updateBehaviorParameter(name, daa.additional, "additional");
		});
		var name_input_field = document.createElement("input");
		name_input_field.setAttribute("value", new_name);
		name_input_field.setAttribute("name", new_name);
		name_input_field.setAttribute("class", "inline_text_edit");
		name_input_field.setAttribute("type", "text");
		name_input_field.addEventListener("blur", function() {
			var name = this.getAttribute("name");
			var entry = Behavior.getBehaviorParameters().findElement(function(element) {
				return element.name == name;
			});
			
			if (entry != undefined) {
				Behavior.updateBehaviorParameter(name, this.value, "name");
				if (entry.label == name) Behavior.updateBehaviorParameter(this.value, this.value, "label");
				if (entry.hint == "Sets the " + name) Behavior.updateBehaviorParameter(this.value, "Sets the " + this.value, "hint");
			}

			this.setAttribute("name", this.value);
			this.parentNode.parentNode.children[0].children[0].setAttribute("name", this.value);
			this.parentNode.parentNode.children[2].children[0].setAttribute("name", this.value);
			this.parentNode.parentNode.children[3].children[0].setAttribute("name", this.value);
		});

		var td_type_input_field = document.createElement("td");
		var td_name_input_field = document.createElement("td");
		var td_edit_button = document.createElement("td");
		var td_remove_button = document.createElement("td");
		var tr = document.createElement("tr");

		td_type_input_field.appendChild(type_input_field);
		td_name_input_field.appendChild(name_input_field);
		td_edit_button.appendChild(edit_button);
		td_remove_button.appendChild(remove_button);
		tr.appendChild(td_type_input_field);
		tr.appendChild(td_name_input_field); // adapt changes on backside name change event
		tr.appendChild(td_edit_button);
		tr.appendChild(td_remove_button);
		document.getElementById("db_parameter_table").appendChild(tr);
	}


	//
	//  Private Functions
	// ===================

	var _addPrivateFunction = function(new_name, new_params) {
		Behavior.getPrivateFunctions().push({name: new_name, params: new_params});

		var name_input_field = document.createElement("input");
		name_input_field.setAttribute("value", new_name);
		name_input_field.setAttribute("name", new_name);
		name_input_field.setAttribute("class", "inline_text_edit_readonly");
		name_input_field.setAttribute("type", "text");
		name_input_field.setAttribute("readonly", "readonly");

		var params_input_field = document.createElement("input");
		params_input_field.setAttribute("value", new_params);
		params_input_field.setAttribute("name", new_name);
		params_input_field.setAttribute("class", "inline_text_edit_readonly");
		params_input_field.setAttribute("type", "text");
		params_input_field.setAttribute("readonly", "readonly");

		var td_name_input_field = document.createElement("td");
		var td_parentheses_left = document.createElement("td");
		var td_params_input_field = document.createElement("td");
		var td_parentheses_right = document.createElement("td");
		var tr = document.createElement("tr");
		tr.setAttribute("id", new_name);

		td_name_input_field.appendChild(name_input_field);
		td_parentheses_left.innerHTML = "&nbsp;&nbsp;(";
		td_params_input_field.appendChild(params_input_field);
		td_parentheses_right.innerHTML = "&nbsp;)";
		tr.appendChild(td_name_input_field);
		tr.appendChild(td_parentheses_left);
		tr.appendChild(td_params_input_field);
		tr.appendChild(td_parentheses_right);
		document.getElementById("db_function_table").appendChild(tr);
	}


	//
	//	State Machine Interface
	// =========================

	var _addBehaviorOutcome = function(new_outcome) {
		Behavior.addInterfaceOutcome(new_outcome);

		var tr = document.createElement("tr");

		var removeFunction = function(outcome) {
			tr.parentNode.removeChild(tr); 
			Behavior.removeInterfaceOutcome(outcome);
			if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
		};
		var addFunction = function(outcome) {
			document.getElementById("db_outcome_table").appendChild(tr);
			Behavior.addInterfaceOutcome(outcome);
			if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
		};

		var remove_button = document.createElement("img");
		remove_button.setAttribute("src", "img/table_row_delete.png");
		remove_button.setAttribute("title", "Remove from outcomes");
		remove_button.setAttribute("class", "img_button");
		remove_button.setAttribute("style", "margin-left: 10px;");
		remove_button.addEventListener("click", function() {
			var outcome = input_field.getAttribute("old_value");
			removeFunction(outcome);

			ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
				"Removed behavior outcome " + outcome,
				function() { addFunction(outcome); },
				function() { removeFunction(outcome); }
			);
		});
		var input_field = document.createElement("input");
		input_field.setAttribute("value", new_outcome);
		input_field.setAttribute("old_value", new_outcome);
		input_field.setAttribute("class", "inline_text_edit");
		input_field.setAttribute("type", "text");
		input_field.addEventListener("blur", function() {
			var new_value = input_field.value;
			var old_value = input_field.getAttribute("old_value");
			Behavior.updateInterfaceOutcome(old_value, new_value);
			input_field.setAttribute("old_value", new_value);

			ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
				"Renamed behavior outcome " + old_value + " to " + new_value,
				function() { 
					Behavior.updateInterfaceOutcome(new_value, old_value);
					input_field.setAttribute("old_value", old_value);
					input_field.value = old_value;
					if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
				},
				function() {
					Behavior.updateInterfaceOutcome(old_value, new_value);
					input_field.setAttribute("old_value", new_value);
					input_field.value = new_value;
					if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
				}
			);
		});

		var td_input_field = document.createElement("td");
		var td_remove_button = document.createElement("td");

		td_input_field.appendChild(input_field);
		td_remove_button.appendChild(remove_button);
		tr.appendChild(td_input_field);
		tr.appendChild(td_remove_button);
		document.getElementById("db_outcome_table").appendChild(tr);

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			"Added behavior outcome " + new_outcome,
			function() { removeFunction(new_outcome); },
			function() { addFunction(new_outcome); }
		);
	}

	var _addBehaviorInputKey = function(new_key) {
		Behavior.addInterfaceInputKey(new_key);

		var tr = document.createElement("tr");

		var removeFunction = function(key) {
			tr.parentNode.removeChild(tr); 
			Behavior.removeInterfaceInputKey(key);
		};
		var addFunction = function(key) {
			document.getElementById("db_input_key_table").appendChild(tr);
			Behavior.addInterfaceInputKey(key);
		};

		var remove_button = document.createElement("img");
		remove_button.setAttribute("src", "img/table_row_delete.png");
		remove_button.setAttribute("title", "Remove from input keys");
		remove_button.setAttribute("class", "img_button");
		remove_button.setAttribute("style", "margin-left: 10px;");
		remove_button.addEventListener("click", function() {
			var key = input_field.getAttribute("old_value");
			removeFunction(key);

			ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
				"Removed behavior input key " + key,
				function() { addFunction(key); },
				function() { removeFunction(key); }
			);
		});
		var input_field = document.createElement("input");
		input_field.setAttribute("value", new_key);
		input_field.setAttribute("old_value", new_key);
		input_field.setAttribute("class", "inline_text_edit");
		input_field.setAttribute("type", "text");
		input_field.addEventListener("blur", function() {
			var new_value = input_field.value;
			var old_value = input_field.getAttribute("old_value");
			Behavior.updateInterfaceInputKeys(old_value, new_value);
			input_field.setAttribute("old_value", new_value);

			ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
				"Renamed behavior input key " + old_value + " to " + new_value,
				function() { 
					Behavior.updateInterfaceInputKeys(new_value, old_value);
					input_field.setAttribute("old_value", old_value);
					input_field.value = old_value;
				},
				function() {
					Behavior.updateInterfaceInputKeys(old_value, new_value);
					input_field.setAttribute("old_value", new_value);
					input_field.value = new_value;
				}
			);
		});

		var td_input_field = document.createElement("td");
		var td_remove_button = document.createElement("td");

		td_input_field.appendChild(input_field);
		td_remove_button.appendChild(remove_button);
		tr.appendChild(td_input_field);
		tr.appendChild(td_remove_button);
		document.getElementById("db_input_key_table").appendChild(tr);

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			"Added behavior input key " + new_key,
			function() { removeFunction(new_key); },
			function() { addFunction(new_key); }
		);
	}

	var _addBehaviorOutputKey = function(new_key) {
		Behavior.addInterfaceOutputKey(new_key);

		var tr = document.createElement("tr");

		var removeFunction = function(key) {
			tr.parentNode.removeChild(tr); 
			Behavior.removeInterfaceOutputKey(key);
			if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
		};
		var addFunction = function(key) {
			document.getElementById("db_output_key_table").appendChild(tr);
			Behavior.addInterfaceOutputKey(key);
			if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
		};

		var remove_button = document.createElement("img");
		remove_button.setAttribute("src", "img/table_row_delete.png");
		remove_button.setAttribute("title", "Remove from output keys");
		remove_button.setAttribute("class", "img_button");
		remove_button.setAttribute("style", "margin-left: 10px;");
		remove_button.addEventListener("click", function() {
			var key = input_field.getAttribute("old_value");
			removeFunction(key);

			ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
				"Removed behavior output key " + key,
				function() { addFunction(key); },
				function() { removeFunction(key); }
			);
		});
		var input_field = document.createElement("input");
		input_field.setAttribute("value", new_key);
		input_field.setAttribute("old_value", new_key);
		input_field.setAttribute("class", "inline_text_edit");
		input_field.setAttribute("type", "text");
		input_field.addEventListener("blur", function() {
			var new_value = input_field.value;
			var old_value = input_field.getAttribute("old_value");
			Behavior.updateInterfaceOutputKeys(old_value, new_value);
			input_field.setAttribute("old_value", new_value);

			ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
				"Renamed behavior output key " + old_value + " to " + new_value,
				function() { 
					Behavior.updateInterfaceOutputKeys(new_value, old_value);
					input_field.setAttribute("old_value", old_value);
					input_field.value = old_value;
					if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
				},
				function() {
					Behavior.updateInterfaceOutputKeys(old_value, new_value);
					input_field.setAttribute("old_value", new_value);
					input_field.value = new_value;
					if (UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();
				}
			);
		});

		var td_input_field = document.createElement("td");
		var td_remove_button = document.createElement("td");

		td_input_field.appendChild(input_field);
		td_remove_button.appendChild(remove_button);
		tr.appendChild(td_input_field);
		tr.appendChild(td_remove_button);
		document.getElementById("db_output_key_table").appendChild(tr);

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			"Added behavior output key " + new_key,
			function() { removeFunction(new_key); },
			function() { addFunction(new_key); }
		);
	}


	var createBehaviorParameterEdit = function(param_name) {
		var param = Behavior.getBehaviorParameters().findElement(function(element) {
			return element.name == param_name;
		});

		var type_input_field = document.createElement("select");
		type_input_field.innerHTML = '<option value="enum"' + (param.type == "enum"? ' selected="selected"' : '') + '>Enum</option>' +
							'<option value="numeric"' + (param.type == "numeric"? ' selected="selected"' : '') + '>Numeric</option>' +
							'<option value="boolean"' + (param.type == "boolean"? ' selected="selected"' : '') + '>Boolean</option>' +
							'<option value="text"' + (param.type == "text"? ' selected="selected"' : '') + '>Text</option>' +
							'<option value="yaml"' + (param.type == "yaml"? ' selected="selected"' : '') + '>File</option>';
		type_input_field.setAttribute("name", param_name);
		type_input_field.setAttribute("class", "input_field");
		type_input_field.addEventListener("change", function() {
			var name = this.getAttribute("name");
			var type = this.options[this.selectedIndex].value;
			var daa = getDefaultAndAdditional(type);
			Behavior.updateBehaviorParameter(name, type, "type");
			Behavior.updateBehaviorParameter(name, daa.default, "default");
			Behavior.updateBehaviorParameter(name, daa.additional, "additional");

			// update input fields
			this.parentNode.parentNode.parentNode.children[3].children[2].children[0].value = daa.default;

			var param_trs = document.getElementById("db_parameter_table").children;
			for (var i = 0; i < param_trs.length; i++) {
				var param_type_input = param_trs[i].children[0].children[0];
				if (param_type_input.name == name) {
					param_type_input.innerHTML = this.innerHTML.replace(' selected="selected"', '');
					for (var i = 0; i < param_type_input.children.length; i++) {
						var opt = param_type_input.children[i];
						if (opt.value == type) {
							opt.setAttribute("selected", "selected");
							break;
						}
					}
					break;
				}
			}

			var additional_td = this.parentNode.parentNode.parentNode.children[5].children[0];
			additional_td.innerHTML = "";
			additional_td.appendChild(createParameterAdditionalEdit(name));
		});
		var label_label = document.createElement("label");
		label_label.innerHTML = "Label: ";
		var label_input_field = document.createElement("input");
		label_input_field.setAttribute("value", param.label);
		label_input_field.setAttribute("name", param_name);
		label_input_field.setAttribute("class", "input_field");
		label_input_field.setAttribute("type", "text");
		label_input_field.addEventListener("blur", function() {
			var name = this.getAttribute("name");
			Behavior.updateBehaviorParameter(name, this.value, "label");
		});

		var hint_label = document.createElement("label");
		hint_label.innerHTML = "Advice for the operator: ";
		var hint_input_field = document.createElement("input");
		hint_input_field.setAttribute("value", param.hint);
		hint_input_field.setAttribute("name", param_name);
		hint_input_field.setAttribute("class", "input_field");
		hint_input_field.setAttribute("type", "text");
		hint_input_field.addEventListener("blur", function() {
			var name = this.getAttribute("name");
			Behavior.updateBehaviorParameter(name, this.value, "hint");
		});

		var name_input_field = document.createElement("input");
		name_input_field.setAttribute("value", param_name);
		name_input_field.setAttribute("name", param_name);
		name_input_field.setAttribute("class", "inline_text_edit");
		name_input_field.setAttribute("type", "text");
		name_input_field.addEventListener("blur", function() {
			var name = this.getAttribute("name");
			var entry = Behavior.getBehaviorParameters().findElement(function(element) {
				return element.name == name;
			});

			this.setAttribute("name", this.value);
			this.parentNode.parentNode.children[2].children[0].setAttribute("name", this.value);

			this.parentNode.parentNode.parentNode.children[0].children[0].children[0].setAttribute("name", this.value);
			this.parentNode.parentNode.parentNode.children[0].children[2].children[0].setAttribute("name", this.value);

			this.parentNode.parentNode.parentNode.children[1].children[1].children[0].setAttribute("name", this.value);

			Behavior.updateBehaviorParameter(name, this.value, "name");
			// add renaming of label and hint
			if (entry.label == name) {
				Behavior.updateBehaviorParameter(this.value, this.value, "label");
				this.parentNode.parentNode.parentNode.children[0].children[2].children[0].value = this.value;
			}
			if (entry.hint == "Sets the " + name) {
				Behavior.updateBehaviorParameter(this.value, "Sets the " + this.value, "hint");
				this.parentNode.parentNode.parentNode.children[1].children[1].children[0].value = "Sets the " + this.value;
			}

			var param_trs = document.getElementById("db_parameter_table").children;
			for (var i = 0; i < param_trs.length; i++) {
				var param_name_input = param_trs[i].children[1].children[0];
				if (param_name_input.name == name) {
					param_name_input.value = this.value;
					param_name_input.focus();
					param_name_input.blur();
					break;
				}
			}
		});
		var value_input_field = document.createElement("input");
		value_input_field.setAttribute("value", param.default);
		value_input_field.setAttribute("name", param_name);
		value_input_field.setAttribute("class", "inline_text_edit");
		value_input_field.setAttribute("type", "text");
		value_input_field.addEventListener("blur", function() {
			var name = this.getAttribute("name");
			var entry = Behavior.getBehaviorParameters().findElement(function(element) {
				return element.name == name;
			});
			if (entry.type == "boolean") {
				if (this.value.match(/^(true|false)$/i) == undefined) {
					this.value = entry.default;
				}
			} else if (entry.type == "numeric") {
				if (this.value.match(/^-?[0-9]+(\.[0-9]+)?$/i) == undefined) {
					this.value = entry.default;
				}
				if (parseFloat(this.value) < parseFloat(entry.additional.min)) {
					this.value = entry.additional.min;
				}
				if (parseFloat(this.value) > parseFloat(entry.additional.max)) {
					this.value = entry.additional.max;
				}
			} else if (entry.type == "enum") {
				if (!param.additional.contains(this.value)) {
					this.value = entry.default;
				}
			}

			Behavior.updateBehaviorParameter(name, this.value, "default");
		});

		document.getElementById("db_parameter_edit_table").innerHTML = "";
		var type_td = document.createElement("td");
		var label_td = document.createElement("td");
		var label_input_td = document.createElement("td");
		var hint_td = document.createElement("td");
		var hint_input_td = document.createElement("td");
		hint_input_td.setAttribute("colspan", "2");
		var name_td = document.createElement("td");
		var eq_td = document.createElement("td");
		var value_td = document.createElement("td");
		var add_td = document.createElement("td");
		add_td.setAttribute("colspan", "3");

		type_td.appendChild(type_input_field);
		label_td.appendChild(label_label);
		label_input_td.appendChild(label_input_field);
		name_td.appendChild(name_input_field);
		eq_td.innerHTML = "&nbsp;&nbsp;=&nbsp;";
		eq_td.setAttribute("style", "text-align: center");
		value_td.appendChild(value_input_field);
		hint_td.appendChild(hint_label);
		hint_input_td.appendChild(hint_input_field);
		add_td.appendChild(createParameterAdditionalEdit(param_name));

		var top_tr = document.createElement("tr");
		var mid_tr = document.createElement("tr");
		var sep_above_tr = document.createElement("tr");
		sep_above_tr.setAttribute("style", "height: 10px");
		var bot_tr = document.createElement("tr");
		var sep_below_tr = document.createElement("tr");
		sep_below_tr.setAttribute("style", "height: 10px");
		var add_tr = document.createElement("tr");

		top_tr.appendChild(type_td);
		top_tr.appendChild(label_td);
		top_tr.appendChild(label_input_td);
		mid_tr.appendChild(hint_td);
		mid_tr.appendChild(hint_input_td);
		bot_tr.appendChild(name_td);
		bot_tr.appendChild(eq_td);
		bot_tr.appendChild(value_td); // adapt changes on additional param events
		add_tr.appendChild(add_td);

		document.getElementById("db_parameter_edit_table").appendChild(top_tr);
		document.getElementById("db_parameter_edit_table").appendChild(mid_tr);
		document.getElementById("db_parameter_edit_table").appendChild(sep_above_tr);
		document.getElementById("db_parameter_edit_table").appendChild(bot_tr);
		document.getElementById("db_parameter_edit_table").appendChild(sep_below_tr);
		document.getElementById("db_parameter_edit_table").appendChild(add_tr);
	}

	var createParameterAdditionalEdit = function(param_name) {
		var param = Behavior.getBehaviorParameters().findElement(function(element) {
			return element.name == param_name;
		});
		var type = param.type;

		var tr = document.createElement("tr");

		if (type == "enum") {
			var add_input = document.createElement("input");
			add_input.setAttribute("class", "input_field");
			add_input.setAttribute("type", "text");
			var add_button = document.createElement("input");
			add_button.setAttribute("value", "Add");
			add_button.setAttribute("name", param_name);
			add_button.setAttribute("type", "button");
			add_button.addEventListener("click", function() {
				var name = this.getAttribute("name");
				var to_add = this.parentNode.parentNode.children[1].children[0].value;
				this.parentNode.parentNode.children[1].children[0].value = "";
				var additional = Behavior.getBehaviorParameters().findElement(function(element) {
					return element.name == name;
				}).additional;
				if (additional.indexOf(to_add) != -1 || to_add == "") return;
				additional.push(to_add);
				Behavior.updateBehaviorParameter(name, additional, "additional");

				// update remove list
				var select = this.parentNode.parentNode.children[4].children[0];
				select.innerHTML = '';
				for (var i = 0; i < additional.length; i++) {
					select.innerHTML += '<option value="' + additional[i] + '">' + additional[i] + '</option>';
				};

				// update default value
				var default_field = this.parentNode.parentNode.parentNode.parentNode.parentNode.parentNode.children[3].children[2].children[0];
				if (additional.length == 1) {
					default_field.value = to_add;
					Behavior.updateBehaviorParameter(name, to_add, "default");
				}
			});

			var remove_input = document.createElement("select");
			remove_input.innerHTML = '';
			for (var i = 0; i < param.additional.length; i++) {
				remove_input.innerHTML += '<option value="' + param.additional[i] + '">' + param.additional[i] + '</option>';
			};
			remove_input.setAttribute("class", "input_field");
			remove_input.setAttribute("style", "min-width: 80px");
			remove_input.setAttribute("type", "text");
			var remove_button = document.createElement("input");
			remove_button.setAttribute("value", "Remove");
			remove_button.setAttribute("name", param_name);
			remove_button.setAttribute("type", "button");
			remove_button.addEventListener("click", function() {
				var name = this.getAttribute("name");
				var select = this.parentNode.parentNode.children[4].children[0];
				var to_remove = select.options[select.selectedIndex].value;
				if (to_remove == "") return;
				var additional = Behavior.getBehaviorParameters().findElement(function(element) {
					return element.name == name;
				}).additional;
				additional.remove(to_remove);
				Behavior.updateBehaviorParameter(name, additional, "additional");

				// update remove list
				select.innerHTML = '';
				for (var i = 0; i < additional.length; i++) {
					select.innerHTML += '<option value="' + additional[i] + '">' + additional[i] + '</option>';
				};

				// update default value
				var default_field = this.parentNode.parentNode.parentNode.parentNode.parentNode.parentNode.children[3].children[2].children[0];
				if (additional.length == 0) {
					default_field.value = "";
					Behavior.updateBehaviorParameter(name, "", "default");
				} else if (to_remove == default_field.value) {
					default_field.value = additional[0];
					Behavior.updateBehaviorParameter(name, additional[0], "default");
				}
			});

			var label_td = document.createElement("td");
			label_td.innerHTML = "Options:&nbsp;&nbsp;&nbsp;";
			var add_input_td = document.createElement("td");
			add_input_td.appendChild(add_input);
			var add_button_td = document.createElement("td");
			add_button_td.appendChild(add_button);
			var spacer_td = document.createElement("td");
			spacer_td.innerHTML = "&nbsp;&nbsp;&nbsp;";
			var remove_input_td = document.createElement("td");
			remove_input_td.appendChild(remove_input);
			var remove_button_td = document.createElement("td");
			remove_button_td.appendChild(remove_button);

			tr.appendChild(label_td);
			tr.appendChild(add_input_td);
			tr.appendChild(add_button_td);
			tr.appendChild(spacer_td);
			tr.appendChild(remove_input_td);
			tr.appendChild(remove_button_td);
		} else if (type == "numeric") {
			var min_input = document.createElement("input");
			min_input.setAttribute("value", param.additional.min);
			min_input.setAttribute("name", param_name);
			min_input.setAttribute("class", "input_field");
			min_input.setAttribute("type", "text");
			min_input.addEventListener("blur", function() {
				if (this.value.match(/^-?[0-9]+(\.[0-9]+)?$/i) == undefined) return;
				var name = this.getAttribute("name");
				var additional = Behavior.getBehaviorParameters().findElement(function(element) {
					return element.name == name;
				}).additional;
				if (parseFloat(this.value) > parseFloat(additional.max)) this.value = additional.max;
				additional.min = this.value;
				Behavior.updateBehaviorParameter(name, additional, "additional");

				// update default value
				var default_field = this.parentNode.parentNode.parentNode.parentNode.parentNode.parentNode.children[3].children[2].children[0];
				if (parseFloat(default_field.value) < parseFloat(this.value)) {
					default_field.value = this.value;
					Behavior.updateBehaviorParameter(name, this.value, "default");
				}
			});
			var max_input = document.createElement("input");
			max_input.setAttribute("value", param.additional.max);
			max_input.setAttribute("name", param_name);
			max_input.setAttribute("class", "input_field");
			max_input.setAttribute("type", "text");
			max_input.addEventListener("blur", function() {
				if (this.value.match(/^-?[0-9]+(\.[0-9]+)?$/i) == undefined) return;
				var name = this.getAttribute("name");
				var additional = Behavior.getBehaviorParameters().findElement(function(element) {
					return element.name == name;
				}).additional;
				if (parseFloat(this.value) < parseFloat(additional.min)) this.value = additional.min;
				additional.max = this.value;
				Behavior.updateBehaviorParameter(name, additional, "additional");

				// update default value
				var default_field = this.parentNode.parentNode.parentNode.parentNode.parentNode.parentNode.children[3].children[2].children[0];
				if (parseFloat(default_field.value) > parseFloat(this.value)) {
					default_field.value = this.value;
					Behavior.updateBehaviorParameter(name, this.value, "default");
				}
			});

			var min_label_td = document.createElement("td");
			min_label_td.innerHTML = "Minimum:&nbsp;&nbsp;&nbsp;";
			var min_input_td = document.createElement("td");
			min_input_td.appendChild(min_input);
			var max_label_td = document.createElement("td");
			max_label_td.innerHTML = "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Maximum:&nbsp;&nbsp;&nbsp;";
			var max_input_td = document.createElement("td");
			max_input_td.appendChild(max_input);

			tr.appendChild(min_label_td);
			tr.appendChild(min_input_td);
			tr.appendChild(max_label_td);
			tr.appendChild(max_input_td);
		} else if (type == "yaml") {
			var key_input = document.createElement("input");
			key_input.setAttribute("value", param.additional.key);
			key_input.setAttribute("name", param_name);
			key_input.setAttribute("class", "input_field");
			key_input.setAttribute("type", "text");
			key_input.addEventListener("blur", function() {
				var name = this.getAttribute("name");
				var additional = Behavior.getBehaviorParameters().findElement(function(element) {
					return element.name == name;
				}).additional;
				additional.key = this.value;
				Behavior.updateBehaviorParameter(name, additional, "additional");
			});

			var key_label_td = document.createElement("td");
			key_label_td.innerHTML = "Key to look up in file:&nbsp;&nbsp;&nbsp;";
			var key_input_td = document.createElement("td");
			key_input_td.appendChild(key_input);

			tr.appendChild(key_label_td);
			tr.appendChild(key_input_td);
		}

		var table = document.createElement("table");
		table.appendChild(tr);

		return table;
	}

	var getDefaultAndAdditional = function(type) {
		var default_value = "";
		var additional = undefined;
		if (type == "numeric") {
			default_value = "0";
			additional = {min: 0, max: 1};
		} else if (type == "boolean") {
			default_value = "False";
		} else if (type == "yaml") {
			additional = {key: ''};
		} else if (type == "enum") {
			additional = [];
		}
		return {default: default_value, additional: additional};
	}


	//
	//  Interface
	// ===========

	this.setReadonly = function() {
		document.getElementById('db_readonly_overlay').style.display = 'block';
	}

	this.unsetReadonly = function() {
		document.getElementById('db_readonly_overlay').style.display = 'none';
	}

	this.behaviorNameChanged = function() {
		var old_name = Behavior.getBehaviorName();
		var new_name = document.getElementById('input_behavior_name').value;
		if (old_name == new_name) return;

		var activity_text = (old_name == "")? "Set behavior name to " + new_name :
							(new_name == "")? "Deleted behavior name " + old_name :
							"Changed behavior name from " + old_name + " to " + new_name;

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			activity_text,
			function() { Behavior.setBehaviorName(old_name); document.getElementById('input_behavior_name').value = Behavior.getBehaviorName(); },
			function() { Behavior.setBehaviorName(new_name); document.getElementById('input_behavior_name').value = Behavior.getBehaviorName(); }
		);

		Behavior.setBehaviorName(new_name);
	}

	this.behaviorPackageChanged = function() {
		var old_package = Behavior.getBehaviorPackage();
		var new_package = document.getElementById('select_behavior_package').value;
		if (old_package == new_package) return;

		var activity_text = (old_package == "")? "Set behavior package to " + new_package :
							(new_package == "")? "Deleted behavior package " + old_package :
							"Changed behavior package from " + old_package + " to " + new_package;

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			activity_text,
			function() { Behavior.setBehaviorPackage(old_package); document.getElementById('select_behavior_package').value = Behavior.getBehaviorPackage(); },
			function() { Behavior.setBehaviorPackage(new_package); document.getElementById('select_behavior_package').value = Behavior.getBehaviorPackage(); }
		);

		Behavior.setBehaviorPackage(new_package);
	}

	this.behaviorDescriptionChanged = function() {
		var old_desc = Behavior.getBehaviorDescription();
		var new_desc = document.getElementById('input_behavior_description').value;
		if (old_desc == new_desc) return;

		var activity_text = (old_desc == "")? "Set behavior description to " + ((new_desc.length > 33)? new_desc.slice(0,30) + "..." : new_desc) :
							(new_desc == "")? "Deleted behavior description" :
							"Changed behavior description to " + ((new_desc.length > 33)? new_desc.slice(0,30) + "..." : new_desc);

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			activity_text,
			function() { Behavior.setBehaviorDescription(old_desc); document.getElementById('input_behavior_description').value = Behavior.getBehaviorDescription(); },
			function() { Behavior.setBehaviorDescription(new_desc); document.getElementById('input_behavior_description').value = Behavior.getBehaviorDescription(); }
		);

		Behavior.setBehaviorDescription(new_desc);
	}

	this.behaviorTagsChanged = function() {
		var old_tags = Behavior.getTags();
		var new_tags = document.getElementById('input_behavior_tags').value;
		if (old_tags == new_tags) return;

		var activity_text = (old_tags == "")? "Set tags to " + new_tags :
							(new_tags == "")? "Deleted tags " + old_tags :
							"Changed tags from " + old_tags + " to " + new_tags;

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			activity_text,
			function() { Behavior.setTags(old_tags); document.getElementById('input_behavior_tags').value = Behavior.getTags(); },
			function() { Behavior.setTags(new_tags); document.getElementById('input_behavior_tags').value = Behavior.getTags(); }
		);

		Behavior.setTags(new_tags);
	}

	this.behaviorAuthorChanged = function() {
		var old_aut = Behavior.getAuthor();
		var new_aut = document.getElementById('input_behavior_author').value;
		if (old_aut == new_aut) return;

		var activity_text = (old_aut == "")? "Set author to " + new_aut :
							(new_aut == "")? "Deleted author " + old_aut :
							"Changed author from " + old_aut + " to " + new_aut;

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			activity_text,
			function() { Behavior.setAuthor(old_aut); document.getElementById('input_behavior_author').value = Behavior.getAuthor(); },
			function() { Behavior.setAuthor(new_aut); document.getElementById('input_behavior_author').value = Behavior.getAuthor(); }
		);

		Behavior.setAuthor(new_aut);
	}

	this.behaviorDateChanged = function() {
		var old_date = Behavior.getCreationDate();
		var new_date = document.getElementById('input_behavior_date').value;
		if (old_date == new_date) return;

		var activity_text = (old_date == "")? "Set creation date to " + new_date :
							(new_date == "")? "Deleted creation date " + old_date :
							"Changed creation date from " + old_date + " to " + new_date;

		ActivityTracer.addActivity(ActivityTracer.ACT_BEHAVIOR_INTERFACE_CHANGE,
			activity_text,
			function() { Behavior.setCreationDate(old_date); document.getElementById('input_behavior_date').value = Behavior.getCreationDate(); },
			function() { Behavior.setCreationDate(new_date); document.getElementById('input_behavior_date').value = Behavior.getCreationDate(); }
		);

		Behavior.setCreationDate(new_date);
	}

	this.setBehaviorName = function(value) {
		document.getElementById('input_behavior_name').value = value;
		that.behaviorNameChanged();
	}

	this.setBehaviorPackage = function(value) {
		document.getElementById('select_behavior_package').value = value;
		that.behaviorPackageChanged();
	}

	this.setBehaviorDescription = function(value) {
		document.getElementById('input_behavior_description').value = value;
		that.behaviorDescriptionChanged();
	}

	this.setBehaviorTags = function(value) {
		document.getElementById('input_behavior_tags').value = value;
		that.behaviorTagsChanged();
	}

	this.setBehaviorAuthor = function(value) {
		document.getElementById('input_behavior_author').value = value;
		that.behaviorAuthorChanged();
	}

	this.setBehaviorDate = function(value) {
		document.getElementById('input_behavior_date').value = value;
		that.behaviorDateChanged();
	}


	this.addPrivateVariable = function(new_key, new_value) {
		_addPrivateVariable(new_key, new_value);
	}
	this.addPrivateVariableClicked = function() {
		UI.Dashboard.addPrivateVariable(
			document.getElementById("input_db_variable_key_add").value,
			document.getElementById("input_db_variable_value_add").value
		);
		document.getElementById("input_db_variable_key_add").value = "";
		document.getElementById("input_db_variable_value_add").value = "";
	}


	this.addDefaultUserdata = function(new_key, new_value) {
		_addDefaultUserdata(new_key, new_value);
	}
	this.addDefaultUserdataClicked = function() {
		UI.Dashboard.addDefaultUserdata(
			document.getElementById("input_db_userdata_key_add").value,
			document.getElementById("input_db_userdata_value_add").value
		);
		document.getElementById("input_db_userdata_key_add").value = "";
		document.getElementById("input_db_userdata_value_add").value = "";
	}

	this.addParameter = function(new_type, new_name) {
		_addBehaviorParameter(new_type, new_name);
	}
	this.addParameterClicked = function() {
		var type_select = document.getElementById("input_db_parameter_type_add");
		UI.Dashboard.addParameter(
			type_select.options[type_select.selectedIndex].value,
			document.getElementById("input_db_parameter_name_add").value
		);
		document.getElementById("input_db_parameter_type_add").selectedIndex = 0;
		document.getElementById("input_db_parameter_name_add").value = "";
	}
	this.turnParameterClicked = function() {
		document.getElementById("parameter_flipper").classList.toggle("flip");
		show_param_edit = !show_param_edit;
	}


	this.addPrivateFunction = function(new_name, new_params) {
		_addPrivateFunction(new_name, new_params);
	}
	this.addPrivateFunctionClicked = function() {
		UI.Dashboard.addPrivateFunction(
			document.getElementById("input_db_function_name_add").value,
			document.getElementById("input_db_function_params_add").value
		);
		document.getElementById("input_db_function_name_add").value = "";
		document.getElementById("input_db_function_params_add").value = "";
	}


	this.addBehaviorOutcome = function(new_outcome) {
		_addBehaviorOutcome(new_outcome);
	}

	this.addBehaviorOutcomeClicked = function() {
		UI.Dashboard.addBehaviorOutcome(document.getElementById("input_db_outcome_add").value);
		document.getElementById("input_db_outcome_add").value = "";
	}

	this.addBehaviorInputKey = function(new_key) {
		_addBehaviorInputKey(new_key);
	}

	this.addBehaviorInputKeyClicked = function() {
		UI.Dashboard.addBehaviorInputKey(document.getElementById("input_db_input_key_add").value);
		document.getElementById("input_db_input_key_add").value = "";
	}

	this.addBehaviorOutputKey = function(new_key) {
		_addBehaviorOutputKey(new_key);
	}

	this.addBehaviorOutputKeyClicked = function() {
		UI.Dashboard.addBehaviorOutputKey(document.getElementById("input_db_output_key_add").value);
		document.getElementById("input_db_output_key_add").value = "";
	}

	this.resetAllFields = function() {
		UI.Settings.createBehaviorPackageSelect(document.getElementById('select_behavior_package'));
		document.getElementById('input_behavior_name').value = "";
		document.getElementById('input_behavior_description').value = "";
		document.getElementById('input_behavior_tags').value = "";
		document.getElementById('input_behavior_author').value = "";
		var current_date_string = (new Date()).toDateString();
		document.getElementById('input_behavior_date').value = current_date_string;
		Behavior.setCreationDate(current_date_string);

		if (show_param_edit) {
			that.turnParameterClicked();
		}

		document.getElementById('db_variable_table').innerHTML = "";
		document.getElementById('db_userdata_table').innerHTML = "";
		document.getElementById('db_parameter_table').innerHTML = "";
		document.getElementById('db_function_table').innerHTML = "";
		document.getElementById('db_outcome_table').innerHTML = "";
		document.getElementById('db_input_key_table').innerHTML = "";
		document.getElementById('db_output_key_table').innerHTML = "";

		// also reset input fields?
		// currently not
	}

}) ();
