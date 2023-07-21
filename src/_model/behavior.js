Behavior = new (function() {
	var that = this;

	// initial values do not matter because they are set by this.resetBehavior()
	var behavior_name = "";
	var behavior_package = "";
	var behavior_description = "";
	var tags = "";
	var author = "";
	var creation_date = "";

	var private_variables = [];		// {key, value}
	var default_userdata = [];		// {key, value} 
	var private_functions = [];		// {name, params}

	var behavior_parameters = [];	// {type, name, default, label, hint, additional}

	var interface_outcomes = [];
	var interface_input_keys = [];
	var interface_output_keys = [];

	var manual_code_import = "";
	var manual_code_init = "";
	var manual_code_create = "";
	var manual_code_func = "";

	var comment_notes = [];

	var root_sm = undefined;

	var file_name = undefined;
	var manifest_path = undefined;

	var readonly = false;


	this.getBehaviorName = function() {
		return behavior_name;
	}
	this.setBehaviorName = function(_behavior_name) {
		behavior_name = _behavior_name;
	}

	this.getBehaviorPackage = function() {
		return behavior_package;
	}
	this.setBehaviorPackage = function(_behavior_package) {
		behavior_package = _behavior_package;
	}

	this.getBehaviorDescription = function() {
		return behavior_description;
	}
	this.setBehaviorDescription = function(_behavior_description) {
		behavior_description = _behavior_description;
	}

	this.getTags = function() {
		return tags;
	}
	this.setTags = function(_tags) {
		tags = _tags;
	}

	this.getAuthor = function() {
		return author;
	}
	this.setAuthor = function(_author) {
		author = _author;
	}

	this.getCreationDate = function() {
		if (creation_date == "") {
			date = new Date();
			creation_date = date.toDateString();
		}
		return creation_date;
	}
	this.setCreationDate = function(_creation_date) {
		creation_date = _creation_date;
	}

	this.getPrivateVariables = function() {
		return private_variables;
	}
	this.setPrivateVariables = function(_private_variables) {
		private_variables = _private_variables;
	}
	this.updatePrivateVariables = function(old_key, new_key, new_value) {
		for (var i = private_variables.length - 1; i >= 0; i--) {
			if (private_variables[i].key == old_key) {
				private_variables[i].key = new_key;
				private_variables[i].value = new_value;
			}
		};
	}

	this.getDefaultUserdata = function() {
		return default_userdata;
	}
	this.setDefaultUserdata = function(_default_userdata) {
		default_userdata = _default_userdata;
	}
	this.updateDefaultUserdata = function(old_key, new_key, new_value) {
		for (var i = default_userdata.length - 1; i >= 0; i--) {
			if (default_userdata[i].key == old_key) {
				default_userdata[i].key = new_key;
				default_userdata[i].value = new_value;
			}
		};
	}

	this.getPrivateFunctions = function() {
		return private_functions;
	}
	this.setPrivateFunctions = function(_private_functions) {
		private_functions = _private_functions;
	}
	this.updatePrivateFunctions = function(old_name, new_name, new_params) {
		for (var i = private_functions.length - 1; i >= 0; i--) {
			if (private_functions[i].name == old_name) {
				private_functions[i].name = new_name;
				private_functions[i].params = new_params;
			}
		};
	}

	this.getBehaviorParameters = function() {
		return behavior_parameters;
	}
	this.setBehaviorParameters = function(_behavior_parameters) {
		behavior_parameters = _behavior_parameters;
	}
	this.updateBehaviorParameter = function(old_name, new_value, key) {
		for (var i = behavior_parameters.length - 1; i >= 0; i--) {
			if (behavior_parameters[i].name == old_name) {
				if (key == "name")
					behavior_parameters[i].name = new_value;
				else if (key == "type")
					behavior_parameters[i].type = new_value;
				else if (key == "default")
					behavior_parameters[i].default = new_value;
				else if (key == "label")
					behavior_parameters[i].label = new_value;
				else if (key == "hint")
					behavior_parameters[i].hint = new_value;
				else if (key == "additional")
					behavior_parameters[i].additional = new_value;
			}
		};
	}
	this.removeBehaviorParameter = function(target_name) {
		var to_remove = behavior_parameters.findElement(function(element) {
			return element.name == target_name;
		});
		behavior_parameters.remove(to_remove);
	}

	this.getInterfaceOutcomes = function() {
		return interface_outcomes;
	}
	this.addInterfaceOutcome = function(to_add) {
		interface_outcomes.push(to_add);

		root_sm.addOutcome(to_add);
	}
	this.removeInterfaceOutcome = function(to_remove) {
		interface_outcomes.remove(to_remove);

		root_sm.removeOutcome(to_remove);
	}
	this.updateInterfaceOutcome = function(old_value, new_value) {
		for (var i = interface_outcomes.length - 1; i >= 0; i--) {
			if (interface_outcomes[i] == old_value) {
				interface_outcomes[i] = new_value;

				root_sm.updateOutcome(old_value, new_value);
			}
		};
	}

	this.getInterfaceInputKeys = function() {
		return interface_input_keys;
	}
	this.setInterfaceInputKeys = function(_interface_input_keys) {
		interface_input_keys = _interface_input_keys;
		root_sm.setInputKeys(interface_input_keys);
	}
	this.addInterfaceInputKey = function(key) {
		interface_input_keys.push(key);
		root_sm.setInputKeys(interface_input_keys);
	}
	this.removeInterfaceInputKey = function(key) {
		interface_input_keys.remove(key);
		root_sm.setInputKeys(interface_input_keys);
	}
	this.updateInterfaceInputKeys = function(old_value, new_value) {
		for (var i = interface_input_keys.length - 1; i >= 0; i--) {
			if (interface_input_keys[i] == old_value) {
				interface_input_keys[i] = new_value;
				root_sm.getInputKeys().remove(old_value);
				root_sm.getInputKeys().push(new_value);
			}
		};
	}

	this.getInterfaceOutputKeys = function() {
		return interface_output_keys;
	}
	this.setInterfaceOutputKeys = function(_interface_output_keys) {
		interface_output_keys = _interface_output_keys;
		root_sm.setOutputKeys(interface_output_keys);
	}
	this.addInterfaceOutputKey = function(key) {
		interface_output_keys.push(key);
		root_sm.setOutputKeys(interface_output_keys);
	}
	this.removeInterfaceOutputKey = function(key) {
		interface_output_keys.remove(key);
		root_sm.setOutputKeys(interface_output_keys);
	}
	this.updateInterfaceOutputKeys = function(old_value, new_value) {
		for (var i = interface_output_keys.length - 1; i >= 0; i--) {
			if (interface_output_keys[i] == old_value)
				interface_output_keys[i] = new_value;
				root_sm.getOutputKeys().remove(old_value);
				root_sm.getOutputKeys().push(new_value);
		};
	}

	this.getManualCodeImport = function() {
		return manual_code_import;
	}
	this.setManualCodeImport = function(_manual_code_import) {
		manual_code_import = _manual_code_import;
	}

	this.getManualCodeInit = function() {
		return manual_code_init;
	}
	this.setManualCodeInit = function(_manual_code_init) {
		manual_code_init = _manual_code_init;
	}

	this.getManualCodeCreate = function() {
		return manual_code_create;
	}
	this.setManualCodeCreate = function(_manual_code_create) {
		manual_code_create = _manual_code_create;
	}

	this.getManualCodeFunc = function() {
		return manual_code_func;
	}
	this.setManualCodeFunc = function(_manual_code_func) {
		manual_code_func = _manual_code_func;
	}

	this.getCommentNotes = function() {
		return comment_notes;
	}
	this.addCommentNote = function(new_note) {
		comment_notes.push(new_note);
	}
	this.removeCommentNote = function(note) {
		comment_notes.remove(note);
	}
	this.clearCommentNotes = function() {
		comment_notes = [];
	}

	this.getStatemachine = function() {
		if (root_sm == undefined) {
			T.debugWarn("Trying to access undefined root state machine!");
		}

		return root_sm;
	}
	this.setStatemachine = function(_root_sm) {
		root_sm = _root_sm;
	}

	this.setReadonly = function(_readonly) {
		readonly = _readonly;
	}

	this.isReadonly = function() {
		return readonly;
	}

	this.resetBehavior = function() {
		behavior_name = "";
		behavior_package = "";
		behavior_description = "";
		author = "";
		creation_date = "";

		private_variables = [];		// {key, value}
		default_userdata = [];		// {key, value} 
		private_functions = [];		// {name, params}

		behavior_parameters = [];	// {type, name, default, label, hint, additional}

		interface_outcomes = [];
		interface_input_keys = [];
		interface_output_keys = [];

		manual_code_import = "";
		manual_code_init = "";
		manual_code_create = "";
		manual_code_func = "";
		
		comment_notes = [];

		root_sm = new Statemachine("", new WS.StateMachineDefinition([], [], []));
		readonly = false;
	}

	this.setFiles = function(_file_name, _manifest_path) {
		file_name = _file_name;
		manifest_path = _manifest_path;
	}

	this.createNames = function() {
		var result = {
			behavior_name: '',
			rosnode_name: '',
			class_name: '',
			manifest_name: '',
			manifest_path: '',
			file_name: '',
			file_name_tmp: ''
		};
		result.behavior_name = behavior_name;
		result.rosnode_name = behavior_package;
		result.class_name = behavior_name.replace(/[^\w]/g, "") + 'SM';
		result.manifest_name = behavior_name.toLowerCase().replace(/[^\w]/g, "_") + '.xml';
		result.manifest_path = manifest_path;
		result.file_name = file_name || behavior_name.toLowerCase().replace(/[^\w]/g, "_") + '_sm.py';
		result.file_name_tmp = result.file_name.replace(/\.py$/, "_tmp.py");
		return result;
	}

	this.createStructureInfo = function() {
		var result = [];

		createStateStructure(root_sm, result);

		return result;
	}

	var createStateStructure = function(s, info) {
		var result = {};
		result.path = s.getStatePath();
		result.outcomes = s.getOutcomes();
		result.transitions = [];
		try {
			if (s.getContainer() != undefined) {
				result.autonomy = s.getAutonomy();
				var transitions = s.getContainer().getTransitions();
				for (var i=0; i<result.outcomes.length; i++) {
					var transition = transitions.findElement(function(element) {
						return element.getFrom().getStateName() == s.getStateName() && element.getOutcome() == result.outcomes[i];
					});
					var target_name = transition.getTo().getStateName();
					if (s.getContainer().isConcurrent() && transition.getTo().getStateClass() == ':CONDITION') {
						target_name = target_name.split('#')[0];
					}
					result.transitions.push(target_name);
				}
			}
			result.children = [];
			if (s instanceof BehaviorState) {
				s = s.getBehaviorStatemachine();
			}
			if (s instanceof Statemachine) {
				var children = s.getStates();
				for (var c=0; c<children.length; c++) {
					var child = children[c];
					result.children.push(children[c].getStateName());
					createStateStructure(children[c], info);
				}
			}
		} catch (error) {
			throw {path: error.path || result.path, error: error.error || error};
		}
		info.push(result);
	}

}) ();