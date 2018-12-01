IO.CodeGenerator = new (function() {
	var that = this;

	var names;
	var sm_counter = 0;
	var sm_names = [];

	var ws = '\t';

	var autonomyMapping = function(autonomy_int) {
		switch(parseInt(autonomy_int)) {
			case 0: return "Autonomy.Off";
			case 1: return "Autonomy.Low";
			case 2: return "Autonomy.High";
			case 3: return "Autonomy.Full";
			default: return "Autonomy.Inherit";
		}
	}

	var generateImports = function() {
		var code = "";

		// collect all states
		var states = helper_collectAllStates(Behavior.getStatemachine());

		// collect required imports
		var imported_states = [];
		for (var i=0; i<states.length; ++i) {
			if (states[i] instanceof Statemachine) continue;

			var import_reference = imported_states.findElement(function(element) {
				return element.getStateClass() == states[i].getStateClass()
					&& element.getStatePackage() == states[i].getStatePackage();
			});
			if (import_reference == undefined) {
				imported_states.push(states[i]);
			}
		}
		// generate import lines
		var import_list = [];
		for (var i=0; i<imported_states.length; ++i) {
			if (!UI.Settings.isExplicitStates() && WS.Statelib.isClassUnique(imported_states[i].getStateClass())) {
				import_list.push("from " + imported_states[i].getStateImport() + " import " + imported_states[i].getStateClass());	
			} else {
				import_list.push("from " + imported_states[i].getStateImport() + " import " + imported_states[i].getStateClass()
					+ " as " + imported_states[i].getStatePackage() + "__" + imported_states[i].getStateClass());
			}
		}
		// put together
		code += "from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger\n";
		code += import_list.join("\n");
		code += "\n";
		// add manual imports
		code += "# Additional imports can be added inside the following tags\n";
		code += "# [MANUAL_IMPORT]";
		if (Behavior.getManualCodeImport() == "") {
			code += "\n\n";
		} else {
			code += Behavior.getManualCodeImport();
		}
		code += "# [/MANUAL_IMPORT]\n";

		return code;
	}

	var generateBehaviorHead = function() {
		var code = "";
		code += "'''\n";
		code += "Created on " + Behavior.getCreationDate() + "\n";
		code += "@author: " + Behavior.getAuthor() + "\n";
		code += "'''\n";
		code += "class " + names.class_name + "(Behavior):\n";
		code += ws+"'''\n";
		code += ws+ Behavior.getBehaviorDescription() + "\n";
		code += ws+"'''\n";
		return code;
	}

	var generateInitialization = function() {
		var code = "";
		// header
		code += ws+"def __init__(self):\n";
		code += ws+ws+"super(" + names.class_name + ", self).__init__()\n";
		code += ws+ws+"self.name = '" + names.behavior_name + "'\n";
		code += "\n";
		// parameters
		code += ws+ws+"# parameters of this behavior\n";
		var params = Behavior.getBehaviorParameters();
		for (var i = 0; i < params.length; i++) {
			var default_value = 
				(params[i].type == "text" || params[i].type == "enum")? "'" + params[i].default + "'" :
				(params[i].type == "yaml")? "dict()" : 
				params[i].default;
			code += ws+ws+"self.add_parameter('" + params[i].name + "', " + default_value + ")\n";
		};
		code += "\n";
		// contains
		code += ws+ws+"# references to used behaviors\n";
		var states = helper_collectAllStates(Behavior.getStatemachine());
		var contained_behaviors = [];
		for (var i = 0; i < states.length; i++) {
			if (!(states[i] instanceof BehaviorState)) continue;

			contained_behaviors.push(states[i]);
		}
		for (var i=0; i<contained_behaviors.length; ++i) {
			code += ws+ws+"self.add_behavior(" + contained_behaviors[i].getStateClass() + ", '" + contained_behaviors[i].getStatePath().substr(1) + "')\n";
		}
		code += "\n";
		// manual
		code += ws+ws+"# Additional initialization code can be added inside the following tags\n";
		code += ws+ws+"# [MANUAL_INIT]";
		if (Behavior.getManualCodeInit() == "") {
			code += "\n"+ws+ws+"\n"+ws+ws;
		} else {
			code += Behavior.getManualCodeInit();
		}
		code += "# [/MANUAL_INIT]\n";
		code += "\n";
		code += ws+ws+"# Behavior comments:\n\n";
		var notes = Behavior.getCommentNotes();
		for (var i = 0; i < notes.length; i++) {
			var n = notes[i];
			code += ws+ws+"# " + (n.isImportant()? "!" : "O") + " " + Math.round(n.getPosition().x) + " " + Math.round(n.getPosition().y) + " " + n.getContainerPath() + "\n";
			code += ws+ws+"# " + n.getContent().replace(/\n/g, "\|n") + "\n\n";
		}
		return code;
	}

	var generateCreation = function() {
		var code = "";
		code += ws+"def create(self):\n";

		// private vars
		for (var i=0; i<Behavior.getPrivateVariables().length; ++i) {
			var v = Behavior.getPrivateVariables()[i];
			code += ws+ws+ v.key + " = " + v.value.trim() + "\n";
		}

		// root declaration
		var pos = [];
		for (var i = 0; i < Behavior.getStatemachine().getSMOutcomes().length; i++) {
			var position = Behavior.getStatemachine().getSMOutcomes()[i].getPosition();
			pos.push("x:" + Math.round(position.x) + " y:" + Math.round(position.y)); 
		}
		code += ws+ws+"# " + pos.join(", ") + "\n";
		code += ws+ws+"_state_machine = OperatableStateMachine(outcomes=['" + Behavior.getInterfaceOutcomes().join("', '") + "']";
		if (Behavior.getInterfaceInputKeys().length > 0) {
			code += ", input_keys=['" + Behavior.getInterfaceInputKeys().join("', '") + "']";
		}
		if (Behavior.getInterfaceOutputKeys().length > 0) {
			code += ", output_keys=['" + Behavior.getInterfaceOutputKeys().join("', '") + "']";
		}
		code += ")\n";

		// default userdata
		for (var i=0; i<Behavior.getDefaultUserdata().length; ++i) {
			var u = Behavior.getDefaultUserdata()[i];
			code += ws+ws+"_state_machine.userdata." + u.key.replace(/"/g, "") + " = " + u.value.trim() + "\n";
		}
		code += "\n";

		// manual creation code
		code += ws+ws+"# Additional creation code can be added inside the following tags\n";
		code += ws+ws+"# [MANUAL_CREATE]";
		if (Behavior.getManualCodeCreate() == "") {
			code += "\n"+ws+ws+"\n"+ws+ws;
		} else {
			code += Behavior.getManualCodeCreate();
		}
		code += "# [/MANUAL_CREATE]\n";
		code += "\n";

		// generate contained state machines
		var sub_sms = helper_getAllSubSMs(Behavior.getStatemachine());
		for (var i = sub_sms.length - 1; i >= 0; i--) {
			code += generateStateMachine(sub_sms[i], true);
			code += "\n";
		}

		// generate root state machine
		code += "\n";
		code += generateStateMachine(Behavior.getStatemachine(), false);
		code += "\n";

		code += ws+ws+"return _state_machine\n";
		return code;
	}

	var generateFunctions = function() {
		var code = "";
		code += ws+"# Private functions can be added inside the following tags\n";
		code += ws+"# [MANUAL_FUNC]";
		if (Behavior.getManualCodeFunc() == "") {
			code += "\n"+ws+"\n"+ws;
		} else {
			code += Behavior.getManualCodeFunc();
		}
		code += "# [/MANUAL_FUNC]\n";
		return code;
	}

	var generateStateMachine = function(sm, include_header) {
		var code = "";
		var sm_name = (sm.getStateName() == "") ? "_state_machine" :
				"_sm_" + sm.getStateName().toLowerCase().replace(/ /g, "_") + "_" + sm_counter;
		sm_counter += 1;
		sm_names.push({sm: sm, name: sm_name});

		// sm declaration
		if (include_header) {
			var pos = [];
			for (var i = 0; i < sm.getSMOutcomes().length; i++) {
				var position = sm.getSMOutcomes()[i].getPosition();
				pos.push("x:" + Math.round(position.x) + " y:" + Math.round(position.y)); 
			}
			code += ws+ws+"# " + pos.join(", ") + "\n";
			if (sm.isConcurrent()) {
				code += ws+ws+ sm_name + " = ConcurrencyContainer(outcomes=['" + sm.getOutcomes().join("', '") + "']";
			} else if (sm.isPriority()) {
				code += ws+ws+ sm_name + " = PriorityContainer(outcomes=['" + sm.getOutcomes().join("', '") + "']";
			} else {
				code += ws+ws+ sm_name + " = OperatableStateMachine(outcomes=['" + sm.getOutcomes().join("', '") + "']";
			}
			if (sm.getInputKeys().length > 0) {
				code += ", input_keys=['" + sm.getInputKeys().join("', '") + "']";
			}
			if (sm.getOutputKeys().length > 0) {
				code += ", output_keys=['" + sm.getOutputKeys().join("', '") + "']";
			}
			if (sm.isConcurrent()) {
				var conditions = sm.getConditions();
				code += ", conditions=[\n";
				var list_entries = []
				for (var i = 0; i < conditions.outcomes.length; i++) {
					var transitions_list = [];
					conditions.transitions[i].forEach(function(t) {
						transitions_list.push("('" + t[0] + "', '" + t[1] + "')");
					});
					list_entries.push(ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+"('" + conditions.outcomes[i].split('#')[0] + "', [" + transitions_list.join(', ') + "])");
				}
				code += list_entries.join(',\n') + "\n"+ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+"]"
			}
			code += ")\n\n";
		}

		code += ws+ws+"with " + sm_name + ":\n";

		// smach needs to start with initial state
		var states = sm.getStates();
		var init_trans = sm.getTransitions().findElement(function(element) {
			return element.getFrom().getStateName() == "INIT";
		});
		var init_state = states.findElement(function(element) {
			return element.getStateName() == init_trans.getTo().getStateName();
		});
		if (init_state != states[0]) {
			states.remove(init_state);
			var temp_state = states[0];
			states[0] = init_state;
			states.push(temp_state);
		}

		// add states
		for (var i=0; i<states.length; ++i) {
			var s = states[i];
			code += generateState(s, sm.getTransitions());
		}

		return code;
	}

	var generateState = function(s, t) {
		var code = "";
		// comment section for internal data
		code += ws+ws+ws+"# x:" + Math.round(s.getPosition().x) + " y:" + Math.round(s.getPosition().y);
		var internal_param_list = [];
		for (var j = 0; j < s.getParameters().length; j++) {
			var p_k = s.getParameters()[j];
			if (!p_k.startsWith("?")) continue;
			var p_v = s.getParameterValues()[j];
			internal_param_list.push(p_k + " = " + p_v);
		}
		if (internal_param_list.length > 0) {
			code += " {" + internal_param_list.join(",") + "}";
		}
		code += "\n";
		
		code += ws+ws+ws+"OperatableStateMachine.add('" + s.getStateName() + "',\n";

		// class
		if (s instanceof Statemachine) {
			var sm_name = sm_names.findElement(function (element) {
				return element.sm.getStatePath() == s.getStatePath();
			}).name;
			code += ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+ sm_name + ",\n";

		} else if (s instanceof BehaviorState) {
			var defkeys_str = "";
			var be_defkeys = s.getDefaultKeys();
			if (be_defkeys.length > 0) {
				var be_defkeys_str = []
				for (var j = 0; j < be_defkeys.length; j++) {
					be_defkeys_str.push("'"+be_defkeys[j]+"'");
				}
				defkeys_str = ", default_keys=[" + be_defkeys_str.join(',') + "]";
			}
			code += ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+"self.use_behavior(" + s.getStateClass() + ", '" + s.getStatePath().substr(1) + "'" + defkeys_str + "),\n";

		} else {
			var class_key = (!UI.Settings.isExplicitStates() && WS.Statelib.isClassUnique(s.getStateClass()))?
				s.getStateClass() :
				s.getStatePackage() + "__" + s.getStateClass();
			code += ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+ class_key + "(";
			var param_strings = [];
			for (var j=0; j<s.getParameters().length; ++j) {
				if (s.getParameters()[j].startsWith("?")) continue;
				param_strings.push(s.getParameters()[j] + "=" + s.getParameterValues()[j]);
			}
			code += param_strings.join(", ");
			code += "),\n";
		}

		// transitions
		code += ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+"transitions={";
		var transition_strings = [];
		var state_transitions = t.filter(function (element) {
			return element.getFrom().getStateName() == s.getStateName();
		});
		for (var j=0; j<s.getOutcomes().length; ++j) {
			var outcome_transition = state_transitions.findElement(function (element) {
				return element.getOutcome() == s.getOutcomes()[j];
			});
			if (outcome_transition == undefined) throw "outcome '" + s.getOutcomes()[j] + "' in state '" + s.getStateName() + "' is not connected";
			if (outcome_transition.getTo().getStateName() == s.getStateName()) T.logWarn("Looping transition for outcome '" + s.getOutcomes()[j] + "' in state '" + s.getStateName() + "' detected");
			var transition_target = outcome_transition.getTo().getStateName();
			if (outcome_transition.getTo().getStateClass() == ':CONDITION') transition_target = transition_target.split('#')[0];
			transition_strings.push("'" + s.getOutcomes()[j] + "': '" + transition_target + "'");
		}
		code += transition_strings.join(", ");
		code += "},\n";

		// autonomy
		code += ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+"autonomy={";
		var autonomy_strings = [];
		for (var j=0; j<s.getOutcomes().length; ++j) {
			autonomy_strings.push("'" + s.getOutcomes()[j] + "': " + autonomyMapping(s.getAutonomy()[j]));
		}
		code += autonomy_strings.join(", ");
		code += "}";

		// remapping
		if (s.getInputKeys().length + s.getOutputKeys().length > 0) {
			code += ",\n";
			code += ws+ws+ws+ws+ws+ws+ws+ws+ws+ws+"remapping={";
			var remapping_strings = [];
			for (var j=0; j<s.getInputKeys().length; ++j) {
				remapping_strings.push("'" + s.getInputKeys()[j] + "': '" + s.getInputMapping()[j] + "'");
			}
			for (var j=0; j<s.getOutputKeys().length; ++j) {
				if (s.getInputKeys().contains(s.getOutputKeys()[j])) continue;
				remapping_strings.push("'" + s.getOutputKeys()[j] + "': '" + s.getOutputMapping()[j] + "'");
			}
			code += remapping_strings.join(", ");
			code += "}";
		}
		
		code += ")\n\n";
		T.logInfo("[+] " + s.getStateName());
		return code;
	}

	this.generateBehaviorCode = function() {
		T.logInfo("Generating code for " + Behavior.getBehaviorName() + "...");
		// test conditions for generating code
		if (Behavior.getStatemachine().getStates().length == 0) throw "state machine contains no states";
		
		names = Behavior.createNames();
		sm_counter = 0;
		sm_names = [];
		ws = UI.Settings.getCodeIndentation();
		console.log('Using whitespace: "'+ws+'"')

		// prefix
		var code = "#!/usr/bin/env python\n";
		code += "# -*- coding: utf-8 -*-\n";
		code += "###########################################################\n";
		code += "#               WARNING: Generated code!                  #\n";
		code += "#              **************************                 #\n";
		code += "# Manual changes may get lost if file is generated again. #\n";
		code += "# Only code inside the [MANUAL] tags will be kept.        #\n";
		code += "###########################################################\n";
		code += "\n";

		// imports
		code += generateImports();
		code += "\n\n";

		// behavior head
		code += generateBehaviorHead();
		code += "\n\n";

		// behavior initialization
		code += generateInitialization();
		code += "\n\n";

		// behavior creation
		code += generateCreation();
		code += "\n\n";

		// private functions
		code += generateFunctions();

		return code;
	}

	var helper_getAllSubSMs = function(sm) {
		var sub_sms = sm.getStates().filter(function(element) {
			return element instanceof Statemachine;
		});
		var sub_sub_sms = []
		sub_sms.forEach(function(element, i) {
			helper_getAllSubSMs(element).forEach(function(element, i) {
				sub_sub_sms.push(element);
			});
		});
		sub_sub_sms.forEach(function(element, i) {
			sub_sms.push(element);
		});
		return sub_sms;
	}

	var helper_collectAllStates = function(sm) {
		var states = [];

		sm.getStates().forEach(function(element, i) {
			states.push(element);
			if (element instanceof Statemachine)
				helper_collectAllStates(element).forEach(function(state, j) {
					states.push(state);
				});
		});

		return states;
	}

}) ();
