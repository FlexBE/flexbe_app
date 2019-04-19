IO.ModelGenerator = new (function() {
	var that = this;

	this.generateBehaviorAttributes = function(data, manifest) {
		UI.Dashboard.setBehaviorName(manifest.name);
		UI.Dashboard.setBehaviorPackage(manifest.rosnode_name);
		UI.Dashboard.setBehaviorDescription(manifest.description);
		UI.Dashboard.setBehaviorTags(manifest.tags);
		UI.Dashboard.setBehaviorAuthor(manifest.author);
		UI.Dashboard.setBehaviorDate(manifest.date != undefined? manifest.date : data.creation_date);

		Behavior.setManualCodeImport(data.manual_code.manual_import);
		Behavior.setManualCodeInit(data.manual_code.manual_init);
		Behavior.setManualCodeCreate(data.manual_code.manual_create);
		Behavior.setManualCodeFunc(data.manual_code.manual_func);

		data.behavior_comments.forEach(function(element, i) {
			var note = new Note(element.content);
			note.setPosition({x: element.pos_x, y: element.pos_y});
			note.setContainerPath(element.container);
			note.setImportant(element.important);
			Behavior.addCommentNote(note);
		});

		data.private_variables.forEach(function(element, i) {
			UI.Dashboard.addPrivateVariable(element.key, element.value);
		});
		data.default_userdata.forEach(function(element, i) {
			UI.Dashboard.addDefaultUserdata(element.key, element.value);
		});
		data.private_functions.forEach(function(element, i) {
			UI.Dashboard.addPrivateFunction(element.key, element.value);
		});

		data.smi_outcomes.forEach(function(element, i) {
			UI.Dashboard.addBehaviorOutcome(element);
		});
		data.smi_input.forEach(function(element, i) {
			UI.Dashboard.addBehaviorInputKey(element);
		});
		data.smi_output.forEach(function(element, i) {
			UI.Dashboard.addBehaviorOutputKey(element);
		});

		manifest.params.forEach(function(element) {
			UI.Dashboard.addParameter(element.type, element.name);
			Behavior.updateBehaviorParameter(element.name, element.default, "default");
			Behavior.updateBehaviorParameter(element.name, element.label, "label");
			Behavior.updateBehaviorParameter(element.name, element.hint, "hint");
			Behavior.updateBehaviorParameter(element.name, element.additional, "additional");
		});
	}

	this.buildStateMachine = function(container_name, container_sm_var_name, sm_defs, sm_states, silent) {
		var container_sm_def = sm_defs.findElement(function(element) {
			return element.sm_name == container_sm_var_name;
		});
		if (container_sm_def == undefined) {
			T.logError('Unknown container definition: ' + container_sm_var_name);
			return new Statemachine(container_name, WS.Statelib.getFromLib(':CONTAINER'));
		}
		var container_sm = new Statemachine(container_name, new WS.StateMachineDefinition(
			container_sm_def.sm_params.outcomes,
			container_sm_def.sm_params.input_keys,
			container_sm_def.sm_params.output_keys
		));
		if (container_sm_def.sm_type == "concurrency") {
			container_sm.setConcurrent(true);
		} else if (container_sm_def.sm_type == "priority") {
			container_sm.setPriority(true);
		}

		// add states
		var container_states = sm_states.findElement(function(element) {
			return element.sm_name == container_sm_var_name;
		}).sm_states;
		for (var i=0; i<container_states.length; i++) {
			var s_def = container_states[i];
			var s;
			if (s_def.state_type == "container") {
				s = that.buildStateMachine(s_def.state_name, s_def.state_class, sm_defs, sm_states, silent);
			} else if (s_def.state_type == "behavior") {
				var state_def = WS.Behaviorlib.getByClass(s_def.state_class);
				if (state_def == undefined) {
					T.logError("Unable to find behavior definition for: " + s_def.state_class);
					T.logInfo("Please check your workspace settings.");
					continue;
				}
				s = new BehaviorState(s_def.state_name, state_def, s_def.parameter_values);
			} else {
				var state_def = undefined;
				if (s_def.state_class.includes("__")) {
					var type_split = s_def.state_class.split("__");
					var state_key = type_split[0] + "." + type_split[1];
					var state_def = WS.Statelib.getFromLib(state_key);
					if (state_def == undefined) {
						s_def.state_class = type_split[1];
					}
				} 
				if (state_def == undefined) {
					var state_key = s_def.state_class;
					var state_def = WS.Statelib.getClassFromLib(state_key, WS.Statelib.isClassUnique(state_key)? undefined : lib_def => {
						for (var j=0; j<s_def.transitions_from.length; j++) {
							if (!lib_def.getOutcomes().contains(s_def.transitions_from[j].outcome)) {
								return false;
							}
						}
						return true;
					});
					if (state_def != undefined && !WS.Statelib.isClassUnique(state_key)) {
						T.logWarn("State class name " + state_key + " is not unique, but was able to re-construct.");
						T.logWarn("Please save behavior again to fix this for the future.");
					}
				}
				if (state_def == undefined) {
					T.logError("Unable to find state definition for: " + state_key);
					T.logInfo("Please check your workspace settings.");
					continue;
				}
				s = new State(s_def.state_name, state_def);
				s.setParameterValues(helper_getSortedValueList(s.getParameters(), s.getParameterValues(), s_def.parameter_values));
			}
			s.setAutonomy(helper_getSortedValueList(s.getOutcomes(), s.getAutonomy(), s_def.autonomy));
			s.setInputMapping(helper_getSortedValueList(s.getInputKeys(), s.getInputMapping(), s_def.remapping));
			s.setOutputMapping(helper_getSortedValueList(s.getOutputKeys(), s.getOutputMapping(), s_def.remapping));
			s.setPosition({x: s_def.state_pos_x, y: s_def.state_pos_y});

			container_sm.addState(s);
			if(!silent) {
				T.logInfo("[+] " + s.getStateName());
			}

			// In SMACH, initial state is always the first one defined
			if (container_sm_def.initial == undefined && i == 0
				|| container_sm_def.initial == s_def.state_name)
			{
				container_sm.setInitialState(s);
			}
		}
		

		// add transitions (requires to have all states)
		if (container_sm.isConcurrent()) {
			container_sm.setConditions(container_sm_def.sm_params.conditions);
			// only initial state has autonomy levels
			var s_def = container_states[0];
			var state_from = container_sm.getStateByName(s_def.state_name);
			for (var j=0; j<s_def.transitions_from.length; j++) {
				var trans_def = s_def.transitions_from[j];
				var autonomy_idx = state_from.getOutcomes().indexOf(trans_def.outcome);
				var autonomy = state_from.getAutonomy()[autonomy_idx];
				var trans = container_sm.getTransitions().findElement(function(t) {
					return t.getFrom().getStateName() == s_def.state_name
						&& t.getOutcome() == trans_def.outcome;
				});
				if (trans == undefined) {
					T.logWarn('Missing conditional transition definition for: ' + s_def.state_name + ' > ' + trans_def.outcome);
					continue;
				}
				trans.setAutonomy(autonomy);
			}
		} else {
			for (var i=0; i<container_states.length; i++) {
				var s_def = container_states[i];
				var state_from = container_sm.getStateByName(s_def.state_name);
				for (var j=0; j<s_def.transitions_from.length; j++) {
					var trans_def = s_def.transitions_from[j];
					var state_to;
					if (container_sm.getOutcomes().contains(trans_def.target)) {
						state_to = container_sm.getSMOutcomeByName(trans_def.target);
					} else if (!container_sm.isConcurrent()) {
						state_to = container_sm.getStateByName(trans_def.target);
					}
					if (state_to == undefined) {
						T.logWarn('Unknown transition target of state ' + s_def.state_name + ': ' + trans_def.target);
						continue;
					}
					var autonomy_idx = state_from.getOutcomes().indexOf(trans_def.outcome);
					var autonomy = state_from.getAutonomy()[autonomy_idx];
					var trans = new Transition(state_from, state_to, trans_def.outcome, autonomy);
					container_sm.addTransition(trans);
				}
			}
		}
		var oc_objs = container_sm.getSMOutcomes();
		var oc_pos_len = Math.min(oc_objs.length, container_sm_def.oc_positions.length);
		for (var i = 0; i < oc_pos_len; i++) {
			oc_objs[i].setPosition(container_sm_def.oc_positions[i]);
		}

		return container_sm;
	}

	this.parseInstantiationMsg = function(states) {
		var sm_defs = [];
		var sm_states = [];

		var required_properties = [
			'state_path', 'state_class',
			'initial_state_name', 'input_keys', 'output_keys',
			'cond_outcome', 'cond_transition',
			'behavior_class',
			'parameter_names', 'parameter_values',
			'position',
			'outcomes', 'transitions', 'autonomy',
			'userdata_keys', 'userdata_remapping'];
		var missing_properties = [];
		for (property of required_properties) {
			if (!states[0].hasOwnProperty(property)) {
				missing_properties.push(property);
			}
		}
		if (missing_properties.length > 0) {
			T.clearLog();
			T.logError('Missing required properties in message definition:');
			T.logError(missing_properties.join(', '));
			T.logInfo('Make sure you use the correct version of the StateInstantiation message.');
			return undefined;
		}
		

		states.forEach(function(s) {
			var path_split = s.state_path.split("/");
			var container_name = path_split.slice(0, -1).join('_');
			var state_name = path_split.slice(-1)[0];
			var sm_varname = (state_name == '')? '' : container_name + '_' + state_name;
			if (state_name != "") {
				var sm_state_list = sm_states.findElement(function(el) { return el.sm_name == container_name; });
				if (sm_state_list == undefined) {
					sm_state_list = { sm_name: container_name, sm_states: [] };
					sm_states.push(sm_state_list);
				}
				var state_type = "state";
				var state_class = s.state_class;
				if (state_class == ":STATEMACHINE" || state_class == ":CONCURRENCY" || state_class == ":PRIORITY") {
					state_type = "container";
					state_class = sm_varname;
				}
				if (state_class == ":BEHAVIOR") {
					state_type = "behavior";
					behavior_def = WS.Behaviorlib.getByName(s.behavior_class);
					if (behavior_def == undefined) {
						T.logWarn('Unknown behavior reference: ' + s.behavior_class);
						return;
					}
					state_class = behavior_def.getStateClass();
				}
				var parameter_values = [];
				for (var i=0; i<s.parameter_names.length; i++) {
					parameter_values.push({key: s.parameter_names[i], value: s.parameter_values[i]});
				}
				var autonomy = [];
				for (var i=0; i<s.autonomy.length; i++) {
					autonomy.push({key: s.outcomes[i], value: s.autonomy[i]});
				}
				var remapping = [];
				for (var i=0; i<s.userdata_keys.length; i++) {
					remapping.push({key: s.userdata_keys[i], value: s.userdata_remapping[i]});
				}
				var transitions_from = [];
				for (var i=0; i<s.transitions.length; i++) {
					transitions_from.push({outcome: s.outcomes[i], target: s.transitions[i]});
				}
				var pos_x = 0;
				var pos_y = 0;
				if (s.position.length == 2) {
					pos_x = s.position[0];
					pos_y = s.position[1];
				} else {
					T.logWarn('Invalid position definition for state: ' + state_name);
				}
				var state = {
					state_name: state_name,
					state_class: state_class,
					state_type: state_type,
					state_pos_x: pos_x + 30,
					state_pos_y: pos_y + 40,
					parameter_values: parameter_values,
					autonomy: autonomy,
					remapping: remapping,
					transitions_from: transitions_from
				};
				sm_state_list.sm_states.push(state);
			}
			if (s.state_class == ":STATEMACHINE" || s.state_class == ":CONCURRENCY" || s.state_class == ":PRIORITY") {
				var conditions = undefined;
				if (s.state_class == ":CONCURRENCY") {
					conditions = {
						outcomes: s.cond_outcome,
						transitions: []
					}
					s.cond_transition.forEach(function(t_msg) {
						var t = [];
						for (var i=0; i<t_msg.state_name.length; i++) {
							t.push([t_msg.state_name[i], t_msg.state_outcome[i]]);
						}
						conditions.transitions.push(t);
					});
				}
				sm_defs.push({
					sm_name: sm_varname,
					sm_params: {
						outcomes: s.outcomes,
						input_keys: s.input_keys,
						output_keys: s.output_keys,
						conditions: conditions
					},
					oc_positions: [],
					sm_type: (s.state_class == ":CONCURRENCY")? "concurrency":
							 (s.state_class == ":PRIORITY")? "priority" :
							 "statemachine",
					initial: s.initial_state_name 
				});
			}
		});

		return {
			sm_defs: sm_defs,
			sm_states: sm_states
		};
	}


	var helper_getSortedValueList = function(key_list, default_value, dict) {
		var result = [];
		for (var i=0; i<dict.length; i++) {
			if (dict[i].key == '*') result.push(dict[i].value);
		}
		for (var i=result.length; i<key_list.length; i++) {
			var kv = dict.findElement(function(element) {
				return element.key == key_list[i];
			});
			result[i] = (kv == undefined)? default_value[i] : kv.value;
		}
		return result;
	}

}) ();