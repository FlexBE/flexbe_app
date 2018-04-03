Statemachine = function(sm_name, sm_definition) {
	State.apply(this, [sm_name, sm_definition]);
	var that = this;

	var states = [];
	var transitions = [];
	transitions.push(new Transition(new State("INIT", WS.Statelib.getFromLib(":INIT")), undefined, "", 0));
	var dataflow = [];

	var concurrent = false;
	var priority = false;

	var initial_state = undefined;
	var sm_outcomes = [];

	var addSMOutcome = function(outcome) {
		var outcome_state = new State(outcome + (concurrent? ('#' + sm_outcomes.length) : ''), WS.Statelib.getFromLib(concurrent? ":CONDITION" : ":OUTCOME"));
		outcome_state.setPosition({x: 30 + sm_outcomes.length * 100, y: UI.Statemachine.getR().height / 2});
		sm_outcomes.push(outcome_state);
		outcome_state.setContainer(that);
	}
	var generateSMOutcomes = function() {
		sm_outcomes = [];
		for (var i = 0; i < sm_definition.getOutcomes().length; i++) {
			addSMOutcome(sm_definition.getOutcomes()[i]);
		}
	}
	generateSMOutcomes();

	var clearTransitions = function() {
		states.forEach(that.removeConnectedTransitions);
	}

	// States
	this.getInitialState = function() {
		return initial_state;
	}
	this.setInitialState = function(_initial_state) {
		initial_state = _initial_state;
		var init_trans = that.getInitialTransition();
		if (init_trans != undefined) {
			init_trans.setTo(initial_state);
		} else {
			T.debugWarn("Could not find initial transition.");
		}
	}
	
	this.getStateByName = function(name) {
		for(var i=0; i<states.length; ++i) {
			if (states[i].getStateName() == name)
				return states[i];
		}
		//T.debugWarn("State '" + name + "' not found in " + that.getStateName());
	}
	this.getStateByPath = function(path) {
		var path_elements = path.split("/");

		if (path_elements[0] != that.getStateName()) {
			T.debugWarn("Path '" + path + "' does not match to " + that.getStateName());
			return undefined;
		}

		if (path_elements.length == 1) {
			T.debugWarn("Invalid path: " + path);
			return undefined;
		}

		if (path_elements.length == 2)
			return that.getStateByName(path_elements[1]);

		var child = that.getStateByName(path_elements[1]);
		if (child instanceof BehaviorState) {
			child = child.getBehaviorStatemachine();
		}
		return child.getStateByPath(path.slice(that.getStateName().length + 1));
	}
	this.traverseStates = function(_filter) {
		var result = [];
		for(var i=0; i<states.length; ++i) {
			if (states[i] instanceof Statemachine) {
				result = result.concat(states[i].traverseStates(_filter));
				continue;
			} else if (states[i] instanceof BehaviorState) {
				continue;
			}
			if (_filter(states[i])) {
				result.push(states[i]);
			}
		}
		return result;
	}

	this.addState = function(state) {
		states.push(state);
		state.setContainer(that);
	}
	this.removeState = function(state) {
		states.remove(state);
		state.setContainer(undefined);
		if (initial_state != undefined && initial_state.getStateName() == state.getStateName())
			initial_state = undefined;

		// remove connected transitions
		that.removeConnectedTransitions(state);
	}

	// Transitions
	this.getInitialTransition = function() {
		return transitions.findElement(function (element) {
			return element.getOutcome() == "" && element.getFrom().getStateName() == "INIT";
		});
	}

	this.hasTransition = function(transition) {
		return transitions.findElement(function(element) {
			return element.getFrom().getStateName() == transition.getFrom().getStateName()
				&& element.getOutcome() == transition.getOutcome();
		}) != undefined;
	}

	this.addTransition = function(transition) {
		if (that.hasTransition(transition)) {
			T.debugWarn("Trying to add already existing transition from state '" + transition.getFrom().getStateName() + "', outcome '" + transition.getOutcome() + "'");
			return;
		}
		transitions.push(transition);
		transition.getFrom().connect(transition.getOutcome());
	}

	this.tryDuplicateOutcome = function(outcome) {
		outcome_states = sm_outcomes.filter(function(state) {
			return state.getStateName().startsWith(outcome);
		});
		transitions.forEach(function(transition) {
			if (outcome_states.contains(transition.getTo())) {
				outcome_states.remove(transition.getTo());
			}
		});
		if (outcome_states.length == 0) {
			addSMOutcome(outcome);
		}
	}

	this.removeTransitionObject = function(transition) {
		transitions.remove(transition);
		transition.getFrom().unconnect(transition.getOutcome());
	}

	this.removeTransitionFrom = function(state, outcome) {
		var trans = transitions.findElement(function(element) {
			return element.getFrom() == state && element.getOutcome() == outcome;
		});
		if (trans != undefined) {
			transitions.remove(trans);
		}
	}
	this.removeConnectedTransitions = function(state) {
		var to_remove = transitions.filter(function (element) {
			return element.getFrom() == state || element.getTo() == state;
		});
		to_remove.forEach(function (element, i) {
			if (element.getOutcome() == "" && element.getFrom().getStateName() == "INIT") {
				that.setInitialState(undefined);
			} else {
				that.removeTransitionObject(element);
			}
		});
	}

	// Userdata
	this.getDataflow = function() {
		return dataflow;
	}

	this.updateDataflow = function() {
		dataflow = [];
		states.forEach(function(state) {
			var added_keys = []
			state.getInputMapping().forEach(function(key, i) {
				if (added_keys.contains(key)) return;
				if (state instanceof BehaviorState && state.getDefaultKeys().contains(state.getInputKeys()[i])) return;
				added_keys.push(key);
				addDataEdgeForPredecessors(state, state, key, []);
			});
		});
		sm_outcomes.forEach(function(outcome) {
			that.getOutputKeys().forEach(function(key) {
				addDataEdgeForPredecessors(outcome, outcome, key, []);
			});
		});
	}

	var addDataEdgeForPredecessors = function(state, target, key, checked) {
		if (concurrent) {
			var init = that.getInitialTransition().getFrom();
			// in concurrency, userdata always needs to be given from container keys
			dataflow.push(new Transition(init, target, key, 0));
		} else {
			transitions.forEach(function(trans) {
				if (trans.getTo() == undefined || trans.getTo().getStateName() != state.getStateName()) return;
				if (trans.getFrom().getStateName() == "INIT") {
					dataflow.push(new Transition(trans.getFrom(), target, key, 0));
				} else if (!checked.contains(trans.getFrom().getStateName())) {
					checked.push(trans.getFrom().getStateName());
					if (trans.getFrom().getOutputMapping().contains(key)) {
						dataflow.push(new Transition(trans.getFrom(), target, key, 0));
					} else {
						addDataEdgeForPredecessors(trans.getFrom(), target, key, checked);
					}
				}
			});
		}
	}


	// Interface
	this.getSMOutcomes = function() {
		return sm_outcomes;
	}
	this.setSMOutcomes = function(_sm_outcomes) {
		sm_outcomes = _sm_outcomes;
	}

	this.getSMOutcomeByName = function(name) {
		for(var i=0; i<sm_outcomes.length; ++i) {
			if ((sm_outcomes[i].getStateName() == name)
			|| (concurrent && name.indexOf('#') == -1 && sm_outcomes[i].getStateName().startsWith(name)))
				return sm_outcomes[i];
		}
		T.debugWarn("Outcome '" + name + "' not found in " + that.getStateName());
	}

	this.addOutcome = function(outcome) {
		sm_definition.addOutcome(outcome);
		var outcome_state = new State(outcome, WS.Statelib.getFromLib(concurrent? ":CONDITION" : ":OUTCOME"));
		outcome_state.setPosition({x: 30 + sm_outcomes.length * 100, y: UI.Statemachine.getR().height / 2});
		outcome_state.setContainer(that);
		sm_outcomes.push(outcome_state);
		that.getOutcomes().push(outcome);
		that.getOutcomesUnconnected().push(outcome);
		that.getAutonomy().push(-1);
	}

	this.removeOutcome = function(outcome) {
		// remove transition away
		if (that.getContainer() != undefined) {
			that.getContainer().removeTransitionFrom(that, outcome);
		}
		// remove outcome object
		var old_element = sm_outcomes.findElement(function(element) {
			return element.getStateName() == outcome;
		});
		sm_outcomes.remove(old_element);

		// remove transitions to
		that.removeConnectedTransitions(old_element);

		// remove outcome
		that.getOutcomes().remove(outcome);
		if (that.getOutcomesUnconnected().contains(outcome))
			that.getOutcomesUnconnected().remove(outcome);
		else
			that.getOutcomesConnected().remove(outcome);

		sm_definition.removeOutcome(outcome);
	}

	this.updateOutcome = function(outcome_old, outcome_new) {
		var oc_element = sm_outcomes.findElement(function(element) {
			return element.getStateName() == outcome_old;
		});
		oc_element.setStateName(outcome_new);
		that.getOutcomes().remove(outcome_old);
		that.getOutcomes().push(outcome_new);
	}

	this.isPriority = function() {
		return priority;
	}

	this.setPriority = function(new_priority) {
		priority = new_priority;
	}

	this.isConcurrent = function() {
		return concurrent;
	}

	this.setConcurrent = function(new_concurrent) {
		concurrent = new_concurrent;
		
		clearTransitions();
		generateSMOutcomes();
	}

	this.getConditions = function() {
		var conditions = {
			outcomes: [],
			transitions: []
		}
		transitions.forEach(function(t) {
			if (t.getOutcome() == "") return;
			if (conditions.outcomes.contains(t.getTo().getStateName())) {
				var idx = conditions.outcomes.indexOf(t.getTo().getStateName());
				conditions.transitions[idx].push([t.getFrom().getStateName(), t.getOutcome()]);
			} else {
				conditions.outcomes.push(t.getTo().getStateName());
				conditions.transitions.push([[t.getFrom().getStateName(), t.getOutcome()]]);
			}
		});
		return conditions;
	}

	this.setConditions = function(conditions) {
		var additional_outcomes = [];
		for (var i = 0; i < conditions.outcomes.length; i++) {
			var o = conditions.outcomes[i];
			var t = conditions.transitions[i];
			if (additional_outcomes.contains(o)) {
				addSMOutcome(o);
				t.forEach(function(so) {
					that.addTransition(new Transition(that.getStateByName(so[0]), sm_outcomes[sm_outcomes.length-1], so[1], 0));
				});
			} else {
				o_label = sm_outcomes.findElement(function(oc) {
					return oc.getStateName().startsWith(o + '#');
				});
				t.forEach(function(so) {
					that.addTransition(new Transition(that.getStateByName(so[0]), o_label, so[1], 0));
				});
				additional_outcomes.push(o);
			}
		}
		additional_outcomes.forEach(addSMOutcome);
	}

	//
	//	DEPRECATED
	//
		
	this.getStates = function() {
		//T.debugWarn("DEPRECATED: " + "getStates");
		return states;
	}
	this.setStates = function(_states) {
		T.debugWarn("DEPRECATED: " + "setStates");
		states = _states;
	}

	this.getTransitions = function() {
		//T.debugWarn("DEPRECATED: " + "getTransitions");
		return transitions;
	}
	this.setTransitions = function(_transitions) {
		T.debugWarn("DEPRECATED: " + "setTransitions");
		transitions = _transitions;
	}

};
Statemachine.prototype = Object.create(State.prototype);