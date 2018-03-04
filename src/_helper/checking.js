Checking = new (function() {
	var that = this;

	var python_varname_pattern = /^[a-z_][a-z0-9_]*$/i;
	var python_reserved_list = ["and", "assert", "break", "class", "continue", "def", "del", "elif", 
	         "else", "except", "exec", "finally", "for", "from", "global", "if", 
	         "import", "in", "is", "lambda", "not", "or", "pass", "print", 
	         "raise", "return", "try", "while", "yield"];

	this.checkBehavior = function() {
		var error = that.checkDashboard();
		if (error != undefined) {
			UI.Menu.toDashboardClicked();
			return error;
		}

		error = that.checkStatemachine();
		if (error != undefined) {
			UI.Menu.toStatemachineClicked();
			return error;
		}

		return undefined;
	}

	this.warnBehavior = function() {
		var warnings = [];

		warnings.push.apply(warnings, that.warnDashboard());
		warnings.push.apply(warnings, that.warnStatemachine());

		return warnings;
	}

	this.checkDashboard = function() {
		if (Behavior.getBehaviorName() == "") return "behavior name is not set";
		if (Behavior.getBehaviorDescription() == "") return "behavior description needs to be set";
		if (Behavior.getAuthor() == "") return "author needs to be set";

		// Python problems
		if (Behavior.getBehaviorName().match(/^[0-9]/)) return "behavior name must not start with a numeric character";

		// variables
		var illegal_element = Behavior.getPrivateVariables().findElement(function(el) {
			return !that.isValidPythonVarname(el.key);
		});
		if (illegal_element != undefined) return "private variable " + illegal_element.key + " has illegal name";

		var variable_values = Behavior.getPrivateVariables().map(function(el) { return el.value; });
		for (var i = 0; i < variable_values.length; i++) {
			if (!that.isValidExpressionSyntax(variable_values[i], true)) return "private variable " + Behavior.getPrivateVariables()[i].key + " has illegal value";
		}

		// userdata
		var illegal_element = Behavior.getDefaultUserdata().findElement(function(el) {
			return !that.isValidPythonVarname(el.key);
		});
		if (illegal_element != undefined) return "userdata key " + illegal_element.key + " has illegal name";

		var userdata_values = Behavior.getDefaultUserdata().map(function(el) { return el.value; });
		for (var i = 0; i < userdata_values.length; i++) {
			if (!that.isValidExpressionSyntax(userdata_values[i], true)) return "userdata value to key " + Behavior.getDefaultUserdata()[i].key + " is illegal";
		}

		// parameters
		var beparams = Behavior.getBehaviorParameters();
		for (var i=0; i<beparams.length; i++) {
			p = beparams[i];
			if (p.name == "") return "a parameter has not been named";
			if (p.label == "") return "parameter " + p.name + " has no label";
			if (!['enum', 'text', 'numeric', 'boolean', 'yaml'].contains(p.type)) return "parameter " + p.name + " has illegal type: " + p.type;
			if (p.type == 'enum') {
				if (p.additional.length == 0) return "enum parameter " + p.name + " has no options to choose from";
				if (!p.additional.contains(p.default)) return "enum parameter " + p.name + " has illegal default value: " + p.default;
			}
		}

		// interface
		if (Behavior.getInterfaceOutcomes().length == 0) return "behavior needs at least one outcome";

		var iiks = Behavior.getInterfaceInputKeys();
		for (var i=0; i<iiks.length; i++) {
			k = iiks[i];
			if (Behavior.getDefaultUserdata().findElement(function(el) {
				return el.key == k;
			}) == undefined) return "interface input key " + k + " is not contained in default userdata";
		}

		var ioks = Behavior.getInterfaceOutputKeys();
		for (var i=0; i<ioks.length; i++) {
			k = ioks[i];
			if (Behavior.getDefaultUserdata().findElement(function(el) {
				return el.key == k;
			}) == undefined) return "interface output key " + k + " is not contained in default userdata";
		}

		return undefined;
	}

	this.warnDashboard = function() {
		var warnings = [];
		if (Behavior.getCreationDate() == "") warnings.push("behavior creation date is not set");
		if (Behavior.getTags() == "") warnings.push("behavior has no tags for quicker access");

		return warnings;
	}

	this.checkStatemachine = function() {
		error = that.checkSingleStatemachine(Behavior.getStatemachine());
		if (error != undefined) return error;

		return undefined;
	}

	this.warnStatemachine = function() {
		return that.warnSingleStatemachine(Behavior.getStatemachine());
	}


	this.checkSingleStatemachine = function(statemachine) {
		statemachine.updateDataflow(); // also required by state checking

		var states = statemachine.getStates();
		if (states.length == 0) {
			UI.Statemachine.setDisplayedSM(statemachine);
			return "state machine " + statemachine.getStatePath() + " contains no states";
		}

		if (statemachine.getInitialState() == undefined) {
			UI.Statemachine.setDisplayedSM(statemachine);
			return "state machine " + statemachine.getStatePath() + " has no initial state";
		}

		for (var i = 0; i < states.length; i++) {
			var error_string = undefined;
			if (states[i] instanceof Statemachine) {
				error_string = that.checkSingleStatemachine(states[i]);
				if (error_string != undefined) return error_string;
			}
			// always check as state because state machines are also states in their container
			error_string = that.checkSingleState(states[i]);
			// do not have to perform inner checks on embedded behaviors (-> readonly)

			if (error_string != undefined)  {
				UI.Statemachine.setDisplayedSM(statemachine);
				return error_string;
			}
		}

		return undefined
	}


	this.warnSingleStatemachine = function(statemachine) {
		var warnings = [];
		statemachine.updateDataflow(); // also required by state checking

		var states = statemachine.getStates();
		for (var i = 0; i < states.length; i++) {
			if (states[i] instanceof Statemachine) {
				warnings.push.apply(warnings, that.warnSingleStatemachine(states[i]));
			}
			warnings.push.apply(warnings, that.warnSingleState(states[i]));
		}

		// check output dataflow
		var dataflow = statemachine.getDataflow();
		for (var i = 0; i < dataflow.length; i++) {
			var d = dataflow[i];
			var available_userdata = statemachine.getInputKeys();
			if (statemachine.getStateName() == "") {
				available_userdata = available_userdata.concat(Behavior.getDefaultUserdata().map(function(el) { return el.key; }));
			}
			if ((d.getTo().getStateClass() == ":OUTCOME" || d.getTo().getStateClass() == ":CONDITION") && d.getFrom().getStateName() == "INIT" && !available_userdata.contains(d.getOutcome()))
				warnings.push("container " + statemachine.getStatePath() + " has undefined userdata for output key " + d.getOutcome() + " at outcome " + d.getTo().getStateName());
		}

		return warnings;
	}

	this.checkSingleState = function(state) {
		if (state.getStateName() == "") return "state at path " + state.getStatePath() + " has empty name";

		// parameters
		if (state.getParameters().length > 0) {
			var sparams = state.getParameterValues();
			for (var i = 0; i < sparams.length; i++) {
				if (sparams[i] == "") return "parameter " + state.getParameters()[i] + " of state " + state.getStatePath() + " has empty value";
				if (!that.isValidExpressionSyntax(sparams[i], false)) return "parameter " + state.getParameters()[i] + " of state " + state.getStatePath() + " has invalid value";
			}
		}

		// input keys
		if (state.getInputKeys().length > 0) {
			var imap = state.getInputMapping();
			for (var i = 0; i < imap.length; i++) {
				if (imap[i] == "") return "input key " + state.getInputKeys()[i] + " of state " + state.getStatePath() + " has empty value";
				if (!imap[i].match(python_varname_pattern)) return "input key " + state.getInputKeys()[i] + " of state " + state.getStatePath() + " has invalid value: " + imap[i];
			}
		}

		// output keys
		if (state.getOutputKeys().length > 0) {
			var omap = state.getOutputMapping();
			for (var i = 0; i < omap.length; i++) {
				if (omap[i] == "") return "output key " + state.getOutputKeys()[i] + " of state " + state.getStatePath() + " has empty value";
				if (!omap[i].match(python_varname_pattern)) return "output key " + state.getOutputKeys()[i] + " of state " + state.getStatePath() + " has invalid value: " + omap[i];
			}
		}

		// userdata
		var sm_dataflow = state.getContainer().getDataflow().filter(function(el) {
			return el.getTo().getStateName() == state.getStateName() && el.getFrom().getStateName() == "INIT";
		});
		for (var i = 0; i < sm_dataflow.length; i++) {
			var available_userdata = state.getContainer().getInputKeys();
			if (state.getContainer().getStateName() == "") {
				available_userdata = available_userdata.concat(Behavior.getDefaultUserdata().map(function(el) { return el.key; }));
			}
			if (!available_userdata.contains(sm_dataflow[i].getOutcome())) {
				if (!UI.Statemachine.isDataflow()) UI.Statemachine.toggleDataflow();
				return "input key " + sm_dataflow[i].getOutcome() + " of state " + state.getStatePath() + " could be undefined";
			}
		}

		// outcomes
		if (state.getOutcomesUnconnected().length > 0) return "outcome " + state.getOutcomesUnconnected()[0] + " of state " + state.getStatePath() + " is unconnected";
		if (state.getContainer().isConcurrent()) {
			var outcome_target_list = [];
			var error_string = undefined;
			oc_transitions = state.getContainer().getTransitions().filter(function(t) {
				return t.getFrom().getStateName() == state.getStateName()
					&& t.getTo().getStateClass() == ":CONDITION";
			});
			oc_transitions.forEach(function(t) {
				if (outcome_target_list.contains(t.getTo().getStateName())) {
					error_string = "multiple outcomes of state " + state.getStateName() + " point to the same outcome of a concurrency container";
				} else {
					outcome_target_list.push(t.getTo().getStateName());
				}
			});
			if (error_string != undefined) return error_string;
		}
	}

	this.warnSingleState = function(state) {
		var warnings = [];

		// unused output keys

		return warnings;
	}

	this.isValidExpressionSyntax = function(expr, allow_comment) {
		if (expr.length == 0) return false;

		var opening = ['(', '[', '{', '"', "'"];
		var closing = [')', ']', '}', '"', "'"];

		var close_stack = [];
		var dot_last = false;
		
		for (var i = 0; i < expr.length; i++) {
			var c = expr[i];
			if (dot_last && !c.match(/[a-z_0-9]/i)) return false;
			else dot_last = false;
			if (c == "," && close_stack.length == 0) return false;
			if (close_stack.length > 0 && c == close_stack[close_stack.length - 1]) {
				close_stack.pop();
				continue;
			}
			if (close_stack.length == 0 || (close_stack[close_stack.length - 1] != "'" && close_stack[close_stack.length - 1] != '"')) {
				if (opening.contains(c)) {
					close_stack.push(closing[opening.indexOf(c)]);
					continue;
				}
				if (c == "#") {
					if (allow_comment) break;
					else return false;
				}
				if (c == '.') {
					dot_last = true;
					continue;
				}
				if (closing.contains(c)) return false;
			}
		}
		if (close_stack.length > 0) return false;

		return true;
	}

	this.isValidPythonVarname = function(expr) {
		return expr.match(python_varname_pattern) && !python_reserved_list.contains(expr);
	}

}) ();