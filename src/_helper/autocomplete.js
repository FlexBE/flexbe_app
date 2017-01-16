Autocomplete = new (function() {
	var that = this;

	this.generateList = function(entry, additional) {
		if (entry == undefined || entry == "") return [];
		value_list = [];

		// private variables
		value_list = value_list.concat(Behavior.getPrivateVariables()
		.filter(function(el) {
			return el.key.startsWith(entry);
		}).map(function(el) {
			return {text: el.key, hint: " = " + el.value.replace(/"/g, '&quot;'), fill: el.key};
		}));

		// behavior parameters
		value_list = value_list.concat(Behavior.getBehaviorParameters()
		.filter(function(el) {
			return el.name.startsWith(entry);
		}).map(function(el) {
			return {text: el.name, hint: "parameter", fill: "self." + el.name};
		}));

		// private functions
		value_list = value_list.concat(Behavior.getPrivateFunctions()
		.filter(function(el) {
			return el.name.startsWith(entry);
		}).map(function(el) {
			return {text: el.name, hint: "(" + el.params + ")", fill: "self." + el.name};
		}));

		// additional custom options
		value_list = value_list.concat(additional
		.filter(function(el) {
			return el.text.startsWith(entry);
		}).map(function(el) {
			return {text: el.text, hint: el.hint, fill: el.fill};
		}));

		// Python
		value_list = value_list.concat([
			{text: 'lambda', hint: 'anonymous function', fill: 'lambda x: '},
			{text: 'True', hint: 'boolean', fill: 'True'},
			{text: 'False', hint: 'boolean', fill: 'False'}
		]
		.filter(function(el) {
			return el.text.startsWith(entry);
		}).map(function(el) {
			return {text: el.text, hint: el.hint, fill: el.fill};
		}));

		return value_list;
	}

	this.generateInputUserdata = function(entry, state) {
		if (entry == undefined || state == undefined) return [];
		var value_list = [];
		var already_offered = [];
		var container = state.getContainer();
		var container_input = (container.getStateName == "")?
				Behavior.getDefaultUserdata().map(function (el) { return el.key; }) :
				container.getInputKeys();

		// available userdata
		container.updateDataflow();
		value_list = value_list.concat(container_input
		.map(function(el) {
			already_offered.push(el);
			return {text: el, hint: "available", fill: el};
		}));

		// known output
		var all_output_keys = [];
		for (var i = 0; i < container.getStates().length; i++) {
			if (container.getStates()[i].getStateName() == state.getStateName()) continue;
			all_output_keys = all_output_keys.concat(container.getStates()[i].getOutputMapping());
		}
		value_list = value_list.concat(all_output_keys
		.filter(function(el) {
			var offered = already_offered.contains(el);
			already_offered.push(el);
			return !offered;
		}).map(function(el) {
			var output_states = container.getStates()
				.filter(function(s) { return s.getOutputMapping().contains(el) && s.getStateName() != state.getStateName(); })
				.map(function(s) { return s.getStateName(); })
				.join(", ");
			return {text: el, hint: output_states, fill: el};
		}));

		// default userdata
		value_list = value_list.concat(Behavior.getDefaultUserdata()
		.filter(function(el) {
			return !already_offered.contains(el.key);
		}).map(function(el) {
			already_offered.push(el.key);
			return {text: el.key, hint: (container.getStateName == "")? "available" : "behavior userdata", fill: el.key};
		}));

		return value_list.filter(function (el) { return el.text.startsWith(entry); });
	}

	this.generateOutputUserdata = function(entry, state) {
		if (entry == undefined || state == undefined) return [];
		var value_list = [];
		var already_offered = [];
		var container = state.getContainer();
		var container_input = (container.getStateName == "")?
				Behavior.getDefaultUserdata().map(function (el) { return el.key; }) :
				container.getInputKeys();
		var container_output = (container.getStateName == "")?
				Behavior.getInterfaceOutputKeys() :
				container.getOutputKeys();

		// container output
		container.updateDataflow();
		value_list = value_list.concat(container_output
		.map(function(el) {
			already_offered.push(el);
			return {text: el, hint: "statemachine output", fill: el};
		}));

		// available userdata
		container.updateDataflow();
		value_list = value_list.concat(container_input
		.map(function(el) {
			already_offered.push(el);
			return {text: el, hint: "override", fill: el};
		}));

		// known input
		var all_input_keys = [];
		for (var i = 0; i < container.getStates().length; i++) {
			if (container.getStates()[i].getStateName() == state.getStateName()) continue;
			all_input_keys = all_input_keys.concat(container.getStates()[i].getInputMapping());
		}
		value_list = value_list.concat(all_input_keys
		.filter(function(el) {
			var offered = already_offered.contains(el);
			already_offered.push(el);
			return !offered;
		}).map(function(el) {
			var output_states = container.getStates()
				.filter(function(s) { return s.getOutputMapping().contains(el) && s.getStateName() != state.getStateName(); })
				.map(function(s) { return s.getStateName(); })
				.join(", ");
			return {text: el, hint: output_states, fill: el};
		}));

		// default userdata
		value_list = value_list.concat(Behavior.getDefaultUserdata()
		.filter(function(el) {
			return !already_offered.contains(el.key);
		}).map(function(el) {
			already_offered.push(el.key);
			return {text: el.key, hint: (container.getStateName == "")? "override" : "behavior userdata", fill: el.key};
		}));

		return value_list.filter(function (el) { return el.text.startsWith(entry); });
	}

	this.generateCommandList = function(input, commands) {
		var input = input.split(" ")[0];
		var value_list = [];

		value_list = value_list.concat(commands
		.filter(function(el) {
			return el.desc.startsWith(input);
		}).map(function(el) {
			return {text: el.desc, hint: "", fill: el.desc.split(" ")[0]};
		}));

		return value_list;
	}

	this.getSameFill = function(input, suggestions) {
		var fill = suggestions[0].fill.substr(input.length);
		for (var i = 1; i < suggestions.length; i++) {
			var s = suggestions[i].fill.substr(input.length);
			for (var j = fill.length; j > 0; j--) {
				if (s.startsWith(fill)) break;
				fill = fill.substr(0,j-1);
			}
		}
		return fill;
	}

}) ();