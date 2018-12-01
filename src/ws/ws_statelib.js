WS.Statelib = new (function() {
	var that = this;

	var statelib = [
		new WS.StateDefinition(":OUTCOME", undefined, "", [], [], [], [], [], [], []),
		new WS.StateDefinition(":CONDITION", undefined, "", [], [], [], [], [], [], []),
		new WS.StateDefinition(":INIT", undefined, "", [], [], [], [], [], [], []),
		new WS.StateDefinition(":CONTAINER", undefined, "", [], [], [], [], [], [], [])
	];


	this.getFromLib = function(state_type) {
		for (var i=0; i<statelib.length; ++i) {
			if (state_type == statelib[i].getStateType())
				return statelib[i];
		}
	}

	this.getClassFromLib = function(state_class) {
		for (var i=0; i<statelib.length; ++i) {
			if (state_class == statelib[i].getStateClass())
				return statelib[i];
		}
	}

	this.isClassUnique = function(state_class) {
		var filtered = statelib.filter(state_def => {
			return state_def.getStateClass() == state_class;
		});
		return filtered.length == 1;
	}

	this.getTypeList = function() {
		list = []
		for (var i=0; i<statelib.length; ++i) {
			if (statelib[i].getStateClass().charAt(0) == ":") continue;
			list.push(statelib[i].getStateType());
		}
		return list;
	}

	this.resetLib = function() {
		statelib = [
			new WS.StateDefinition(":OUTCOME", undefined, "", [], [], [], [], [], [], []),
			new WS.StateDefinition(":CONDITION", undefined, "", [], [], [], [], [], [], []),
			new WS.StateDefinition(":INIT", undefined, "", [], [], [], [], [], [], []),
			new WS.StateDefinition(":CONTAINER", undefined, "", [], [], [], [], [], [], [])
		];
	}

	this.addToLib = function(state) {
		statelib.push(state);
	}

	this.updateDef = function(state_def) {
		var state_class = state_def.getStateClass();
		statelib.remove(statelib.findElement(
			(state) => state.getStateClass() == state_class)
		);
		statelib.push(state_def);
	}

}) ();