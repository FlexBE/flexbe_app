WS.Statelib = new (function() {
	var that = this;

	var statelib = [
		new WS.StateDefinition(":OUTCOME", undefined, "", [], [], [], [], [], [], []),
		new WS.StateDefinition(":CONDITION", undefined, "", [], [], [], [], [], [], []),
		new WS.StateDefinition(":INIT", undefined, "", [], [], [], [], [], [], []),
		new WS.StateDefinition(":CONTAINER", undefined, "", [], [], [], [], [], [], [])
	];


	this.getFromLib = function(state_class) {
		for (var i=0; i<statelib.length; ++i) {
			if (statelib[i].getStateClass() == state_class)
				return statelib[i];
		}
	}

	this.getClassList = function() {
		list = []
		for (var i=0; i<statelib.length; ++i) {
			if (statelib[i].getStateClass().charAt(0) == ":") continue;
			list.push(statelib[i].getStateClass());
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

}) ();