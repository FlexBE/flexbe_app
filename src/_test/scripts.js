Scripts = new (function() {
	var that = this;

	this.viewStates = function(sm) { that.viewFilteredStates(sm, ""); }
	this.viewAllStates = function() { that.viewStates(Behavior.getStatemachine()); }
	this.viewAllStatesFiltered = function(f) { that.viewFilteredStates(Behavior.getStatemachine(), f); }

	this.viewFilteredStates = function(sm, filter) {
		if (sm.getStateName().indexOf(filter) != -1) {
			console.log(sm.getStatePath());
			console.log(sm);
		}

		if(sm instanceof BehaviorState)
			sm = sm.getBehaviorStatemachine();
		if (sm instanceof Statemachine)
			sm.getStates().forEach(function (s) { that.viewFilteredStates(s, filter); }); }

	this.findStateUsage = function(class_name) {
		var searchFunction = function(sm, pred) {
			var result = [];
			sm.getStates().forEach(function(s) {
				if (pred(s)) result.push(s);
				if (s instanceof Statemachine) {
					result = result.concat(searchFunction(s, pred));
				}
			});
			return result;
		}
		T.clearLog();
		T.show();
		var total = 0;
		WS.Behaviorlib.getBehaviorList().forEach(function (bn) {
			var bsm = bn.cloneBehaviorStatemachine();
			var states = searchFunction(bsm, function(s) { return s.getStateClass() == class_name; });
			if (states.length > 0) {
				if (total == 0) {
					T.logInfo("Found the following uses of state class " + class_name + ":");
				}
				T.logInfo(bn.getBehaviorName() + " (" + states.length + "x)");
				states.forEach(function(s) { T.logInfo("&nbsp;&nbsp;" + s.getStatePath()); });
			}
			total += states.length;
		});
		if (total == 0) {
			T.logInfo("Did not find any usage of state class " + class_name + ".");
		}
	}

}) ();
