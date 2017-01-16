Transition = function(from_state, to_state, outcome, autonomy) {
	var that = this;

	var from = from_state;
	var to = to_state;
	var outcome = outcome;
	var autonomy = autonomy;


	this.getFrom = function() {
		return from;
	}
	this.setFrom = function(_from) {
		from = _from;
	}

	this.getTo = function() {
		return to;
	}
	this.setTo = function(_to) {
		to = _to;
	}

	this.getOutcome = function() {
		return outcome;
	}
	this.setOutcome = function(_outcome) {
		outcome = _outcome;
	}

	this.getAutonomy = function() {
		return autonomy;
	}
	this.setAutonomy = function(_autonomy) {
		autonomy = _autonomy;
	}

	/* Deprecated */
	this.getDrawnFromState = function() {
		return UI.Statemachine.getDrawnState(this.from);
	}

	/* Deprecated */
	this.getDrawnToState = function() {
		if (this.to)
			return UI.Statemachine.getDrawnState(this.to);
		else
			return UI.Statemachine.mouse_pos;
	}

};