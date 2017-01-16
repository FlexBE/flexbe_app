WS.StateMachineDefinition = function(outcomes, input_keys, output_keys) {
	var that = this;

	var autonomy = [];
	for (var i = 0; i < outcomes.length; ++i) {
		autonomy.push(-1);
	};

	this.__proto__ = new WS.StateDefinition(":STATEMACHINE", undefined, "", [], 
		outcomes, input_keys, output_keys, [], autonomy, []);

	outcomes = that.getOutcomes();

	this.getOutcomes = function() { return outcomes; }

	this.addOutcome = function(oc) { outcomes.push(oc); };
	this.removeOutcome = function(oc) { outcomes.remove(oc); };
};