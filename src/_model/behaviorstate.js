BehaviorState = function(be_name, be_definition) {
	State.apply(this, [be_name, be_definition]);
	var that = this;

	var behavior_name = be_definition.getBehaviorName();
	var behavior_manifest = be_definition.getBehaviorManifest();
	var behavior_statemachine = be_definition.cloneBehaviorStatemachine();
	behavior_statemachine.setBehavior(that);

	this.getBehaviorName = function() {
		return behavior_name;
	}

	this.getBehaviorStatemachine = function() {
		return behavior_statemachine;
	}

	this.getBehaviorManifest = function() {
		return behavior_manifest;
	}
	
};
BehaviorState.prototype = Object.create(State.prototype);