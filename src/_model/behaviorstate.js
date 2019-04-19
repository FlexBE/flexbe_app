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

	this.getParameterDefinition = function(param) {
		return behavior_manifest.params.findElement(el => {
			return el.name == param;
		});
	}

	this.getDefaultUserdataValue = function(key) {
		var element = be_definition.getDefaultUserdata().findElement(function(el) {
			return el.key == key;
		});
		return (element != undefined)? element.value : "";
	}
	
};
BehaviorState.prototype = Object.create(State.prototype);