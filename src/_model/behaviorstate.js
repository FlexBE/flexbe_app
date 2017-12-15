BehaviorState = function(be_name, be_definition, be_defkeys) {
	State.apply(this, [be_name, be_definition]);
	var that = this;

	var behavior_name = be_definition.getBehaviorName();
	var behavior_manifest = be_definition.getBehaviorManifest();
	var behavior_statemachine = be_definition.cloneBehaviorStatemachine();
	behavior_statemachine.setBehavior(that);
	var default_keys = be_defkeys;

	this.getBehaviorName = function() {
		return behavior_name;
	}

	this.getBehaviorStatemachine = function() {
		return behavior_statemachine;
	}

	this.getBehaviorManifest = function() {
		return behavior_manifest;
	}

	this.getDefaultKeys = function() {
		return default_keys;
	}

	this.addDefaultKey = function(new_key) {
		if (default_keys.contains(new_key)) return;
		default_keys.push(new_key);
	}

	this.removeDefaultKey = function(key) {
		if (!default_keys.contains(key)) return;
		default_keys.remove(key);
	}

	this.getDefaultValue = function(key) {
		var element = be_definition.getDefaultUserdata().findElement(function(el) {
			return el.key == key;
		});
		return (element != undefined)? element.value : "";
	}
	
};
BehaviorState.prototype = Object.create(State.prototype);