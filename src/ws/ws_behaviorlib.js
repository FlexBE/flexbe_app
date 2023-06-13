WS.Behaviorlib = new (function() {
	var that = this;

	var behaviorlib = [];


	this.getByName = function(behavior_name) {
		return behaviorlib.findElement(function(element) {
			return element.getBehaviorName() == behavior_name;
		});
	}

	this.getByClass = function(class_name) {
		return behaviorlib.findElement(function(element) {
			return element.getStateClass() == class_name;
		});
	}

	this.getBehaviorList = function() {
		list = []
		for (var i=0; i<behaviorlib.length; ++i) {
			//list.push(behaviorlib[i].getBehaviorName());
			list.push(behaviorlib[i]);
		}
		return list.sort(function(a,b) { return a.getBehaviorName().toLowerCase().localeCompare(b.getBehaviorName().toLowerCase()); });
	}

	this.resetLib = function() {
		behaviorlib = [];
	}

	this.addToLib = function(behavior) {
		if (that.getByName(behavior.getBehaviorName()) != undefined) {
			T.logWarn("Behavior "+behavior.getBehaviorName()+" is already defined!");
		}
		behaviorlib.push(behavior);
	}

	this.updateEntry = function(be_entry, callback) {
		IO.BehaviorLoader.loadBehaviorInterface(be_entry.getBehaviorManifest(), function(ifc) {
			if (be_entry.getBehaviorManifest().class_name != ifc.class_name) {
				T.logwarn("Inconsistent class name for: " + be_entry.getBehaviorManifest().class_name + " / " + ifc.class_name);
				return;
			}
			behaviorlib.remove(be_entry);
			behaviorlib.push(new WS.BehaviorStateDefinition(be_entry.getBehaviorManifest(), ifc.smi_outcomes, ifc.smi_input, ifc.smi_output, callback));
		});
	}

}) ();
