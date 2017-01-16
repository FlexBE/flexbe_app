IO.ManifestGenerator = new (function() {
	var that = this;

	var names;

	var generateManifestHeader = function() {
		content = "";

		content += "\t<executable package_path=\"" + names.rosnode_name + "." + names.file_name.replace(/\.py$/, "") + "\" class=\"" + names.class_name + "\" />\n";
		content += "\t<tagstring>" + Behavior.getTags() + "</tagstring>\n";
		content += "\t<author>" + Behavior.getAuthor() + "</author>\n";
		content += "\t<date>" + Behavior.getCreationDate() + "</date>\n";
		content += "\t<description>\n";
		content += "\t\t" + Behavior.getBehaviorDescription() + "\n";
		content += "\t</description>\n";
		content += "\n";

		return content;
	}

	var generateManifestContains = function() {
		content = "\t<!-- Contained Behaviors -->\n";

		var states = helper_collectAllStates(Behavior.getStatemachine());
		var contained_behaviors = [];
		for (var i = 0; i < states.length; i++) {
			if (!(states[i] instanceof BehaviorState)) continue;

			var contain_reference = contained_behaviors.findElement(function(element) {
				return element.getStateClass() == states[i].getStateClass();
			});
			if (contain_reference == undefined) {
				contained_behaviors.push(states[i]);
			}
		}
		for (var i=0; i<contained_behaviors.length; ++i) {
			content += '\t<contains name="' + contained_behaviors[i].getBehaviorName() + '" />\n';
		}

		return content;
	}

	var generateManifestParameters = function() {
		content = "\t<!-- Available Parameters -->\n";
		var params = Behavior.getBehaviorParameters();
		if (params.length == 0) return content;

		content += "\t<params>\n";

		for (var i = 0; i < params.length; i++) {
		 	var p = params[i];
		 	content += "\n";
		 	content += '\t\t<param type="' + p.type
		 		+ '" name="' + p.name
		 		+ '" default="' + p.default
		 		+ '" label="' + p.label
		 		+ '" hint="' + p.hint
		 		+ '"';
		 	if (p.type == "enum") {
		 		content += ">\n";
		 		for (var j = 0; j < p.additional.length; j++) {
		 			content += '\t\t\t<option value="' + p.additional[j] + '" />\n';
		 		};
		 		content += "\t\t</param>\n";
		 	} else if (p.type == "numeric") {
		 		content += ">\n";
		 		content += '\t\t\t<min value="' + p.additional.min + '" />\n';
		 		content += '\t\t\t<max value="' + p.additional.max + '" />\n';
		 		content += "\t\t</param>\n";
		 	} else if (p.type == "yaml") {
		 		content += ">\n";
		 		content += '\t\t\t<key name="' + p.additional.key + '" />\n';
		 		content += "\t\t</param>\n";
		 	} else {
		 		content += " />\n";
		 	}
		 }

		content += "\n";
		content += "\t</params>\n";
		content += "\n";

		return content;
	}

	var generateManifestParameter = function() {
		content = "";

		return content;
	}


	this.generateManifest = function() {
		var manifestContent = "";
		names = Behavior.createNames();

		manifestContent += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
		manifestContent += "\n";
		manifestContent += "<behavior name=\"" + names.behavior_name + "\">\n";
		manifestContent += "\n";

		manifestContent += generateManifestHeader();
		manifestContent += "\n";

		manifestContent += generateManifestContains();
		manifestContent += "\n";

		manifestContent += generateManifestParameters();
		manifestContent += "\n";

		manifestContent += "</behavior>";

		return manifestContent;
	}

	var helper_collectAllStates = function(sm) {
		var states = [];

		sm.getStates().forEach(function(element, i) {
			states.push(element);
			if (element instanceof Statemachine)
				helper_collectAllStates(element).forEach(function(state, j) {
					states.push(state);
				});
		});

		return states;
	}

}) ();