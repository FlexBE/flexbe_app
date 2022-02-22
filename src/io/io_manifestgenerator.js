IO.ManifestGenerator = new (function() {
	var that = this;

	var names;

	var generateManifestHeader = function() {
		content = "\t\"executable\": {";
		content += "\n";
		content += "\t\t\"package_path\": \"" + names.rosnode_name + "." + names.file_name.replace(/\.py$/, "") + "\",";
		content += "\n";
		content += "\t\t\"class\": \"" + names.class_name + "\" \n";
		content += "\t},\n";
		content += "\n";
		content += "\t\"author\": \"" + Behavior.getAuthor() + "\",\n";
		content += "\t\"date\": \"" + Behavior.getCreationDate() + "\",\n";
		content += "\t\"tagstring\": \"" + Behavior.getTags() + "\",\n";
		content += "\t\"description\": \"\"\""
		content += "\t\t" + Behavior.getBehaviorDescription() + "\"\"\","
		return content;
	}

	var generateManifestContains = function() {
		content = "\t# Contained behaviors\n";

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

		content += "\t\"contained_behaviors\": [\n";
		for (var i=0; i<contained_behaviors.length; ++i) {
			content += "\t\t" + contained_behaviors[i].getBehaviorName() + ",\n";
		}
		content += "\t],";

		return content;
	}

	var generateManifestParameters = function() {
		content = "\t# Available parameters\n";

		var params = Behavior.getBehaviorParameters();
		if (params.length == 0) return content;

		content += "\t\"params\": [ \n";

		for (var i = 0; i < params.length; i++) {
			content += "\t\t{";
			content += "\n";

		 	var p = params[i];
		 	content += '\t\t\t\"type\": \"' + p.type + "\",\n";
			content += '\t\t\t\"name\": \"' + p.name + "\",\n";
			content += '\t\t\t\"default\": \"' + p.default + "\",\n";
			content += '\t\t\t\"label\": \"' + p.label + "\",\n";
			content += '\t\t\t\"hint\": \"' + p.hint + "\",\n";
			content += '\t\t\t\"additional\": { \n';

		 	if (p.type == "enum") {
		 		content += ">\n";
		 		for (var j = 0; j < p.additional.length; j++) {
		 			content += '\t\t\t\t \"option value:\" \"' + p.additional[j] + '\",\n';
		 		};
		 		content += "\t\t</param>\n";
		 	} else if (p.type == "numeric") {
		 		content += '\t\t\t\t\"min\": \"' + p.additional.min + '\",\n';
		 		content += '\t\t\t\t\"max\": \"' + p.additional.max + '\",\n';
		 	} else if (p.type == "yaml") {
		 		content += ">\n";
		 		content += '\t\t\t\t \"key name=\": \"' + p.additional.key + '\",\n';
		 	} else {
		 		content += " \n";
		 	}
			content += "\t\t\t},\n";
			content += "\t\t},\n";
		 }

		content += "\n";
		content += "\t],";
		return content;
	}


	var generateManifestParameter = function() {
		content = "";

		return content;
	}

	this.generateManifest = function() {
		var manifestContent = "";
		names = Behavior.createNames();

		var dict_name = names.behavior_name.toLowerCase().split(' ').join('_')

		manifestContent += dict_name + " = {";
		manifestContent += "\n";
		manifestContent += "\t\"name\": \"" + names.behavior_name + "\",";
		manifestContent += "\n";

		manifestContent += generateManifestHeader();
		manifestContent += "\n";
		manifestContent += "\n";

		manifestContent += generateManifestContains();
		manifestContent += "\n";
		manifestContent += "\n";

		manifestContent += generateManifestParameters();
		manifestContent += "\n";

		manifestContent += "}";

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
