IO.ManifestGenerator = new (function() {
	var that = this;

	var names;

	var generateManifestHeader = function() {
		content = "";

		content += "    <executable package_path=\"" + names.rosnode_name + "." + names.file_name.replace(/\.py$/, "") + "\" class=\"" + names.class_name + "\" />\n";
		content += "    <tagstring>" + Behavior.getTags() + "</tagstring>\n";
		content += "    <author>" + Behavior.getAuthor() + "</author>\n";
		content += "    <date>" + Behavior.getCreationDate() + "</date>\n";
		content += "    <description>\n";
		var lines = Behavior.getBehaviorDescription().split("\n")
		for (var i = 0; i < lines.length; i++) {
			content += "        " + lines[i] + "\n";
		}
		content += "    </description>\n";
		content += "\n";

		return content;
	}

	var generateManifestContains = function() {
		content = "    <!-- Contained Behaviors -->\n";

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
			content += '    <contains name="' + contained_behaviors[i].getBehaviorName() + '" />\n';
		}

		return content;
	}

	var generateManifestParameters = function() {
		content = "    <!-- Available Parameters -->\n";
		var params = Behavior.getBehaviorParameters();
		if (params.length == 0) return content;

		content += "    <params>\n";

		for (var i = 0; i < params.length; i++) {
			var p = params[i];
			content += "\n";
			content += '        <param type="' + p.type
				+ '" name="' + p.name
				+ '" default="' + p.default
				+ '" label="' + p.label
				+ '" hint="' + p.hint
				+ '"';
			if (p.type == "enum") {
				content += ">\n";
				for (var j = 0; j < p.additional.length; j++) {
					content += '            <option value="' + p.additional[j] + '" />\n';
				};
				content += "        </param>\n";
			} else if (p.type == "numeric") {
				content += ">\n";
				content += '            <min value="' + p.additional.min + '" />\n';
				content += '            <max value="' + p.additional.max + '" />\n';
				content += "        </param>\n";
			} else if (p.type == "yaml") {
				content += ">\n";
				content += '            <key name="' + p.additional.key + '" />\n';
				content += "        </param>\n";
			} else {
				content += " />\n";
			}
		 }

		content += "\n";
		content += "    </params>\n";
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