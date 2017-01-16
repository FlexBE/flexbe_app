VarSolver = new (function() {
	var that = this;

	this.resolveVar = function(target_var, deep_resolve) {
		// split if list
		if (target_var.match(/^\[.*\]$/)) {
			var target_var_list = target_var.replace(/^\[/, "").replace(/\]$/, "").replace(/ /g, "").split(",");
			try {
				var resolved_var_list = []
				target_var_list.forEach(function(element, i) {
					var new_element = that.resolveVar(element, deep_resolve);
					resolved_var_list.push(new_element);
					if (new_element === false) throw "param not found";
				});
			} catch (err) {
				return false;
			}
			return [].concat.apply([], resolved_var_list); // flattens the list
		}

		// check for string
		if (target_var.match(/^["'].*["']$/)) {
			return target_var;
		}

		// check for numeric
		if (target_var.match(/^-?[0-9]*(.[0-9]+)?$/)) {
			return target_var;
		}

		// check for boolean
		if (target_var.match(/^True|False$/)) {
			return target_var;
		}

		// resolve
		var referenced_parameter = Behavior.getPrivateVariables().findElement(function(element) {
			return element.key == target_var;
		});

		if (referenced_parameter != undefined) {
			if (deep_resolve)
				return that.resolveVar(referenced_parameter.value, true);
			else
				return referenced_parameter.value;
		} else {
			return false;
		}

	}

	this.resolveAsList = function(target_var) {
		var resolved = that.resolveVar(target_var, true);
		if (!(resolved instanceof Array)) {
			var val = resolved;
			resolved = [];
			if (!(val === false) && val != "")
				resolved.push(val);
		}
		return resolved;
	}

	this.getStringValues = function(target_var, force) {
		var resolved = that.resolveAsList(target_var);
		var result = [];
		if (resolved.length == 1 && resolved[0] == "") {
			return result;
		}
		for (var i = 0; i < resolved.length; i++) {
			var match_result = resolved[i].match(/^["'](.*)["']/);
			if (match_result != null) {
				result.push(match_result[1]);
			} else if (force) {
				return false;
			}
		}
		return result;
	}

}) ();