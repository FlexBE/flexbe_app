IO.StateParser = new (function() {
	var that = this;

	this.parseState = function(content, import_path) {
		// Patterns
			// Inherits from EventState, state description is directly below class definition.
		var name_desc_pattern = /class (\w+)\(EventState\):(?:\n\r?\s+(?:'''|""")\n?\r?((?:\s*(?:.*?)\n?\r?\s*)*?)(?:'''|"""))?/i;
			// Returns all params as list
		var param_pattern = /def __init__\(self, ?([^)]+)\):/i;
			// Extracts parameters of super class call, such as outcomes.
		var super_pattern = /super\(.*\)\.__init__\(((?:.|\s)*?)\)\n/i;
			// Has two matches: 1) key 2) list
		var list_pattern = /(\w+)=\[(.*)\]/i;
			// Finds conditions to be monitored
		var monitor_pattern = /^\s*self\.monitor\(.+?,\s*["']([A-Z_0-9]+)["']\)/igm;

		var name_desc_results = content.match(name_desc_pattern);
		if (name_desc_results == null) return;
		var state_class = name_desc_results[1];

		var state_desc = "";
		var argument_doc = [];
		var last_argument = undefined;
		if (name_desc_results[2] != undefined) {
			var line_split = name_desc_results[2].trim().replace(/[\n\r]+/g, "\n\r").split(/[\n\r]+/g);
			for (var i = 0; i < line_split.length; i++) {
				var l = line_split[i].trim();
				if (l.match(/^(--|>#|#>)/)) {
					if (last_argument != undefined) argument_doc.push(last_argument);
					var arg_split = l.match(/^(--|>#|#>)\s+([^\s]+)\s+([^\s]+)\s+(.+)$/);
					if (arg_split == null || arg_split.length < 5) {
						T.logWarn('Entry in ' + state_class + ' does not fit documentation format: ' + l);
					} else {
						last_argument = {
							symbol: arg_split[1],
							name: arg_split[2],
							type: arg_split[3],
							desc: arg_split[4]
						};
					}
				} else if (l.startsWith("<=")) {
					if (last_argument != undefined) argument_doc.push(last_argument);
					var arg_split = l.match(/^(<=)\s+([^\s]+)\s+(.+)$/);
					if (arg_split == null || arg_split.length < 4) {
						T.logWarn('Entry in ' + state_class + ' does not fit documentation format: ' + l);
					} else {
						last_argument = {
							symbol: arg_split[1],
							name: arg_split[2],
							type: "",
							desc: arg_split[3]
						};
					}
				} else if (last_argument != undefined) {
					last_argument['desc'] += " " + l;
				} else {
					if (state_desc != "") state_desc += " ";
					state_desc += l;
				}
			}
			if (last_argument != undefined) argument_doc.push(last_argument);

		} 
		if (state_desc.match(/^\s*$/)) {
			state_desc = "[no documentation]";
		}
		var state_doc = new WS.Documentation(state_desc);
		for (var i = 0; i < argument_doc.length; i++) {
			var a = argument_doc[i];
			state_doc.addDescription(a['symbol'], a['name'], a['type'], a['desc']);
		}

		var class_var_content = content.split(param_pattern)[0].split(/(?:"""|''')/);
		class_var_content = class_var_content[class_var_content.length - 1].split(/[\n\r]+/g);
		var class_vars = [];
		for (var i = 0; i < class_var_content.length; i++) {
			var l = class_var_content[i];
			var class_var_match = l.match(/^\s*([A-Z_0-9]+)\s*=\s*(.+)$/i);
			if (class_var_match != null) {
				class_vars.push({name: class_var_match[1], value: class_var_match[2]});
			}
		}
		

		var param_results = content.match(param_pattern);
		var state_params = [];
		var state_params_values = [];
		if (param_results != null) {
			var p = helper_splitOnTopCommas("("+param_results[1].replace(/\s/g, "")+")");
			p.forEach(function(param, i) {
				if (param.indexOf("=") != -1) {
					var splitted = param.split("=");
					state_params[i] = splitted[0];
					state_params_values[i] = splitted[1];
				} else {
					state_params[i] = param;
					state_params_values[i] = '';
				}
			})
		}

		var super_result = content.match(super_pattern);
		var super_string = super_result[1];
		var super_list = super_string.replace(/\s/g, "").split(/,(?=(outcomes|input_keys|output_keys))/g);
		var state_outcomes = [], state_autonomy = [], state_input = [], state_output = [];
		for(var i=0; i<super_list.length; ++i) {
			var opt = super_list[i].split("=");
			if (opt.length != 2) continue;
			if (opt[0] == "outcomes") {
				var outcome_list = opt[1].startsWith("[")? helper_splitOnTopCommas(opt[1]) : [opt[1]];
				for (var j=0; j<outcome_list.length; ++j) {
					var quotes_match = outcome_list[j].match(/^["'](.*)["']/);
					if (quotes_match != null) {
						state_outcomes.push(quotes_match[1]);
						state_autonomy.push(0);
					} else if (state_params.contains(outcome_list[j])) {
						state_outcomes.push("$"+outcome_list[j]);
					} else {
						state_input.push("$?outcomes");
						state_params.push("?outcomes");
						state_params_values.push('');
						state_doc.addDescription('--', '?outcomes', "list", "This parameter has been added automatically. The outcomes of this state couldn't be parsed, please enter them manually.");
					}
				}
			} else if (opt[0] == "input_keys") {
				var input_list = opt[1].startsWith("[")? helper_splitOnTopCommas(opt[1]) : [opt[1]];
				for (var j=0; j<input_list.length; ++j) {
					var quotes_match = input_list[j].match(/^["'](.*)["']/);
					if (quotes_match != null) {
						state_input.push(quotes_match[1]);
					} else if (state_params.contains(input_list[j])) {
						state_input.push("$"+input_list[j]);
					} else {
						state_input.push("$?input_keys");
						state_params.push("?input_keys");
						state_params_values.push('');
						state_doc.addDescription('--', '?input_keys', "list", "This parameter has been added automatically. The input keys of this state couldn't be parsed, please enter them manually.");
					}
				}
			} else if (opt[0] == "output_keys") {
				var output_list = opt[1].startsWith("[")? helper_splitOnTopCommas(opt[1]) : [opt[1]];
				for (var j=0; j<output_list.length; ++j) {
					var quotes_match = output_list[j].match(/^["'](.*)["']/);
					if (quotes_match != null) {
						state_output.push(quotes_match[1]);
					} else if (state_params.contains(output_list[j])) {
						state_output.push("$"+output_list[j]);
					} else {
						state_output.push("$?output_keys");
						state_params.push("?output_keys");
						state_params_values.push('');
						state_doc.addDescription('--', '?output_keys', "list", "This parameter has been added automatically. The output keys of this state couldn't be parsed, please enter them manually.");
					}
				}
			}
		}

		content.replace(monitor_pattern, function(s, key) {
			state_outcomes.push(key.toLowerCase());
			state_autonomy.push(2);
			return s;
		});

		return new WS.StateDefinition(
			state_class,
			state_doc,
			import_path,
			state_params,
			state_outcomes,
			state_input,
			state_output,
			state_params_values,
			state_autonomy,
			class_vars
		)
	}

	var helper_splitOnTopCommas = function(code) {
		// code has to start with anyone of ([{ and will be cut off at matching }])
		// result contains all top-level comma separated fields
		var depth = 0;
		var inside_single_quotes = false;
		var inside_double_quotes = false;
		var last_split = 1;
		var result = [];

		// extract and split params
		for (var i=0; i<code.length; i++) {
			var c = code[i];
			if (!inside_double_quotes && c == "'") inside_single_quotes = !inside_single_quotes;
			if (!inside_single_quotes && c == '"') inside_double_quotes = !inside_double_quotes;
			// ignore string content
			if (inside_single_quotes || inside_double_quotes) continue;
			// track depth
			if (c == "(" || c == "[" || c == "{") depth += 1;
			if (c == ")" || c == "]" || c == "}") depth -= 1;

			if (depth == 0 || depth == 1 && c == ",") {
				result.push(code.substring(last_split, i).trim());
				last_split = i + 1; // skip comma
			}

			if (depth == 0) break;
		}

		return result;
	}

}) ();