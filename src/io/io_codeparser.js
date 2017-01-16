IO.CodeParser = new (function() {
	var that = this;

	// Patterns

		// [1] - manual import code
	var manual_import_pattern_begin = "# \[MANUAL_IMPORT\]";
	var manual_import_pattern_end = "# \[\/MANUAL_IMPORT\]";
		// [1] - manual init code
	var manual_init_pattern_begin = "# \[MANUAL_INIT\]";
	var manual_init_pattern_end = "# \[\/MANUAL_INIT\]";
		// [1] - manual create code
	var manual_create_pattern_begin = "# \[MANUAL_CREATE\]";
	var manual_create_pattern_end = "# \[\/MANUAL_CREATE\]";
		// [1] - manual func code
	var manual_func_pattern_begin = "# \[MANUAL_FUNC\]";
	var manual_func_pattern_end = "# \[\/MANUAL_FUNC\]";
		// to remove comment
	var comment_manual_pattern = /\s*# (Additional imports|Additional initialization code|Additional creation code|Private functions) can be added inside the following tags\n\r?/ig;

		// Inherits from Behavior, meant to also split the file
		// [1] - global indentation, [2] - class name
	var class_def_pattern = /^(\s*)class\s+(\w+)\s*\(Behavior\):$/im;
		// Finds first documentation block
		// [1] - content of the block
	var doc_block_pattern = /'''\n\r?([^']*)\n\r?\s*'''/i;
		// Finds all import lines
		// [1] - from ..., [3] - import ... (comma separated list)
	var import_line_pattern = /^\s*from ((\w|\.)+) import (.+)\s*$/igm;
		// [1] - date of behavior creation
	var behavior_date_pattern = /Created on\s*(.+)/i;
		// [1] - author of the behavior
	var behavior_author_pattern = /@author:\s*(.+)/i;

		// Finds constructor definition, to split
	var init_def_pattern = /^\s*def __init__\(self\):$/im;
		// Finds create definition, to split
	var create_def_pattern = /^\s*def create\(self\):$/im;
		// Finds function definitions for private functions
		// [1] - function name, [2] - param string (without "self")
	var function_def_pattern = /^\s*def (\w+)\(self(?:, ([^)]+))?\):$/img;

		// splits the comments from the rest of the init section (comments are at the end)
	var comments_split_pattern = /^\s*# Behavior comments:$/img;
		// gets a comment note
		// [1] - isImportant, [2] - x position, [3] - y position, [4] - container path, [5] - content
	var comment_note_pattern = /^\s*# (O|!) ([0-9]+) ([0-9]+) ([^\n]*)\n\r?\s*# ([^\n]+)$/igm;

		// To remove the super constructor call
	var super_pattern = /^\s*super\(\w+, self\)\.__init__\(\)$/im;
		// Gets the behavior name
		// [1] - name
	var behavior_name_pattern = /^\s*self\.name ?= ?['"]([^'"]*)['"]\s*$/im;
		// Removes all comments
	var remove_comment_pattern = /#.*$/img;
		// Removes parameters (use from manifest)
	var remove_parameter_pattern = /^\s*self\.add_parameter\(/igm;
		// Removes contains (use from manifest)
	var remove_contains_pattern = /^\s*self\.add_behavior\(/igm;
		// Removes all empty lines (compare with trim() to remove last line)
	var remove_empty_lines_pattern = /^\s*\n\r?/igm;

		// [1] - name of the root statemachine variable
	var return_sm_pattern = /^\s*return (\w+)/im;
		// Matches all state machine definitions (root and subs)
		// [1] - list of outcome positions, [2] - variable name of the sm, [3] - parameter collection (outcomes, input_keys, output_keys)
	var sm_definition_pattern = /^(?:\s*# ((?:x:-?\d+ y:-?\d+(?:, )?)+))?\s*(\w+) = OperatableStateMachine\(([^)]+)\)/img;
		// [1] - list of outcome positions, [2] - variable name of the cc, [3] - parameter collection (outcomes, input_keys, output_keys, conditions)
	var cc_definition_pattern = /^(?:\s*# ((?:x:-?\d+ y:-?\d+(?:, )?)+))?\s*(\w+) = ConcurrencyContainer\(((?:.|\n)*?)\s\]\)\n/img;
		// [1] - list of outcome positions, [2] - variable name of the sm, [3] - parameter collection (outcomes, input_keys, output_keys)
	var pc_definition_pattern = /^(?:\s*# ((?:x:-?\d+ y:-?\d+(?:, )?)+))?\s*(\w+) = PriorityContainer\(([^)]+)\)/img;
		// Matches all variable definitions (including sm! remove those first)
		// [1] - variable name, [2] - variable value
	var var_definition_pattern = /^\s*(\w+) = (.+)/img;
		// Matches all userdata definitions. Add Tabs and root sm name before.
		// [1] - userdata key, [2] - userdata value
	var userdata_definition_pattern = /\.userdata\.(\w+) = (.+)/img;
		// Finds all container openings, can be used to split the code in sections per container
		// [1] - name of the following container
	var container_split_pattern = /^\s*with (\w+):/img;
		// Finds the start of a state, search for the end of it by counting parenthesis pairs
		// [1] - x position if set, [2] - y position if set, [3] - additional custom parameter data if set
	var state_begin_pattern = /^(?:\s*# x:(-?\d+) y:(-?\d+)(?: (\{.+\}))?)?\s*OperatableStateMachine\.add/im;

		// [1] - content without quotes
	var string_quotes_pattern = /^["'](.*)["']/;
		// [1] - name of the state class
	var state_class_pattern = /^\s*(\w+)\(/;
		// [1] - name of the behavior class
	var state_behavior_pattern = /^\s*self\.use_behavior\((\w+)(?:, ['"](?:[^'"]*)['"])?\)/;
		// [1] - kind of data (transitions/autonomy/remapping), [2] - comma separated list of values including surrounding braces
	var state_interface_pattern = /(transitions|autonomy|remapping)\s*=\s*(\{[^}]*\})/;


	var need_manual_imports = false;
	var need_manual_init = false;
	var need_manual_create = false;
	var need_manual_func = false;


	this.extractManual = function(code) {
		var manual = ["", "", "", ""];

		var import_split_begin = code.split(manual_import_pattern_begin);
		if (import_split_begin.length == 2) {
			var import_split_end = import_split_begin[1].split(manual_import_pattern_end);
			if (import_split_end.length != 2) throw "inconsistent tag [MANUAL_IMPORT]"
			var import_result = import_split_end[0];
			if (import_result != "") {
				manual[0] = import_result;
			} else {
				need_manual_imports = true;
			}
			code = code.replace(manual_import_pattern_begin + import_split_end[0] + manual_import_pattern_end, "");
		} else {
			//T.logWarn("No manual section for additional imports found.");
		}

		var init_split_begin = code.split(manual_init_pattern_begin);
		if (init_split_begin.length == 2) {
			var init_split_end = init_split_begin[1].split(manual_init_pattern_end);
			if (init_split_end.length != 2) throw "inconsistent tag [MANUAL_INIT]";
			var init_result = init_split_end[0];
			if (init_result != "") {
				manual[1] = init_result;
			} else {
				need_manual_inits = true;
			}
			code = code.replace(manual_init_pattern_begin + init_split_end[0] + manual_init_pattern_end, "");
		} else {
			//T.logWarn("No manual section for behavior initialization found.");
		}

		var create_split_begin = code.split(manual_create_pattern_begin);
		if (create_split_begin.length == 2) {
			var create_split_end = create_split_begin[1].split(manual_create_pattern_end);
			if (create_split_end.length != 2) throw "inconsistent tag [MANUAL_CREATE]";
			var create_result = create_split_end[0];
			if (create_result != "") {
				manual[2] = create_result;
			} else {
				need_manual_creates = true;
			}
			code = code.replace(manual_create_pattern_begin + create_split_end[0] + manual_create_pattern_end, "");
		} else {
			//T.logWarn("No manual section for behavior state machine creation found.");
		}

		var func_split_begin = code.split(manual_func_pattern_begin);
		var func_defs = [];
		if (func_split_begin.length == 2) {
			var func_split_end = func_split_begin[1].split(manual_func_pattern_end);
			if (func_split_end.length != 2) throw "inconsistent tag [MANUAL_FUNC]";
			var func_result = func_split_end[0];
			if (func_result != "") {
				manual[3] = func_result;
				func_result.replace(function_def_pattern, function(all, name, params) {
					func_defs.push({key: name, value: params});
				});
			} else {
				need_manual_funcs = true;
			}
			code = code.replace(manual_func_pattern_begin + func_split_end[0] + manual_func_pattern_end, "");
		} else {
			//T.logWarn("No manual section for private functions found.");
		}

		code = code.replace(comment_manual_pattern, "");

		return {
			new_code: code,
			manual_import: manual[0],
			manual_init: manual[1],
			manual_create: manual[2],
			manual_func: manual[3],
			func_defs: func_defs		// [key, value]
		};
	}


	var parseTopSection = function(code) {
		// parse documentation
		var comment_result = code.match(doc_block_pattern);
		var behavior_date = "";
		var behavior_author = "";
		if (comment_result != null) {
			var comment = comment_result[1];
			var date_result = comment.match(behavior_date_pattern);
			behavior_date = (date_result != null)? date_result[1] : "";
			var author_result = comment.match(behavior_author_pattern);
			behavior_author = (author_result != null)? author_result[1] : "";
		}
		code = code.replace(doc_block_pattern, "");

		// parse import statements
		var additional_imports = "";
		if (need_manual_imports) {
			
		}

		return {
			behavior_date: behavior_date,
			behavior_author: behavior_author,
			additional_imports: additional_imports
		}
	}


	var parseInitSection = function(code) {
		// remove undesired content
		code = code.replace(doc_block_pattern, "");
		code = code.replace(super_pattern, "");

		// get behavior name
		var behavior_name_result = code.match(behavior_name_pattern);
		var behavior_name = (behavior_name_result != null)? behavior_name_result[1] : "";
		code = code.replace(behavior_name_pattern, "");

		var comments_split = code.split(comments_split_pattern);
		code = comments_split[0];
		var comments = [];
		if (comments_split.length > 1) {
			var comments_code = comments_split[1];
			var comments_def = comments_code.split(comment_note_pattern);
			for (var i = 6; i < comments_def.length; i += 6) {
				comments.push({
					important: comments_def[i-5] == "!",
					pos_x: parseInt(comments_def[i-4]),
					pos_y: parseInt(comments_def[i-3]),
					container: comments_def[i-2],
					content: comments_def[i-1].replace(/\|n/g, "\n")
				});
			}
		}
		
		code = code.replace(remove_comment_pattern, "");

		// parse additional init code
		var additional_init = "";
		if (need_manual_init) {
			code = code.replace(remove_parameter_pattern, "");
			code = code.replace(remove_contains_pattern, "");
			code = code.replace(remove_empty_lines_pattern, "");
			code = code.trim();
			additional_init = code;
			T.logInfo("Generated missing manual init code section.");
		}

		return {
			behavior_name: behavior_name,
			additional_init: additional_init,
			comments: comments
		}
	}


	var parseCreateSection = function(code, only_interface) {
		// get root sm var name
		var root_sm_name_result = code.match(return_sm_pattern);
		if (root_sm_name_result == null) throw "could not identify root state machine";
		var root_sm_name = root_sm_name_result[1];
		code = code.replace(return_sm_pattern, "");

		// get all sm definitions
		var sm_defs = [];
		code = code.replace(sm_definition_pattern, function(s, positions, name, params) {
			var pos = [];
			if (positions != undefined && positions != "") {
				positions.split(", ").forEach(function (element) {
					var xy = element.replace("x:", "").replace("y:", "").split(" ");
					pos.push({x: parseInt(xy[0]), y: parseInt(xy[1])});
				});
			}
			sm_defs.push({sm_name: name, sm_params: parseSMIDefinition(params), oc_positions: pos, sm_type: 'statemachine'});
			return "";
		});
		// get all cc definitions
		code = code.replace(cc_definition_pattern, function(s, positions, name, params) {
			var pos = [];
			if (positions != undefined && positions != "") {
				positions.split(", ").forEach(function (element) {
					var xy = element.replace("x:", "").replace("y:", "").split(" ");
					pos.push({x: parseInt(xy[0]), y: parseInt(xy[1])});
				});
			}
			sm_defs.push({sm_name: name, sm_params: parseSMIDefinition(params), oc_positions: pos, sm_type: 'concurrency'});
			return "";
		});
		// get all pc definitions
		code = code.replace(pc_definition_pattern, function(s, positions, name, params) {
			var pos = [];
			if (positions != undefined && positions != "") {
				positions.split(", ").forEach(function (element) {
					var xy = element.replace("x:", "").replace("y:", "").split(" ");
					pos.push({x: parseInt(xy[0]), y: parseInt(xy[1])});
				});
			}
			sm_defs.push({sm_name: name, sm_params: parseSMIDefinition(params), oc_positions: pos, sm_type: 'priority'});
			return "";
		});
		// get root sm definition
		var root_sm_def = sm_defs.findElement(function (element) {
			return element.sm_name == root_sm_name;
		});

		if (only_interface)
			return {
				smi_outcomes: root_sm_def.sm_params.outcomes,
				smi_input: root_sm_def.sm_params.input_keys,
				smi_output: root_sm_def.sm_params.output_keys
			};

		// get all further variable definitions
		var var_defs = [];
		code = code.replace(var_definition_pattern, function(s, name, value) {
			// according to Behavior class
			var_defs.push({key: name, value: value});
			return "";
		});

		// get all userdata definitions
		var ud_defs = [];
		code = code.replace(userdata_definition_pattern, function(s, key, value) {
			// according to Behavior class
			ud_defs.push({key: key, value: value});
			return "";
		});

		// split into sm parts
		var sm_states = [];
		var sm_parts = code.split(container_split_pattern);
		if (sm_parts.length == 1) throw "did not find any state machines"
		for (var i=0; i<(sm_parts.length-1)/2; i++) {
			var idx = i * 2 + 1;
			sm_states.push({
				sm_name: sm_parts[idx],
				sm_states: parseStates(sm_parts[idx+1])
			});
		}

		return {
			var_defs: var_defs,
			ud_defs: ud_defs,
			root_sm_name: root_sm_name,
			smi_outcomes: root_sm_def.sm_params.outcomes,
			smi_input: root_sm_def.sm_params.input_keys,
			smi_output: root_sm_def.sm_params.output_keys,
			sm_defs: sm_defs,
			sm_states: sm_states
		}
	}


	var parseSMIDefinition = function(sm_params) {
		var result = {
			outcomes: [],
			input_keys: [],
			output_keys: [],
			conditions: undefined
		};

		var param_list = sm_params.replace(/([()[\],])\s+/g, "$1").split(/,(?=(outcomes|input_keys|output_keys|conditions))/g);
		for(var i=0; i<param_list.length; ++i) {
			var opt = param_list[i].split("=");
			if (opt.length != 2) continue;
			if (opt[0] == "outcomes") {
				var outcome_list = opt[1].replace(/^\[/, "").replace(/\]$/, "").split(",");
				for (var j=0; j<outcome_list.length; ++j) {
					if (outcome_list[j] == "") continue;
					var quote_match = outcome_list[j].match(/^["'](.*)["']/);
					if (quote_match != null) {
						result.outcomes.push(quote_match[1]);
					} else {
						result.outcomes.push(outcome_list[j]);
					}
				}
			} else if (opt[0] == "input_keys") {
				var input_list = opt[1].replace(/^\[/, "").replace(/\]$/, "").split(",");
				for (var j=0; j<input_list.length; ++j) {
					if (input_list[j] == "") continue;
					var quote_match = input_list[j].match(/^["'](.*)["']/);
					if (quote_match != null) {
						result.input_keys.push(quote_match[1]);
					} else {
						result.input_keys.push(input_list[j]);
					}
				}
			} else if (opt[0] == "output_keys") {
				var output_list = opt[1].replace(/^\[/, "").replace(/\]$/, "").split(",");
				for (var j=0; j<output_list.length; ++j) {
					if (output_list[j] == "") continue;
					var quote_match = output_list[j].match(/^["'](.*)["']/);
					if (quote_match != null) {
						result.output_keys.push(quote_match[1]);
					} else {
						result.output_keys.push(output_list[j]);
					}
				}
			} else if (opt[0] == "conditions") {
				result.conditions = {
					outcomes: [],
					transitions: []
				};
				var condition_list = opt[1].replace(/^\[\(/, "").replace(/\)$/, "").split(")]),(");
				for (var j=0; j<condition_list.length; ++j) {
					var ot_split = condition_list[j].replace(/\)\]$/, "").split(",[(");
					result.conditions.outcomes.push(ot_split[0].replace(/'/g, ""));
					transitions_list = [];
					var t_split = ot_split[1].split("),(");
					for (var k=0; k<t_split.length; ++k) {
						var so_split = t_split[k].split("','");
						transitions_list.push([so_split[0].replace("'", ""),so_split[1].replace("'", "")]);
					}
					result.conditions.transitions.push(transitions_list);
				}
			}
		}

		if (result.outcomes.length == 0) {
			T.logWarn("A statemachine has no outcomes, make sure to add at least one.");
		}

		return result;
	}


	var parseStates = function(code) {
		var code_splitted = code.split(state_begin_pattern);
		if (code_splitted.length == 1) throw "a container does not contain any states"

		var state_list = [];

		for (var i=4; i<code_splitted.length; i+=4) {
			var state_param_result = parseStateParams(helper_splitOnTopCommas(code_splitted[i]));
			if (code_splitted[i-3] != undefined)
				state_param_result.state_pos_x = parseInt(code_splitted[i-3]);
			if (code_splitted[i-2] != undefined)
				state_param_result.state_pos_y = parseInt(code_splitted[i-2]);
			if (code_splitted[i-1] != undefined) {
				state_param_result.parameter_values = state_param_result.parameter_values.concat(
					helper_splitOnTopCommas(code_splitted[i-1]).map(function(el) {
						var el_split = el.split(/ = /);
						return {key: el_split[0], value: el_split[1]};
					})
				);
			}
			state_list.push(state_param_result);
		}

		return state_list;
	}

	var parseStateParams = function(params) {
		// get name
		var state_name = helper_removeQuotes(params[0]);

		// get state class and params
		var state_class = "";
		var state_type = "state";
		var parameter_values = [];
		var class_result = params[1].match(state_class_pattern);
		if (class_result != null) {
			state_class = class_result[1];
			var params_split = helper_splitOnTopCommas(params[1].replace(state_class, ""));
			params_split.forEach(function(element, i) {
				var keyvalue = helper_splitKeyValue(element, "=");
				if (keyvalue != undefined && !element.startsWith("lambda")) {
					parameter_values.push(keyvalue);
				} else {
					//T.logWarn("Could not allocate parameter of state " + state_name + ": " + element);
					parameter_values.push({
						key: '*',
						value: element
					});
				}
			});
		} else {
			var behavior_use_result = params[1].match(state_behavior_pattern);
			if (behavior_use_result != null) {
				state_class = behavior_use_result[1];
				state_type = "behavior";
				parameter_values = [];
			} else {
				state_class = params[1];
				parameter_values = undefined;
				state_type = "container";
			}
		}

		// get further parameters
		var transitions = [];
		var autonomy = [];
		var remapping = [];
		for(var i=2; i<params.length; i++) {
			var param_result = params[i].match(state_interface_pattern);
			if (param_result == null) {
				T.logWarn("Found strange parameter of state " + state_name + ": " + params[i]);
				continue;
			}
			// transitions
			if (param_result[1] == "transitions") {
				var transition_list = helper_splitOnTopCommas(param_result[2]);
				for (var j=0; j<transition_list.length; j++) {
					var transition_kv = helper_splitKeyValue(transition_list[j].trim(), ":");
					if (transition_kv != undefined) {
						transitions.push({
							outcome: helper_removeQuotes(transition_kv.key),
							target: helper_removeQuotes(transition_kv.value)
						});
					} else {
						T.logWarn("Could not parse transition of state " + state_name + ": " + transition_list[j].trim());
					}
				}
			}
			// autonomy
			else if (param_result[1] == "autonomy") {
				var autonomy_list = helper_splitOnTopCommas(param_result[2]);
				for (var j=0; j<autonomy_list.length; j++) {
					var autonomy_kv = helper_splitKeyValue(autonomy_list[j].trim(), ":");
					if (autonomy_kv != undefined) {
						autonomy.push({
							key: helper_removeQuotes(autonomy_kv.key),
							value: helper_autonomyToInt(autonomy_kv.value)
						});
					} else {
						T.logWarn("Could not parse autonomy of state " + state_name + ": " + autonomy_list[j].trim());
					}
				}
			}
			// remapping
			else if (param_result[1] == "remapping") {
				var remapping_list = helper_splitOnTopCommas(param_result[2]);
				for (var j=0; j<remapping_list.length; j++) {
					var remapping_kv = helper_splitKeyValue(remapping_list[j].trim(), ":");
					if (remapping_kv != undefined) {
						remapping.push({
							key: helper_removeQuotes(remapping_kv.key),
							value: helper_removeQuotes(remapping_kv.value),
						});
					} else {
						T.logWarn("Could not parse remapiing of state " + state_name + ": " + remapping_list[j].trim());
					}
				}
			}
		}

		return {
			state_name: state_name,
			state_class: state_class,
			state_type: state_type,
			state_pos_x: 30,
			state_pos_y: 40,
			parameter_values: parameter_values,		// [key, value]
			autonomy: autonomy,						// [key, value]
			remapping: remapping,					// [key, value]
			transitions_from: transitions			// [outcome, target]
		};
	}


	this.parseCode = function(code) {
		var extract_result = that.extractManual(code);
		code = extract_result.new_code;

		// find class definition
		var class_name_result = code.match(class_def_pattern);
		if (class_name_result == null) throw "behavior class definition could not be found";
		var class_name = class_name_result[2];

		// parse top section
		var code_class_split = code.split(class_def_pattern);
		// [0] - before
		// [1] - capt group indentation
		// [2] - capt group class name
		// [3] - after
		var top_result = parseTopSection(code_class_split[0]);

		// parse behavior description
		if (code_class_split.length != 4) throw "behavior class content could not be separated";

		var desc_result = code_class_split[3].match(doc_block_pattern);
		var behavior_description = (desc_result != null)? desc_result[1].trim() : "";

		// split into relevant methods
		var code_init_split = code_class_split[3].split(init_def_pattern);
		if (code_init_split.length == 1) throw "behavior constructor definition could not be found";
		var code_init = code_init_split[1].split(/\tdef/)[0];

		var code_create_split = code_class_split[3].split(create_def_pattern);
		if (code_create_split.length == 1) throw "behavior state machine creation section could not be found";
		var code_create = code_create_split[1].split(/\tdef/)[0];

		// parse init section
		var init_result = parseInitSection(code_init);

		// parse create section
		var create_result = parseCreateSection(code_create, false);

		// parse additional functions



		// write data
		var manual_import = (top_result.additional_imports != "")?
			extract_result.manual_import
				+ "\n# Added additional imports\n"
				+ top_result.additional_imports :
			extract_result.manual_import;

		var manual_init = (init_result.additional_init != "")?
			extract_result.manual_init
				+ "\n\t\t# Added additional init code\n\t\t"
				+ init_result.additional_init :
			extract_result.manual_init;


		//console.log(create_result);

		return {
			behavior_name: 			init_result.behavior_name,
			behavior_description: 	behavior_description,
			author: 				top_result.behavior_author,
			creation_date: 			top_result.behavior_date,
			behavior_comments: 		init_result.comments,

			manual_code: 			{
									manual_import: 	manual_import,
									manual_init: 	manual_init,
									manual_create: 	extract_result.manual_create,
									manual_func: 	extract_result.manual_func
									},

			private_variables: 		create_result.var_defs,			// [key, value]
			default_userdata: 		create_result.ud_defs,			// [key, value]
			private_functions: 		extract_result.func_defs,		// [key, value]

			smi_outcomes: 			create_result.smi_outcomes,		// string
			smi_input: 				create_result.smi_input,		// string
			smi_output: 			create_result.smi_output,		// string

			root_sm_name: 			create_result.root_sm_name,
			sm_defs: 				create_result.sm_defs,
			sm_states: 				create_result.sm_states
		}

	}

	this.parseSMInterface = function(code) {
		var extract_result = that.extractManual(code);
		code = extract_result.new_code;

		// parse top section
		var code_class_split = code.split(class_def_pattern);
		// [0] - before
		// [1] - capt group indentation
		// [2] - capt group class name
		// [3] - after
		if (code_class_split.length != 4) throw "behavior class content could not be separated";
		var class_name = code_class_split[2];

		var code_create_split = code_class_split[3].split(create_def_pattern);
		if (code_create_split.length == 1) throw "behavior state machine creation section could not be found";
		var code_create = code_create_split[1].split(/\tdef/)[0];

		// parse create section
		var create_result = parseCreateSection(code_create, true);

		return {
			class_name: 	class_name,

			smi_outcomes: 	create_result.smi_outcomes,		// string
			smi_input: 		create_result.smi_input,		// string
			smi_output: 	create_result.smi_output,		// string
		}
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


	var helper_removeQuotes = function(quoted_string) {
		var match_result = quoted_string.match(string_quotes_pattern);
		if (match_result != null) {
			return match_result[1];
		} else {
			T.logWarn("Missing quotes for " + quoted_string);
			return quoted_string;
		}
	}

	var helper_splitKeyValue = function(element, separator) {
		var element_split = element.split(separator);
		if (element_split.length > 1) {
			// required if value contains separator itself:
			var value_string = element.replace(element_split[0]+separator, "");
			return {
				key: element_split[0].trim(),
				value: value_string.trim()
			};
		}
		return undefined;
	}

	var helper_autonomyToInt = function(autonomy) {
		if (autonomy.match(/^-?\d+$/) != null)	return autonomy;
		if (autonomy == "Autonomy.Off")			return 0;
		if (autonomy == "Autonomy.Low")			return 1;
		if (autonomy == "Autonomy.High")		return 2;
		if (autonomy == "Autonomy.Full")		return 3;
		return -1;
	}

}) ();
