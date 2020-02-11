IO.StateParser = new (function() {
	var that = this;
	var os = require('os');
	var spawn = require('child_process').spawn;
	var crypto = require('crypto');
	var md5 = str => crypto.createHash('md5').update(str).digest('hex');

////////////////////////////////
// BEGIN Python implementation
	var impl = `
import sys
import inspect
import json
from flexbe_core import EventState
# for some reason need iter(..) call as in https://bugs.python.org/issue26290
for data in iter(sys.stdin.readline, ""):
	request = json.loads(data)
	state_defs = []
	result = {'id': request['id'], 'state_defs': state_defs}
	try:
		pkg = __import__(request['import_path'], fromlist=[request['import_path']])
		def is_state(member):
			return (inspect.isclass(member) and
					member.__module__ == pkg.__name__ and
					issubclass(member, EventState))
		for name, cls in inspect.getmembers(pkg, is_state):
			state_def = dict()
			state_def['state_class'] = cls.__name__
			state_def['state_doc'] = inspect.getdoc(cls)
			argspec = inspect.getargspec(cls.__init__)
			args = [arg for arg in argspec.args if arg != 'self']
			argdefs = [repr(default) for default in list(argspec.defaults or [])]
			state_def['state_params'] = args
			state_def['state_params_values'] = [''] * (len(args) - len(argdefs)) + argdefs
			def __event_init(*args, **kwargs):
				state_def['state_outcomes'] = kwargs.get('outcomes', [])
				state_def['state_autonomy'] = [0] * len(state_def['state_outcomes'])
				state_def['state_input'] = kwargs.get('input_keys', [])
				state_def['state_output'] = kwargs.get('output_keys', [])
				raise NotImplementedError()  # prevent further instantiation to avoid side-effects
			EventState.__init__ = __event_init
			try:
				cls(*args)  # pass variable names for resolving symbols later
			except NotImplementedError:  # this error type is expected
				pass  # we do nothing because state_def has been updated already
			except Exception as e:  # any other error is passed onwards
				raise Exception(
					"Cannot instantiate state '%s' to determine interface, "
					"consider removing any code before 'super' in '__init__'. "
					"Error: %s" % (cls.__name__, str(e)))
			state_def['class_vars'] = [n for n, t in cls.__dict__.items()
				if not inspect.isfunction(t) and not n.startswith('__')]
			state_defs.append(state_def)
	except ImportError as e:
		sys.stderr.write("Failed to import " + request['import_path'] + " (" + str(e) + ") ")
		sys.stderr.flush()
	except Exception as e:
		sys.stderr.write("Failed to process " + request['import_path'] + " (" + str(e) + ") ")
		sys.stderr.flush()
	sys.stdout.write(json.dumps(result))
	sys.stdout.flush()
`;
// END Python implementation
//////////////////////////////

	var i = 0;
	var loader = undefined;
	var buffer = "";
	var parseCallbacks = [];

	var spawnLoader = function() {
		loader = spawn('python', ['-c', impl]);
		loader.stdout.on('data', (data) => {
			buffer += data;
			var try_parse = true;
			while (try_parse) {
				try_parse = false;
				try {
					var [obj, idx] = json_parse_raw(buffer);
					if (obj == null) obj = undefined;
					if (idx != 0) {
						buffer = buffer.slice(idx);
						try_parse = true;
						// process result
						var entry = parseCallbacks.findElement(element => {
							return element['id'] == obj['id'];
						})
						if (entry != undefined) {
							parseCallbacks.remove(entry);
							entry['callback'](obj['state_defs']);
						}
					}
				} catch (err) {
					try_parse = false;
					console.log('State Parser Error:');
					console.log(err);
					if (err.hasOwnProperty('name') && err.name == "SyntaxError" && err.hasOwnProperty('at')) {
						buffer.slice(err.at);
						try_parse = true;
					}
				}
			}
		});
		loader.stderr.on('data', (data) => {
			T.logWarn("[State parser] " + data);
		});
		loader.on('close', () => {
			loader = undefined;
		});
	}

	this.close = function() {
		if (loader == undefined) return;
		loader.stdin.end();
		loader.kill('SIGKILL');
		loader = undefined;
	}

	this.parseState = function(content, import_path, callback) {
		if (UI.Settings.getStateParser() == 'regex') {
			parseStateRegex(content, import_path, callback);
		} else if (UI.Settings.getStateParser() == 'python') {
			parseStatePython(content, import_path, callback);
		} else {
			T.logError('Unknown state parser: ' + UI.Settings.getStateParser());
		}
	}

	var parseStateRegex = function(content, import_path, callback) {
		// Patterns:
		// Class inherits from EventState, state description is in the docstring.
		var name_desc_pattern = /class (\w+)\(EventState\):(?:\n\r?\s+(?:'''|""")\n?\r?((?:\s*(?:.*?)\n?\r?\s*)*?)(?:'''|"""))?/i;
		// Return all params as list
		var param_pattern = /def __init__\(self, ?([^)]+)\):/i;
		// Extract parameters of super class call, such as outcomes.
		var super_pattern = /super\(.*\)\.__init__\(((?:.|\s)*?)\)/i;

		var name_desc_results = content.match(name_desc_pattern);
		if (name_desc_results == null) {
			callback(undefined);
			return;
		}
		var state_class = name_desc_results[1];
		var state_doc = parseDocumentation(name_desc_results[2] || "");

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
						state_outcomes.push("$?outcomes");
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

		callback(new WS.StateDefinition(
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
		));
	}

	var parseStatePython = function(content, import_path, callback) {
		if (loader == undefined) spawnLoader();
		var id = md5(import_path + content + i++);

		var parseCallback = function(state_defs) {
			if (state_defs.length > 0) {
				var state_def = state_defs[0];  // only support one state per file
				if (typeof state_def['state_outcomes'] == "string") {
					state_def['state_outcomes'] = ['$' + state_def['state_outcomes']];
					state_def['state_autonomy'] = [];
				}
				if (typeof state_def['state_input'] == "string")
					state_def['state_input'] = ['$' + state_def['state_input']];
				if (typeof state_def['state_output'] == "string")
					state_def['state_output'] = ['$' + state_def['state_output']];
				callback(new WS.StateDefinition(
					state_def['state_class'],
					parseDocumentation(state_def['state_doc']),
					import_path,
					[].concat(state_def['state_params'] || []),
					[].concat(state_def['state_outcomes'] || []),
					[].concat(state_def['state_input'] || []),
					[].concat(state_def['state_output'] || []),
					[].concat(state_def['state_params_values'] || []),
					[].concat(state_def['state_autonomy'] || []),
					[].concat(state_def['class_vars'] || [])
				));
			} else {
				callback(undefined);
			}
		}
		parseCallbacks.push({
			id: id,
			callback: parseCallback
		});
		loader.stdin.write(JSON.stringify({
			id: id,
			import_path: import_path,
			content: content
		}) + os.EOL);
	}

	var parseDocumentation = function(docstring) {
		if (typeof docstring != "string" || docstring == "")
			return new WS.Documentation("[no documentation]");
		var state_desc = "";
		var argument_doc = [];
		var last_argument = undefined;
		var line_split = docstring.trim().replace(/[\n\r]+/g, "\n\r").split(/[\n\r]+/g);
		for (var i = 0; i < line_split.length; i++) {
			var l = line_split[i].trim();
			if (l.match(/^(--|>#|#>)/)) {
				if (last_argument != undefined) argument_doc.push(last_argument);
				var arg_split = l.match(/^(--|>#|#>)\s+([^\s]+)\s+([^\s]+)\s+(.+)$/);
				if (arg_split == null || arg_split.length < 5) {
					T.logWarn('Entry does not fit documentation format: ' + l);
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
					T.logWarn('Entry does not fit documentation format: ' + l);
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

		if (state_desc.match(/^\s*$/)) {
			state_desc = "[no documentation]";
		}
		var state_doc = new WS.Documentation(state_desc);
		for (var i = 0; i < argument_doc.length; i++) {
			var a = argument_doc[i];
			state_doc.addDescription(a['symbol'], a['name'], a['type'], a['desc']);
		}
		return state_doc;
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