WS.StateDefinition = function(state_class, state_desc, state_path, parameters, outcomes, input_keys, output_keys, parameter_values, autonomy, class_vars) {
	var that = this;

	var state_class = state_class;
	var state_desc = state_desc;
	var state_path = state_path;
	var state_package = state_path.split('.')[0];
	var parameters = parameters;
	var outcomes = outcomes;
	var input_keys = input_keys;
	var output_keys = output_keys;
	var default_parameter_values = parameter_values;
	var default_autonomy = autonomy;
	var class_vars;


	this.getStateClass = function() { return state_class; }
	this.getStateDesc = function() { return state_desc.getLong(); }
	this.getShortDesc = function() { return state_desc.getShort(); }
	this.getStatePath = function() { return state_path; }
	this.getStatePackage = function() { return state_package; }
	this.getParameters = function() { return parameters; }
	this.getOutcomes = function() { return outcomes; }
	this.getInputKeys = function() { return input_keys; }
	this.getOutputKeys = function() { return output_keys; }
	this.getDefaultParameterValues = function() { return default_parameter_values; }
	this.getDefaultAutonomy = function() { return default_autonomy; }

	this.getParamDesc = function() { return state_desc.getParams(); }
	this.getInputDesc = function() { return state_desc.getInput(); }
	this.getOutputDesc = function() { return state_desc.getOutput(); }
	this.getOutcomeDesc = function() { return state_desc.getOutcomes(); }

	this.getClassVariables = function() { return class_vars; }

};