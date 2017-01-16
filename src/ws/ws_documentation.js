WS.Documentation = function(description) {
	var that = this;

	var desc_long = description;
	var desc_sentence_split = description.split(". ");
	var desc_short = desc_sentence_split[0] + (desc_sentence_split.length > 1? "." : "");

	var params = [];	// name, type, desc
	var input = [];		// name, type, desc
	var output = [];	// name, type, desc
	var outcomes = [];	// name, desc

	this.getLong = function() {
		return desc_long;
	}

	this.getShort = function() {
		return desc_short;
	}

	this.addDescription = function(symbol, name, type, desc) {
		switch (symbol) {
			case '--': params.push({name: name, type: type, desc: desc}); break;
			case '>#': input.push({name: name, type: type, desc: desc}); break;
			case '#>': output.push({name: name, type: type, desc: desc}); break;
			case '<=': outcomes.push({name: name, desc: desc}); break;
			default:
				T.debugWarn("Documentation symbol " + symbol + " unknown!");
		}
	}

	this.getParams = function() {
		return params;
	}
	this.getInput = function() {
		return input;
	}
	this.getOutput = function() {
		return output;
	}
	this.getOutcomes = function() {
		return outcomes;
	}

};