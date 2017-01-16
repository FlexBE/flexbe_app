//=====================
//	Array
//=====================

Array.prototype.remove = function (element) {
	this.splice(this.indexOf(element), 1);
}

Array.prototype.clone = function() {
	copy = [];
	for(var i=0; i<this.length; ++i)
		copy.push(this[i]);
	return copy;
}

Array.prototype.filter = function(predicate) {
	var filtered_array = [];
	this.forEach(function (element, i) {
		if (predicate(element)) filtered_array.push(element);
	});
	return filtered_array;
}

Array.prototype.findElement = function(predicate) {
	for (var i=0; i<this.length; ++i) {
		if (predicate(this[i])) {
			return this[i];
		}
	}
}

Array.prototype.contains = function(element) {
	return this.indexOf(element) != -1;
}


//=====================
//	String
//=====================

String.prototype.endsWith = function(suffix) {
	return this.slice(-suffix.length) == suffix;
}

String.prototype.startsWith = function (prefix){
	return this.slice(0, prefix.length) == prefix;
}