IO.ManifestParser = new (function() {
	var that = this;

	this.parseManifest = function(content, file_path, python_path) {
		parser = new DOMParser();
		xml = parser.parseFromString(content,"text/xml");

		// structure test
		if (xml.getElementsByTagName("behavior").length != 1
		 || xml.getElementsByTagName("executable").length != 1
		 || xml.getElementsByTagName("executable")[0].getAttribute("package_path") == undefined
		 || xml.getElementsByTagName("executable")[0].getAttribute("package_path").split(".").length != 2
		) return;

		var name = xml.getElementsByTagName("behavior")[0].getAttribute("name");
		var description = (xml.getElementsByTagName("description").length > 0 && xml.getElementsByTagName("description")[0].childNodes.length > 0) ?
			xml.getElementsByTagName("description")[0].childNodes[0].nodeValue.trim().replace(/\s+/, " ") : "";
		var tags = (xml.getElementsByTagName("tagstring").length > 0 && xml.getElementsByTagName("tagstring")[0].childNodes.length > 0) ?
			xml.getElementsByTagName("tagstring")[0].childNodes[0].nodeValue.trim() : "";
		var author = (xml.getElementsByTagName("author").length > 0 && xml.getElementsByTagName("author")[0].childNodes.length > 0) ?
			xml.getElementsByTagName("author")[0].childNodes[0].nodeValue.trim() : "";
		var date = (xml.getElementsByTagName("date").length > 0 && xml.getElementsByTagName("date")[0].childNodes.length > 0)?
			xml.getElementsByTagName("date")[0].childNodes[0].nodeValue.trim()
			: undefined;

		var path = xml.getElementsByTagName("executable")[0].getAttribute("package_path").split(".");
		var rosnode_name = path[0];
		var codefile_name = path[1] + ".py";
		var codefile_path = python_path;
		var class_name = xml.getElementsByTagName("executable")[0].getAttribute("class");

		var params_element = xml.getElementsByTagName("params");
		var param_list = [];
		if (params_element.length > 0) {
			var params = params_element[0].getElementsByTagName("param");
			for (var i = 0; i < params.length; i++) {
				var p = params[i];
				var p_obj = {
					type: p.getAttribute("type"),
					name: p.getAttribute("name"),
					default: p.getAttribute("default"),
					label: p.getAttribute("label"),
					hint: p.getAttribute("hint"),
					additional: undefined
				};
				if (p_obj.type == "enum") {
					p_obj.additional = [];
					var options = p.getElementsByTagName("option");
					for (var j = 0; j < options.length; j++) {
						p_obj.additional.push(options[j].getAttribute("value"));
					}
				} else if (p_obj.type == "numeric") {
					p_obj.additional = {min: undefined, max: undefined};
					p_obj.additional.min = p.getElementsByTagName("min")[0].getAttribute("value");
					p_obj.additional.max = p.getElementsByTagName("max")[0].getAttribute("value");
				} else if (p_obj.type == "yaml") {
					p_obj.additional = {key: undefined};
					p_obj.additional.key = p.getElementsByTagName("key")[0].getAttribute("name");
				}
				param_list.push(p_obj);
			}
		}

		var contains_elements = xml.getElementsByTagName("contains");
		var contains_list = [];
		for (var i = 0; i < contains_elements.length; i++) {
			contains_list.push(contains_elements[i].getAttribute("name"));
		}

		return {
			name: 			name,
			description: 	description,
			tags: 			tags,
			author: 		author,
			date: 			date,
			rosnode_name: 	rosnode_name,
			codefile_name: 	codefile_name,
			codefile_path: 	codefile_path,
			class_name: 	class_name,
			params: 		param_list,
			contains: 		contains_list,
			file_path: 		file_path
		};
	}

}) ();