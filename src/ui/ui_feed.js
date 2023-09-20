UI.Feed = new (function() {
	var that = this;

	var requestLatestVersion = function(callback) {
		var xhr = new XMLHttpRequest();
		xhr.open("GET", "https://api.github.com/repos/mojin-robotics/flexbe_app/tags", true);
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4) {
				if (xhr.responseText != "") {
					var resp = JSON.parse(xhr.responseText);
					var found = false;
					for (var i=0; i<resp.length; i++) {
						if (resp[i].name.match(/^\d+\.\d+\.\d+$/) != null) {
							callback(resp[i].name);
							found = true;
							break;
						}
					}
					if (!found) callback(undefined);
				} else {
					callback(undefined);
				}
			}
		}
		xhr.send();
	}

	var displayVersionIndicator = function(latest_version_label) {
		var current_version_label = UI.Settings.getVersion();
		var status_element = document.getElementById("flexbe_version_status");

		if (latest_version_label == undefined) {
			status_element.setAttribute("src", "img/version_offline.png");
			status_element.setAttribute("title", "Unable to retrieve latest version.");
			console.log("Version: " + current_version_label + " (unknown release version)");
			return;
		}

		var current_version = current_version_label.split(".");
		var latest_version = latest_version_label.split(".");

		current_version = parseInt(current_version[0]) * 1000 * 1000 + parseInt(current_version[1]) * 1000 + parseInt(current_version[2]);
		latest_version = parseInt(latest_version[0]) * 1000 * 1000 + parseInt(latest_version[1]) * 1000 + parseInt(latest_version[2]);

		if (current_version < latest_version) {
			displayPredefinedMessage("msg_notify_update");
			status_element.setAttribute("src", "img/version_old.png");
			status_element.setAttribute("title", "New release available!");
			console.log("Version: " + current_version_label + " (old), Release: " + latest_version_label);

		} else if (current_version > latest_version) {
			status_element.setAttribute("src", "img/version_devel.png");
			status_element.setAttribute("title", "Pre-release development version");
			console.log("Version: " + current_version_label + " (devel), Release: " + latest_version_label);
		
		} else {
			status_element.setAttribute("src", "img/version_latest.png");
			status_element.setAttribute("title", "Running latest release");
			console.log("Version: " + current_version_label + " (latest)");
		}
	}

	var displayPredefinedMessage = function(id) {
		var msg = document.getElementById(id);
		var close_button = document.getElementById(id + "_close");
		close_button.addEventListener('click', function() {
			msg.style.display = "none";
		});
		msg.style.display = "block";
	}

	this.displayCustomMessage = function(id, severity, title, content, child) {
		var msg_element = document.createElement('div');
		if (id != undefined) {
			var el = document.getElementById(id);
			if (el != undefined) el.parentNode.removeChild(el);
			msg_element.setAttribute('id', id);
		}
		msg_element.setAttribute('class', "feed_message");

		var msg_table = document.createElement('table');
		msg_table.setAttribute('cellpadding', "0");
		msg_table.setAttribute('cellspacing', "0");
		msg_table.setAttribute('border', "0");
		msg_table.setAttribute('style', "width:100%");

		var msg_header = document.createElement('tr');

		var msg_icon = document.createElement('td');
		msg_icon.setAttribute('style', "width:20px");
		if (severity == 1) {
			msg_icon.innerHTML = '<img src="img/warning.png">';
			msg_element.setAttribute('style', "background:#FFC");
		} else if (severity == 2) {
			msg_icon.innerHTML = '<img src="img/error.png">';
			msg_element.setAttribute('style', "background:#FDC");
		} else {
			msg_icon.innerHTML = '<img src="img/information.png">';
		}

		var msg_title = document.createElement('td');
		msg_title.setAttribute('style', "vertical-align: middle");
		msg_title.innerHTML = '<h2>' + title + '</h2>';

		var msg_close = document.createElement('td');
		msg_close.setAttribute('style', "width:20px");
		msg_close.innerHTML = '<img src="img/close.png">';
		msg_close.addEventListener('click', function() {
			msg_element.parentNode.removeChild(msg_element);
		});

		msg_header.appendChild(msg_icon);
		msg_header.appendChild(msg_title);
		msg_header.appendChild(msg_close);

		var msg_body = document.createElement('tr');

		var msg_content = document.createElement('td');
		msg_content.setAttribute('colspan', "3");
		msg_content.setAttribute('style', "padding:2px; padding-top:5px;");
		msg_content.innerHTML = content;
		if (child != undefined) msg_content.appendChild(child);

		msg_body.appendChild(msg_content);

		msg_table.appendChild(msg_header);
		msg_table.appendChild(msg_body);

		msg_element.appendChild(msg_table);

		document.getElementById('feed_area').appendChild(msg_element);
	}

	this.initialize = function() {
		document.getElementById("flexbe_version_label").innerText = chrome.runtime.getManifest().version;

		requestLatestVersion(
			displayVersionIndicator
		);
	}

	this.showAbout = function() {
		displayPredefinedMessage("msg_about");
	}

	this.showWelcome = function() {
		displayPredefinedMessage("msg_welcome");
	}

	this.hideAbout = function() { }

}) ();
