UI.Panels.Terminal = new (function() {
	var that = this;
	var debug_mode = false;
	var is_active = false;

	var logTerminal = function (msg, color) {
		document.getElementById("terminal").innerHTML += "<font style='color: " + color + ";'>" + msg + "</font><br />";
		document.getElementById("terminal").scrollTop = document.getElementById("terminal").scrollHeight;
	}


	this.logInfo = function(msg) {
		logTerminal(msg, "white");
		console.log(msg);
	}

	this.logWarn = function(msg) {
		logTerminal(msg, "yellow");
		console.log("[WARN] " + msg);
	}

	this.logError = function(msg) {
		logTerminal(msg, "red");
		console.log("[ERROR] " + msg);
		this.show();
	}


	this.debugInfo = function(msg) {
		if (debug_mode) {
			logTerminal("> " + msg, "white");
		}
		console.log(msg);
	}

	this.debugWarn = function(msg) {
		if (debug_mode) {
			logTerminal("> " + msg, "yellow");
			//this.show();
		}
		console.log("[WARN] " + msg);
	}

	this.debugError = function(msg) {
		if (debug_mode) {
			logTerminal("> " + msg, "red");
			this.show();
		}
		console.log("[ERROR] " + msg);
	}
	
	this.clearLog = function() {
		document.getElementById("terminal").innerHTML += "<br /><br />";
	}

	this.show = function() {
		UI.Panels.setActivePanel(UI.Panels.TERMINAL_PANEL);
		is_active = true;
	}

	this.hide = function() {
		UI.Panels.hidePanelIfActive(UI.Panels.TERMINAL_PANEL);
		is_active = false;
	}

	this.toggle = function() {
		if (is_active) that.hide();
		else that.show();
	}

}) ();

T = UI.Panels.Terminal;