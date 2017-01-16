UI.Panels = new (function() {
	var that = this;

	var activePanel = this.NO_PANEL;

	var displayAddState = function() {
		document.getElementById("panel_add_state").style.right = "0px";
		document.getElementById("panel_add_state").style.padding = "5px 5px";
	}

	var hideAddState = function() {
		document.getElementById("panel_add_state").style.right = "-360px";
		document.getElementById("panel_add_state").style.padding = "5px 0px";
	}

	var displaySelectBehavior = function() {
		document.getElementById("panel_select_behavior").style.right = "0px";
		document.getElementById("panel_select_behavior").style.padding = "5px 5px";
	}

	var hideSelectBehavior = function() {
		document.getElementById("panel_select_behavior").style.right = "-360px";
		document.getElementById("panel_select_behavior").style.padding = "5px 0px";
	}

	var displayProperties = function() {
		document.getElementById("panel_properties").style.right = "0px";
		document.getElementById("panel_properties").style.padding = "5px 5px";
	}

	var hideProperties = function() {
		document.getElementById("panel_properties").style.right = "-360px";
		document.getElementById("panel_properties").style.padding = "5px 0px";
	}

	var displayTerminal = function() {
		document.getElementById("terminal").style.height = "calc(30% - 30px)";
		document.getElementById("terminal").style.padding = "5px";
		document.getElementById("terminal").style.width = "calc(100% - 10px)";
	}

	var hideTerminal = function() {
		document.getElementById("terminal").style.height = "0";
		document.getElementById("terminal").style.padding = "0";
		document.getElementById("terminal").style.width = "100%";
	}


	this.ADD_STATE_PANEL = "addState";
	this.SELECT_BEHAVIOR_PANEL = "selectBehavior";
	this.STATE_PROPERTIES_PANEL = "stateProperties";
	this.TERMINAL_PANEL = "terminal";
	this.NO_PANEL = "no";

	this.setActivePanel = function(panel) {
		if (panel == activePanel) return;

		that.hidePanelIfActive(activePanel);

		if (panel == that.ADD_STATE_PANEL) displayAddState();
		else if (panel == that.SELECT_BEHAVIOR_PANEL) displaySelectBehavior();
		else if (panel == that.STATE_PROPERTIES_PANEL) displayProperties();
		else if (panel == that.TERMINAL_PANEL) displayTerminal();

		activePanel = panel;
	}

	this.hidePanelIfActive = function(panel) {
		if (panel != activePanel) return;

		if (activePanel == that.ADD_STATE_PANEL) hideAddState();
		else if (activePanel == that.SELECT_BEHAVIOR_PANEL) hideSelectBehavior();
		else if (activePanel == that.STATE_PROPERTIES_PANEL) hideProperties();
		else if (activePanel == that.TERMINAL_PANEL) hideTerminal();

		activePanel = that.NO_PANEL;
	}

}) ();