ActivityTracer = new (function() {
	var that = this;

	var activity_list = [undefined];
	var current_index = 0;
	var last_save_index = 0;
	var onboard_index = 0;

	var logging_enabled = true;

	var update_callback = undefined;

	this.ACT_INTERNAL_CONFIG_ADD = "act_internal_config_add";
	this.ACT_INTERNAL_CONFIG_CHANGE = "act_internal_config_change";
	this.ACT_INTERNAL_CONFIG_REMOVE = "act_internal_config_remove";
	this.ACT_BEHAVIOR_INTERFACE_CHANGE = "act_behavior_interface_change";

	this.ACT_STATE_ADD = "act_state_add";
	this.ACT_STATE_CHANGE = "act_state_change";
	this.ACT_STATE_REMOVE = "act_state_remove";

	this.ACT_TRANSITION = "act_transition";

	this.ACT_COMPLEX_OPERATION = "act_complex_operation";

	this.addActivity = function(category, description, undo, redo) {
		if (!logging_enabled) return;

		console.log("Activity: " + description);

		if (current_index != activity_list.length - 1) {
			activity_list = activity_list.slice(0, current_index + 1);
		}
		current_index = activity_list.length;
		activity_list.push({time: Date.now(), category: category, description: description, undo: undo, redo: redo});

		RC.Controller.signalChanged();
		if (update_callback != undefined) update_callback();
	}

	this.setUpdateCallback = function(cb) {
		update_callback = cb;
	}

	this.addSave = function() {
		last_save_index = current_index;

		RC.Controller.signalBehavior();
	}

	this.addExecution = function() {
		onboard_index = current_index;
		if (onboard_index != activity_list.length - 1) {
			activity_list = activity_list.slice(0, onboard_index + 1);
		}
	}

	this.undo = function() {
		if (current_index == 0) return;
		if (RC.Controller.isRunning() && current_index == onboard_index) return;
		logging_enabled = false;

		activity_list[current_index].undo();

		current_index -= 1;
		logging_enabled = true;

		if (current_index == onboard_index) {
			RC.Controller.signalReset();
		} else if (current_index == last_save_index) {
			RC.Controller.signalBehavior();
		} else {
			RC.Controller.signalChanged();
		}
		if (update_callback != undefined) update_callback();
	}

	this.redo = function() {
		if (current_index == activity_list.length - 1) return;
		logging_enabled = false;
		current_index += 1;

		activity_list[current_index].redo();

		logging_enabled = true;

		if (current_index == onboard_index) {
			RC.Controller.signalReset();
		} else if (current_index == last_save_index) {
			RC.Controller.signalBehavior();
		} else {
			RC.Controller.signalChanged();
		}
		if (update_callback != undefined) update_callback();
	}

	this.hasUnsavedChanges = function() {
		return last_save_index != current_index;
	}

	this.getActivityList = function() {
		return activity_list;
	}

	this.getCurrentIndex = function() {
		return current_index;
	}

	this.getLastSaveIndex = function () {
		return last_save_index;
	}

	this.getExecutionIndex = function () {
		return onboard_index;
	}

	this.resetActivities = function() {
		activity_list = [undefined];
		current_index = 0;
		last_save_index = 0;
		onboard_index = 0;
		logging_enabled = true;

		RC.Controller.signalBehavior();
	}

	this.resetToSave = function() {
		that.goToIndex(last_save_index);
	}

	this.resetToExecution = function() {
		that.goToIndex(onboard_index);
	}

	this.goToIndex = function(target_index) {
		if (RC.Controller.isRunning() && target_index < onboard_index) {
			target_index = onboard_index;
		}
		if (current_index == target_index) return;
		var operation = (current_index > target_index)? that.undo : that.redo;

		while(current_index != target_index) {
			operation();
		}
	}

	this.doNotTrace = function(operation) {
		logging_enabled = false;
		operation();
		logging_enabled = true;
	}

	this.printCurrentHistory = function() {
		for (var i = activity_list.length - 1; i >= 0; i--) {
			if (activity_list[i] == undefined) break;
			var txt = "";
			if (i == current_index) txt += "> ";
			if (i == last_save_index) txt += "(saved) ";
			txt += activity_list[i].description;
			console.log(txt);
		}
	}

}) ();