RC.Sync = new (function() {
	var that = this;

	this.STATUS_OK = 0;
	this.STATUS_WARN = 1;
	this.STATUS_ERROR = 2;
	var processes = [];

	var update_vis_cb = undefined;

	var updateSync = function() {
		var status = that.STATUS_OK;

		var sync = 100;
		processes.forEach(function(element) {
			sync -= element.sync * (1 - element.fulfilled);
			status = Math.max(status, element.status);
		});

		if (sync < 10) sync = 10;
		UI.RuntimeControl.setProgress(sync);
		UI.RuntimeControl.setProgressStatus(status);

		if (update_vis_cb != undefined) update_vis_cb(processes);
	}

	this.setVisualizationCallback = function(cb) {
		update_vis_cb = cb;
		if (update_vis_cb != undefined) update_vis_cb(processes);
	}

	this.register = function(key, sync) {
		var process = processes.findElement(function (element) {
			return element.key == key;
		});
		if (process == undefined) {
			processes.push({key: key, sync: sync, fulfilled: 0, status: that.STATUS_OK});
		} else {
			process.sync = sync;
			process.fulfilled = 0;
			process.status = that.STATUS_OK;
		}
		updateSync();
	}

	this.remove = function(key) {
		var process = processes.findElement(function (element) {
			return element.key == key;
		});
		if (process == undefined) {
			T.debugWarn("Process " + key + " not found for RC.Sync update, skipping remove operation.");
		} else {
			processes.remove(process);
		}
		updateSync();
	}

	this.hasProcess = function(key) {
		var process = processes.findElement(function (element) {
			return element.key == key;
		});
		 return process != undefined;
	}

	this.setProgress = function(key, fulfilled, relative) {
		var process = processes.findElement(function (element) {
			return element.key == key;
		});
		if (process == undefined) {
			T.debugWarn("Process " + key + " not found for RC.Sync update, can't update progress.");
			return;
		}

		process.fulfilled = (relative? process.fulfilled : 0) + fulfilled;

		process.fulfilled = Math.min(Math.max(process.fulfilled, 0), 1);

		updateSync();
	}

	this.setStatus = function(key, new_status) {
		var process = processes.findElement(function (element) {
			return element.key == key;
		});
		if (process == undefined) {
			T.debugWarn("Process " + key + " not found for RC.Sync update, can't set status.");
			return;
		}

		process.status = new_status;
		updateSync();
	}

	this.shutdown = function() {
		processes = [];
		UI.RuntimeControl.resetProgress();
		UI.RuntimeControl.setProgressStatus(that.STATUS_OK);
		if (update_vis_cb != undefined) update_vis_cb(processes);
	}

	this.print = function() {
		processes.forEach(function(element) {
			console.log(element);
		});
	}

}) ();