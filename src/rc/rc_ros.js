RC.ROS = new (function() {
	var that = this;

	var connected = false;
	var trying = false;
	var namespace = "";

	this.getROS = function() {
		return undefined;
	}

	var setupConnection = function(node_namespace) {
		if (node_namespace == undefined) return;
		namespace = node_namespace;
		T.logInfo("ROS connection running!");
		connected = true;

		// at first, not connected to the behavior engine
		// will change as soon as we get any message from the onboard be
		UI.Menu.displayRuntimeStatus("disconnected");
		RC.Sync.register("ROS", 90);
		RC.Sync.setStatus("ROS", RC.Sync.STATUS_ERROR);

		UI.Settings.setRosProperties(namespace);
		RC.PubSub.initialize(namespace);
	}

	this.trySetupConnection = function() {
		T.logInfo("Setting up ROS connection ...");
		trying = true;
		UI.Settings.setRosProperties('');
		ROS.init(setupConnection);
		T.logInfo("Done ROS connection setup!");
	}

	this.closeConnection = function() {
		T.logInfo("Closing ROS connection...");
		RC.PubSub.shutdown();
		connected = false;
		trying = false;
		UI.Settings.setRosProperties('');

		RC.Controller.signalDisconnected();
		RC.Sync.remove("ROS");
		RC.Sync.shutdown();
		ROS.shutdown();
		UI.Menu.displayRuntimeStatus("offline");
		T.logInfo("ROS connection closed!");
	}

	this.isConnected = function() {
		return connected;
	}

	this.isTrying = function() {
		return trying;
	}

}) ();
