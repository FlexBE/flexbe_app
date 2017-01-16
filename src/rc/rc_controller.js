RC.Controller = new (function() {
	var that = this;

	var current_state_path = "";
	var locked_state_path = "";
	var behaviorAvailable = false;

	var sync_timer = undefined;

	var STATE_NOTHING = {
		onEnter: function() {
			UI.Menu.displayRuntimeStatus('offline');
			UI.RuntimeControl.displayEngineOffline();
			if (RC.Sync.hasProcess("ROS")) {
				RC.Sync.setStatus("ROS", RC.Sync.STATUS_ERROR);
				RC.Sync.setProgress("ROS", 0, false);
				UI.Menu.displayRuntimeStatus('disconnected');
			}
			if (RC.Sync.hasProcess("Delay")) RC.Sync.remove("Delay", 50);
		},
		onExit: function() { },
		isActive: false,
		label: "STATE_NOTHING"
	};
	var STATE_OFFLINE = {
		onEnter: function() {
			UI.Menu.displayRuntimeStatus('offline');
			UI.RuntimeControl.displayEngineOffline();
			UI.Dashboard.unsetReadonly();
			if (RC.Sync.hasProcess("ROS")) {
				RC.Sync.setStatus("ROS", RC.Sync.STATUS_ERROR);
				RC.Sync.setProgress("ROS", 0, false);
				UI.Menu.displayRuntimeStatus('disconnected');
			}
			if (RC.Sync.hasProcess("Delay")) RC.Sync.remove("Delay", 50);
		},
		onExit: function() { },
		isActive: false,
		label: "STATE_OFFLINE"
	};
	var STATE_NO_BEHAVIOR = {
		onEnter: function() {
			UI.Menu.displayRuntimeStatus('online');
			UI.RuntimeControl.displayNoBehavior();
			RC.Sync.setStatus("ROS", RC.Sync.STATUS_OK);
			RC.Sync.setProgress("ROS", 1, false);

			RC.Sync.register("Delay", 50);
		},
		onExit: function() { },
		isActive: false,
		label: "STATE_NO_BEHAVIOR"
	};
	var STATE_CONFIGURATION = {
		onEnter: function() {
			if (vis_update_timer != undefined) clearTimeout(vis_update_timer);
			vis_update_timer = undefined;
			UI.Menu.displayRuntimeStatus('online');
			UI.RuntimeControl.displayBehaviorConfiguration();
			UI.RuntimeControl.resetPauseButton();
			RC.Sync.setStatus("ROS", RC.Sync.STATUS_OK);
			RC.Sync.setProgress("ROS", 1, false);
			UI.Dashboard.unsetReadonly();

			RC.Sync.register("Delay", 50);

			if (RC.Sync.hasProcess("State")) RC.Sync.remove("State");
			if (RC.Sync.hasProcess("Transition")) RC.Sync.remove("Transition");
			if (RC.Sync.hasProcess("Unlock")) RC.Sync.remove("Unlock");
			if (RC.Sync.hasProcess("Lock")) RC.Sync.remove("Lock");
			if (RC.Sync.hasProcess("Preempt")) RC.Sync.remove("Preempt");
			if (RC.Sync.hasProcess("Autonomy")) RC.Sync.remove("Autonomy");
			if (RC.Sync.hasProcess("BehaviorStart")) RC.Sync.remove("BehaviorStart");
			if (RC.Sync.hasProcess("Switch")) RC.Sync.remove("Switch");
			if (RC.Sync.hasProcess("Changes")) RC.Sync.remove("Changes");
			if (RC.Sync.hasProcess("Sync")) RC.Sync.remove("Sync");
			if (RC.Sync.hasProcess("Pause")) RC.Sync.remove("Pause");
			if (RC.Sync.hasProcess("Repeat")) RC.Sync.remove("Repeat");

			document.getElementById("button_ros_connect").removeAttribute("disabled", "disabled");

			current_state_path = "";
			locked_state_path = "";
		},
		onExit: function() {
			UI.RuntimeControl.displayOutcomeRequest("", undefined);
		},
		isActive: false,
		label: "STATE_CONFIGURATION"
	};
	var STATE_EXTERNAL_NO_BEHAVIOR = {
		onEnter: function() {
			document.getElementById("selection_rc_autonomy").setAttribute("disabled", "disabled");
			UI.RuntimeControl.displayExternalBehavior();
			T.clearLog();
			T.logInfo('Running behavior detected!');
			T.logInfo('You may go to Runtime Control in order to attach and monitor execution.');
			T.show();
		},
		onExit: function() {
			document.getElementById("selection_rc_autonomy").removeAttribute("disabled", "disabled");
		},
		isActive: false,
		label: "STATE_STARTING"
	};
	var STATE_EXTERNAL = {
		onEnter: function() {
			document.getElementById("selection_rc_autonomy").setAttribute("disabled", "disabled");
			UI.RuntimeControl.displayExternalBehavior();
			T.clearLog();
			T.logInfo('Running behavior detected!');
			T.logInfo('You may go to Runtime Control in order to attach and monitor execution.');
			T.show();
		},
		onExit: function() {
			if (RC.Sync.hasProcess("Attach")) RC.Sync.remove("Attach");
			document.getElementById("selection_rc_autonomy").removeAttribute("disabled", "disabled");
		},
		isActive: false,
		label: "STATE_STARTING"
	};
	var STATE_STARTING = {
		onEnter: function() {
			document.getElementById("selection_rc_autonomy").setAttribute("disabled", "disabled");
			UI.RuntimeControl.displayWaitingForBehavior();
			ActivityTracer.addExecution();
			UI.Dashboard.setReadonly();
			UI.Panels.SelectBehavior.hide();
			UI.Panels.AddState.hide();
			UI.Panels.StateProperties.hide();
			document.getElementById("button_ros_connect").setAttribute("disabled", "disabled");

			RC.Sync.register("State", 30);
		},
		onExit: function() {
			document.getElementById("selection_rc_autonomy").removeAttribute("disabled", "disabled");
		},
		isActive: false,
		label: "STATE_STARTING"
	};
	var STATE_ACTIVE = {
		onEnter: function() {
			// repeat for external
			ActivityTracer.addExecution();
			UI.Dashboard.setReadonly();
			RC.Sync.register("State", 30);

			UI.Menu.displayRuntimeStatus('running');
			UI.RuntimeControl.displayState(current_state_path);
			UI.RuntimeControl.displayLockBehavior();
			UI.RuntimeControl.refreshView();
			UI.Panels.SelectBehavior.hide();
			UI.Panels.AddState.hide();
			UI.Panels.StateProperties.hide();

			RC.Sync.setProgress("State", 1, false);

			/* Now based on heartbeat delay
			if (sync_timer != undefined) clearTimeout(sync_timer);
			var reduceSync = function() {
				RC.Sync.setProgress("State", -0.01, true);
				sync_timer = setTimeout(reduceSync, 1000);
			};
			reduceSync();*/

			locked_state_path = "";
		},
		onExit: function() {
			if (sync_timer != undefined) clearTimeout(sync_timer);
			if (vis_update_timer != undefined) clearTimeout(vis_update_timer);
			vis_update_timer = undefined;
			UI.Statemachine.refreshView();
			RC.Sync.setProgress("State", 1, false);
			if (RC.Sync.hasProcess("Transition")) RC.Sync.remove("Transition");
		},
		isActive: false,
		label: "STATE_ACTIVE"
	};
	var STATE_LOCKED = {
		onEnter: function() {
			UI.Menu.displayRuntimeStatus('locked');
			UI.RuntimeControl.displayUnlockBehavior(false);
			UI.RuntimeControl.refreshView();
		},
		onExit: function() {
		},
		isActive: false,
		label: "STATE_LOCKED"
	};
	var STATE_CHANGED = {
		onEnter: function() {
			UI.RuntimeControl.displayBehaviorChanged();
			RC.Sync.setStatus("Changes", RC.Sync.STATUS_WARN);
		},
		onExit: function() {
			RC.Sync.setStatus("Changes", RC.Sync.STATUS_OK);
		},
		isActive: false,
		label: "STATE_CHANGED"
	};
	var STATE_NEW_VERSION = {
		onEnter: function() {
			UI.RuntimeControl.displayUnlockBehavior(true);
		},
		onExit: function() {
		},
		isActive: false,
		label: "STATE_NEW_VERSION"
	};

	var current_state;
	var hasTransition = function(from, to) {
		if (from.isActive) setState(to);
	} 

	var setState = function(new_state) {
		//if (new_state == current_state) return;

		console.log("Transition to state: " + new_state.label);

		var exit_state = current_state;
		current_state.isActive = false;
		current_state = new_state;
		current_state.isActive = true;
		exit_state.onExit();
		current_state.onEnter();
	}

	var vis_update_timer;
	var vis_update_required = false;
	var vis_update = function() {
		if (vis_update_required && that.isActive()) {
			vis_update_required = false;
			if (that.isRunning()) {
				UI.RuntimeControl.displayState(current_state_path);
				if (RC.Sync.hasProcess("State"))
					RC.Sync.setProgress("State", 1, false);
			}
			if(UI.Menu.isPageStatemachine())
				UI.Statemachine.refreshView();
		}

		vis_update_timer = setTimeout(vis_update, 1000 / 25);
	}

	this.initialize = function() {
		current_state = STATE_NOTHING;
		current_state.isActive = true;
		current_state.onEnter();
	}


	this.signalChanged = function() {
		hasTransition(STATE_CONFIGURATION,	STATE_NO_BEHAVIOR);
		hasTransition(STATE_OFFLINE, 		STATE_NOTHING);
		hasTransition(STATE_LOCKED, 		STATE_CHANGED);
		hasTransition(STATE_NEW_VERSION, 	STATE_CHANGED);
		hasTransition(STATE_EXTERNAL, 		STATE_EXTERNAL_NO_BEHAVIOR);
	}
	this.signalBehavior = function() {
		hasTransition(STATE_NOTHING,				STATE_OFFLINE);
		hasTransition(STATE_NO_BEHAVIOR,			STATE_CONFIGURATION);
		hasTransition(STATE_CHANGED,				STATE_NEW_VERSION);
		hasTransition(STATE_CONFIGURATION,			STATE_CONFIGURATION);
		hasTransition(STATE_EXTERNAL_NO_BEHAVIOR,	STATE_EXTERNAL);
	}
	this.signalDisconnected = function() {
		hasTransition(STATE_NO_BEHAVIOR,	STATE_NOTHING);
		hasTransition(STATE_CONFIGURATION,	STATE_OFFLINE);
		hasTransition(STATE_STARTING,		STATE_OFFLINE);
		hasTransition(STATE_ACTIVE,			STATE_OFFLINE);
		hasTransition(STATE_LOCKED,			STATE_OFFLINE);
		hasTransition(STATE_CHANGED,		STATE_NOTHING);
	}
	this.signalConnected = function() {
		hasTransition(STATE_NOTHING,	STATE_NO_BEHAVIOR);
		hasTransition(STATE_OFFLINE,	STATE_CONFIGURATION);
	}
	this.signalExternal = function() {
		hasTransition(STATE_NO_BEHAVIOR,	STATE_EXTERNAL_NO_BEHAVIOR);
		hasTransition(STATE_CONFIGURATION,	STATE_EXTERNAL);
	}
	this.signalStarted = function() {
		hasTransition(STATE_CONFIGURATION,	STATE_STARTING);
	}
	this.signalRunning = function() {
		hasTransition(STATE_STARTING,		STATE_ACTIVE);
		hasTransition(STATE_EXTERNAL,		STATE_ACTIVE);
	}
	this.signalFinished = function() {
		hasTransition(STATE_ACTIVE,					STATE_CONFIGURATION);
		hasTransition(STATE_LOCKED,					STATE_CONFIGURATION);
		hasTransition(STATE_CHANGED,				STATE_CONFIGURATION);
		hasTransition(STATE_NEW_VERSION,			STATE_CONFIGURATION);
		hasTransition(STATE_STARTING,				STATE_CONFIGURATION);
		hasTransition(STATE_EXTERNAL_NO_BEHAVIOR,	STATE_CONFIGURATION);
		hasTransition(STATE_EXTERNAL,				STATE_CONFIGURATION);
	}
	this.signalLocked = function() {
		hasTransition(STATE_ACTIVE,		STATE_LOCKED);
	}
	this.signalUnlocked = function() {
		hasTransition(STATE_LOCKED,			STATE_ACTIVE);
		hasTransition(STATE_NEW_VERSION,	STATE_ACTIVE);
	}
	this.signalReset = function() {
		hasTransition(STATE_CHANGED,		STATE_LOCKED);
		hasTransition(STATE_NEW_VERSION,	STATE_LOCKED);
	}


	this.isConnected = function() {
		return !STATE_OFFLINE.isActive && !STATE_NOTHING.isActive;
	}
	this.isReadonly = function() {
		return STATE_ACTIVE.isActive || STATE_STARTING.isActive;
	}
	this.isRunning = function() {
		return STATE_ACTIVE.isActive || STATE_LOCKED.isActive || STATE_CHANGED.isActive || STATE_NEW_VERSION.isActive;
	}
	this.isActive = function() {
		return STATE_ACTIVE.isActive;
	}
	this.isExternal = function() {
		return STATE_EXTERNAL.isActive || STATE_EXTERNAL_NO_BEHAVIOR.isActive;
	}
	this.isLocked = function() {
		return STATE_LOCKED.isActive || STATE_CHANGED.isActive || STATE_NEW_VERSION.isActive;
	}
	this.needSwitch = function() {
		return STATE_NEW_VERSION.isActive;
	}
	this.haveBehavior = function() {
		return !STATE_NOTHING.isActive && !STATE_NO_BEHAVIOR.isActive;
	}


	this.onboardTimeout = 10; // seconds (this is only the default value, actual value set by settings)

	this.isCurrentState = function(state_to_check, include_path) {
		if (current_state_path == undefined) return false;

		return current_state_path == state_to_check.getStatePath()
			|| include_path && current_state_path.startsWith(state_to_check.getStatePath() + "/");
	}

	this.getCurrentState = function() {
		return Behavior.getStatemachine().getStateByPath(current_state_path);
	}

	this.setCurrentStatePath = function(state_path) {
		current_state_path = state_path;

		vis_update_required = true;
		if (vis_update_timer == undefined) vis_update();
	}

	this.setLockedStatePath = function(state_path) {
		locked_state_path = state_path;
	}
	this.isStateLocked = function(state_path) {
		return state_path == locked_state_path;
	}
	this.isOnLockedPath = function(state_path) {
		return locked_state_path == state_path || locked_state_path.startsWith(state_path + "/");
	}

}) ();