RC.PubSub = new (function() {
	var that = this;

	var current_state_listener;
	var outcome_request_listener;
	var behavior_feedback_listener;
	var behavior_status_listener;
	var command_feedback_listener;
	var onboard_heartbeat_listener;
	var ros_command_listener;

	var behavior_start_publisher;
	var transition_command_publisher;
	var autonomy_level_publisher;
	var preempt_behavior_publisher;
	var lock_behavior_publisher;
	var unlock_behavior_publisher;
	var sync_mirror_publisher;
	var attach_behavior_publisher;
	var repeat_behavior_publisher;
	var pause_behavior_publisher;
	var version_publisher;
	var ros_notification_publisher;

	var synthesis_action_client;

	var last_heartbeat_time = undefined;
	var expected_sync_path = undefined;

	var current_state_callback = function (msg) {
		if (RC.Sync.hasProcess("Transition")) RC.Sync.remove("Transition");
		//if (expected_sync_path != undefined && RC.Sync.hasProcess("Sync") && expected_sync_path == msg.data) RC.Sync.remove("Sync");

		RC.Controller.setCurrentStatePath(msg.data);

		if (msg.data == "") {
			RC.Controller.signalFinished();
		} else if (!RC.Controller.isExternal()) {
			RC.Controller.signalRunning();
		}
	}
	var outcome_request_callback = function(msg) {
		var target_state = Behavior.getStatemachine().getStateByPath(msg.target);
		UI.RuntimeControl.displayOutcomeRequest(target_state.getOutcomes()[msg.outcome], target_state);
	}

	var behavior_feedback_callback = function (msg){
		UI.RuntimeControl.displayBehaviorFeedback(msg.status_code, msg.text);
	}

	var behavior_status_callback = function (msg){
		console.log(msg);
		// constants of BEStatus.msg (not directly accessible via roslib.js)
		var STARTED = 0;
		var FINISHED = 1;
		var FAILED = 2;
		var WARNING = 10;
		var ERROR = 11;
		var READY = 20;

		if (msg.code == STARTED && !RC.Controller.haveBehavior() && UI.Settings.isStopBehaviors()) {
			T.logError("Onboard behavior is still running! Stopping it...");
			RC.Sync.register("EmergencyStop", 30);
			RC.Sync.setStatus("EmergencyStop", RC.Sync.STATUS_ERROR);
			RC.PubSub.sendPreemptBehavior();
			return;
		}
		if (RC.Sync.hasProcess("EmergencyStop") && (msg.code == FINISHED || msg.code == FAILED)) {
			RC.Sync.remove("EmergencyStop");
			T.logInfo("Onboard behavior stopped!");
			T.logInfo("Please press 'Stop Execution' next time before closing this window when running a behavior.");
		}

		if (RC.Controller.isLocked() && msg.code == STARTED && msg.args.length > 0 
			&& RC.Controller.isCurrentState(Behavior.getStatemachine().getStateByPath(msg.args[0]), false)) {

			RC.Sync.remove("Switch");
			that.sendBehaviorUnlock(msg.args[0]);
		} else if (msg.code == FINISHED || msg.code == FAILED) {
			RC.Controller.signalFinished();
			UI.RuntimeControl.displayBehaviorFeedback(4, "No behavior active.");
		} else if (msg.code == STARTED) {
			if (RC.Sync.hasProcess("BehaviorStart")) {
				RC.Sync.remove("BehaviorStart");
				RC.Controller.signalRunning();
			} else {
				RC.Controller.signalConnected();
				RC.Controller.signalExternal();
			}
		} else if (msg.code == ERROR) {
			RC.Sync.setProgress("BehaviorStart", 1, false);
			RC.Sync.setStatus("BehaviorStart", RC.Sync.STATUS_ERROR);
			if(!RC.Controller.isLocked()) {
				RC.Controller.signalFinished();
				UI.RuntimeControl.displayBehaviorFeedback(4, "No behavior active.");
			}
		} else if (msg.code == READY) {
			RC.Controller.signalFinished();
			UI.RuntimeControl.displayBehaviorFeedback(4, "Onboard engine just started.");
		}
	}

	var heartbeat_timer;
	var onboard_heartbeat_callback = function (msg){
		if (heartbeat_timer != undefined) clearTimeout(heartbeat_timer);
		RC.Controller.signalConnected();

		var now = Date.now();
		if (last_heartbeat_time != undefined) {
			delay = (now - last_heartbeat_time) / 1000;
			last_heartbeat_time = now;
			relative_delay = (delay - 1) / (RC.Controller.onboardTimeout - 1);
			RC.Sync.setProgress("Delay", Math.min(Math.max(1 - relative_delay, 0), 1), false);
				 if (relative_delay > 0.95) RC.Sync.setStatus("Delay", RC.Sync.STATUS_ERROR);
			else if (relative_delay > 0.60) RC.Sync.setStatus("Delay", RC.Sync.STATUS_WARN);
			else RC.Sync.setStatus("Delay", RC.Sync.STATUS_OK);
			//console.log("Heartbeat delay: " + delay + " (= " + Math.round(relative_delay * 100) + "%)");
		} else {
			last_heartbeat_time = now;
			RC.Sync.setProgress("Delay", 1, false);
		}

		heartbeat_timer = setTimeout(function() {
			console.log("Onboard connection timed out.");
			RC.Controller.signalDisconnected();
		}, RC.Controller.onboardTimeout * 1000);
	}

	var ros_command_callback = function (msg) {
		if (!UI.Settings.isCommandsEnabled()) {
			that.sendRosNotification('');
			return;
		}
		if (UI.Settings.getCommandsKey() != '' && msg.key != UI.Settings.getCommandsKey()) {
			T.clearLog();
			T.logError('Captured unauthorized command execution attempt!');
			T.logInfo('You should disable ROS commands in the configuration view and check "rostopic info /flexbe/uicommand" for suspicious publishers.');
			that.sendRosNotification('');
			return;
		}

		T.clearLog();
		T.logInfo('Executing received command: ' + msg.command);
		UI.Tools.startRosCommand(msg.command);
		T.show();
	}

	var command_feedback_callback = function (msg) {
		console.log(msg);
		if (msg.command == "transition") {
			if (msg.args[0] == msg.args[1]) {
				RC.Sync.setProgress("Transition", 0.8, false);
			} else {
				RC.Sync.setStatus("Transition", RC.Sync.STATUS_WARN);
			}
		}
		if (msg.command == "autonomy") {
			RC.Sync.remove("Autonomy");
		}
		if (msg.command == "attach") {
			if (RC.Sync.hasProcess("Attach")) {
				if (msg.args[0] == Behavior.getBehaviorName()) {
					RC.Controller.signalRunning();
					RC.Sync.remove("Attach");
				} else {
					UI.RuntimeControl.displayBehaviorFeedback(3, "Failed to attach! Please load behavior: " + msg.args[0]);
					RC.Sync.setStatus("Attach", RC.Sync.STATUS_ERROR);
				}
			}
		}
		if (msg.command == "repeat") {
			if (RC.Sync.hasProcess("Repeat")) {
				RC.Sync.remove("Repeat");
			}
		}
		if (msg.command == "pause") {
			if (RC.Sync.hasProcess("Pause")) {
				UI.RuntimeControl.switchPauseButton();
				RC.Sync.setProgress("Pause", 1, false);
			}
		}
		if (msg.command == "resume") {
			if (RC.Sync.hasProcess("Pause")) {
				UI.RuntimeControl.switchPauseButton();
				RC.Sync.remove("Pause");
			}
		}
		if (msg.command == "preempt") {
			if (RC.Sync.hasProcess("Preempt")) {
				RC.Sync.remove("Preempt");
				RC.Controller.signalFinished();
			}
		}
		if (msg.command == "lock") {
			if (msg.args[0] == msg.args[1]) {
				RC.Sync.remove("Lock");
				RC.Sync.register("Changes", 0);
				RC.Sync.setProgress("Changes", 1, false);
				RC.Controller.signalLocked();
			} else {
				RC.Sync.setProgress("Lock", 1, false);
				RC.Sync.setStatus("Lock", RC.Sync.STATUS_WARN);
				RC.Sync.remove("Lock");
			}
		}
		if (msg.command == "unlock") {
			if (msg.args[0] == msg.args[1]) {
				RC.Sync.remove("Unlock");
				RC.Sync.remove("Changes");
				RC.Controller.signalUnlocked();
			} else {
				RC.Sync.setStatus("Unlock", RC.Sync.STATUS_WARN);
			}
		}
		if (msg.command == "sync") {
			expected_sync_path = msg.args[0];
			//RC.Sync.setProgress("Sync", 0.6, false);
			RC.Sync.remove("Sync");
		}
		if (msg.command == "switch") {
			if (msg.args[0] == "failed") 			RC.Sync.setStatus("Switch", RC.Sync.STATUS_ERROR);
			if (msg.args[0] == "not_switchable")	RC.Sync.setStatus("Switch", RC.Sync.STATUS_WARN);
			if (msg.args[0] == "received")			RC.Sync.setProgress("Switch", 0.2);
			if (msg.args[0] == "start")				RC.Sync.setProgress("Switch", 0.4);
			if (msg.args[0] == "prepared")			RC.Sync.setProgress("Switch", 0.6);
		}
		UI.Tools.notifyRosCommand(msg.command);
	}

	var synthesis_action_timeout_callback = function(timeout_cb) {
		T.logError('Synthesis timed out! Check if the synthesis server is listening on topic: ' +  UI.Settings.getSynthesisTopic());

		if(timeout_cb != undefined) timeout_cb();
	}

	var synthesis_action_feedback_callback = function(feedback, root, feedback_cb) {
		console.log('Synthesis status: ' + feedback.status + ' (' (feedback.progress * 100) + '%)');

		if(feedback_cb != undefined) feedback_cb(feedback);
	}

	var synthesis_action_result_callback = function(result, root, result_cb) {
		if (result == undefined) {
			T.logError("Synthesis cancelled.");
			return;
		}
		if (result.error_code.value != 1) {
			T.logError("Synthesis failed: " + result.error_code.value);
			return;
		}

		var root_split = root.split("/");
		var root_name = root_split[root_split.length - 1];
		var root_container_path = root.replace("/" + root_name, "");
		var root_container = (root_container_path == "")? Behavior.getStatemachine() : 
								Behavior.getStatemachine().getStateByPath(root_container_path);
		var root_varname = "";
		var defs = IO.ModelGenerator.parseInstantiationMsg(result.states);
		if (defs == undefined) {
			T.logError('Aborted synthesis because of previous errors.');
			return;
		}

		var state_machine = IO.ModelGenerator.buildStateMachine(root_name, root_varname, defs.sm_defs, defs.sm_states, true);

		var sm_instance = root_container.getStateByName(state_machine.getStateName());
		if (sm_instance != undefined) {
			var transitions = root_container.getTransitions().filter(function(t) {
				return t.getFrom().getStateName() == sm_instance.getStateName() && state_machine.getOutcomes().contains(t.getOutcome())
					|| t.getTo() != undefined && t.getTo().getStateName() == sm_instance.getStateName();
			});
			var is_initial = root_container.getInitialState() != undefined && sm_instance.getStateName() == root_container.getInitialState().getStateName();
			root_container.removeState(sm_instance);
			root_container.addState(state_machine);
			if (is_initial) root_container.setInitialState(sm_instance);
			transitions.forEach(function (t) {
				if (t.getTo() != undefined && t.getTo().getStateName() == state_machine.getStateName()) t.setTo(state_machine);
				if (t.getFrom().getStateName() == state_machine.getStateName()) t.setFrom(state_machine);
			});
			transitions.forEach(root_container.addTransition);
		} else {
			root_container.addState(state_machine);
		}

		if(UI.Menu.isPageStatemachine()) UI.Statemachine.refreshView();

		ActivityTracer.addActivity(ActivityTracer.ACT_STATE_ADD,
			"Added synthesized statemachine " + root_name,
			function() {
				state_machine.getContainer().removeState(state_machine);
				if (UI.Panels.StateProperties.isCurrentState(state_machine)) {
					UI.Panels.StateProperties.hide();
				}
				UI.Statemachine.refreshView();
			},
			function() {
				var container = (root_container_path == "")? Behavior.getStatemachine() : Behavior.getStatemachine().getStateByPath(root_container_path);
				container.addState(state_machine);
				UI.Statemachine.refreshView();
			}
		);

		if(result_cb != undefined) result_cb(result);
	}


	this.initialize = function(ns) {
		if (!ns.startsWith('/')) ns = '/' + ns;
		if (!ns.endsWith('/')) ns += '/';


		// Subscriber

		current_state_listener = new ROS.Subscriber(
			ns + 'flexbe/behavior_update',
			'std_msgs/String',
			current_state_callback);

		outcome_request_listener = new ROS.Subscriber(
			ns + 'flexbe/outcome_request',
			'flexbe_msgs/OutcomeRequest',
			outcome_request_callback);

		behavior_feedback_listener = new ROS.Subscriber(
			ns + 'flexbe/log',
			'flexbe_msgs/BehaviorLog',
			behavior_feedback_callback);

		behavior_status_listener = new ROS.Subscriber(
			ns + 'flexbe/status',
			'flexbe_msgs/BEStatus',
			behavior_status_callback);

		command_feedback_listener = new ROS.Subscriber(
			ns + 'flexbe/command_feedback',
			'flexbe_msgs/CommandFeedback',
			command_feedback_callback);

		onboard_heartbeat_listener = new ROS.Subscriber(
			ns + 'flexbe/heartbeat',
			'std_msgs/Empty',
			onboard_heartbeat_callback);

		ros_command_listener = new ROS.Subscriber(
			ns + 'flexbe/uicommand',
			'flexbe_msgs/UICommand',
			ros_command_callback);


		// Publisher

		behavior_start_publisher = new ROS.Publisher(
			ns + 'flexbe/request_behavior',
			'flexbe_msgs/BehaviorRequest');

		transition_command_publisher = new ROS.Publisher(
			ns + 'flexbe/command/transition',
			'flexbe_msgs/OutcomeRequest');

		autonomy_level_publisher = new ROS.Publisher(
			ns + 'flexbe/command/autonomy',
			'std_msgs/UInt8');

		preempt_behavior_publisher = new ROS.Publisher(
			ns + 'flexbe/command/preempt',
			'std_msgs/Empty');

		lock_behavior_publisher = new ROS.Publisher(
			ns + 'flexbe/command/lock',
			'std_msgs/String');

		unlock_behavior_publisher = new ROS.Publisher(
			ns + 'flexbe/command/unlock',
			'std_msgs/String');

		sync_mirror_publisher = new ROS.Publisher(
			ns + 'flexbe/command/sync',
			'std_msgs/Empty');

		attach_behavior_publisher = new ROS.Publisher(
			ns + 'flexbe/command/attach',
			'std_msgs/UInt8');

		repeat_behavior_publisher = new ROS.Publisher(
			ns + 'flexbe/command/repeat',
			'std_msgs/Empty');

		pause_behavior_publisher = new ROS.Publisher(
			ns + 'flexbe/command/pause',
			'std_msgs/Bool');

		ros_notification_publisher = new ROS.Publisher( 
			ns + 'flexbe/uinotification',
			'std_msgs/String');

		version_publisher = new ROS.Publisher(
			ns + 'flexbe/ui_version',
			'std_msgs/String',
			latched=true);
		process.nextTick(() => { version_publisher.publish({data: UI.Settings.getVersion()}); });

		// Action Clients
		if (UI.Settings.isSynthesisEnabled()) that.initializeSynthesisAction(ns);
	}

	this.initializeSynthesisAction = function(ns) {
		var topic = UI.Settings.getSynthesisTopic();
		var action_type = UI.Settings.getSynthesisType();
		if (action_type.endsWith('Action')) action_type = action_type.replace(/Action$/, "");
		if (synthesis_action_client != undefined) {
			synthesis_action_client.close();
		}
		synthesis_action_client = new ROS.ActionClient(topic, action_type);
	}

	this.shutdown = function() {
		if (heartbeat_timer != undefined) clearTimeout(heartbeat_timer);

		if (current_state_listener) current_state_listener.close();
		if (outcome_request_listener) outcome_request_listener.close();
		if (behavior_feedback_listener) behavior_feedback_listener.close();
		if (command_feedback_listener) command_feedback_listener.close();
		if (onboard_heartbeat_listener) onboard_heartbeat_listener.close();
		if (behavior_status_listener) behavior_status_listener.close();
		if (ros_command_listener) ros_command_listener.close();

		if (behavior_start_publisher) behavior_start_publisher.close();
		if (transition_command_publisher) transition_command_publisher.close();
		if (autonomy_level_publisher) autonomy_level_publisher.close();
		if (preempt_behavior_publisher) preempt_behavior_publisher.close();
		if (lock_behavior_publisher) lock_behavior_publisher.close();
		if (unlock_behavior_publisher) unlock_behavior_publisher.close();
		if (sync_mirror_publisher) sync_mirror_publisher.close();
		if (attach_behavior_publisher) attach_behavior_publisher.close();
		if (repeat_behavior_publisher) repeat_behavior_publisher.close();
		if (pause_behavior_publisher) pause_behavior_publisher.close();
		if (ros_notification_publisher) ros_notification_publisher.close();
		if (version_publisher) version_publisher.close();

		if (synthesis_action_client) synthesis_action_client.close();
	}

	this.sendBehaviorStart = function(param_keys, param_vals, autonomy) {
		if (behavior_start_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		var names = Behavior.createNames();
		IO.BehaviorPacker.loadBehaviorCode(function(code) {
			RC.Controller.signalStarted();
			RC.Sync.register("BehaviorStart", 60);

			// request start
			behavior_start_publisher.publish({
				behavior_name: Behavior.getBehaviorName(),
				autonomy_level: autonomy,
				arg_keys: param_keys,
				arg_values: param_vals,
				structure: Behavior.createStructureInfo()
			});
			RC.Sync.setProgress("BehaviorStart", 0.2, false);
		});
	}

	this.sendBehaviorUpdate = function(param_keys, param_vals, autonomy) {
		if (behavior_start_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		var names = Behavior.createNames();
		RC.Sync.register("Switch", 70);
		IO.BehaviorPacker.loadBehaviorCode(function(code) {
			//RC.Controller.signalStarted(); // would it work?

			// request start
			behavior_start_publisher.publish({
				behavior_name: Behavior.getBehaviorName(),
				autonomy_level: autonomy,
				arg_keys: param_keys,
				arg_values: param_vals,
				structure: Behavior.createStructureInfo()
			});
		});
		RC.Sync.setProgress("Switch", 0.2, false);
	}

	this.sendOutcomeRequest = function(state, outcome) {
		if (transition_command_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		var target_name = state.getStateName();
		RC.Sync.register("Transition", 50);
		transition_command_publisher.publish({
			target: target_name,
			outcome: state.getOutcomes().indexOf(outcome)
		});
		RC.Sync.setProgress("Transition", 0.2, false);
	}

	this.sendAutonomyLevel = function(level) {
		if (autonomy_level_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isRunning()) {
			RC.Sync.register("Autonomy", 30);
		}
		autonomy_level_publisher.publish({
			data: level
		});
		RC.Sync.setProgress("Autonomy", 0.2, false);
	}

	this.sendAttachBehavior = function(level) {
		if (attach_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isConnected() && RC.Controller.isExternal()) {
			RC.Sync.register("Attach", 30);

			attach_behavior_publisher.publish({
				data: level
			});
			RC.Sync.setProgress("Attach", 0.2, false);
		}
	}

	this.sendRepeatBehavior = function() {
		if (repeat_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isRunning()) {
			RC.Sync.register("Repeat", 30);
			RC.Sync.setProgress("Repeat", 0.2, false);
		}
		repeat_behavior_publisher.publish();
	}

	this.sendPauseBehavior = function() {
		if (pause_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isRunning()) {
			RC.Sync.register("Pause", 40);
			RC.Sync.setProgress("Pause", 0.2, false);
		}
		pause_behavior_publisher.publish({data: true});
	}

	this.sendResumeBehavior = function() {
		if (pause_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isRunning()) {
			if (RC.Sync.hasProcess("Pause")) {
				RC.Sync.setProgress("Pause", 0.4, false);
			}
		}
		pause_behavior_publisher.publish({data: false});
	}

	this.sendPreemptBehavior = function() {
		if (preempt_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isConnected()) {
			RC.Sync.register("Preempt", 60);
			RC.Sync.setProgress("Preempt", 0.2, false);
		}
		preempt_behavior_publisher.publish();
	}

	this.sendBehaviorLock = function(path) {
		if (lock_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (!RC.Controller.isActive()) return;

		RC.Sync.register("Lock", 50);
		lock_behavior_publisher.publish({
			data: path
		});
		RC.Sync.setProgress("Lock", 0.2, false);
	}
	this.sendBehaviorUnlock = function(path) {
		if (unlock_behavior_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (!RC.Controller.isLocked()) return;

		RC.Sync.register("Unlock", 50);
		unlock_behavior_publisher.publish({
			data: path
		});
		RC.Sync.setProgress("Unlock", 0.2, false);
	}

	this.sendSyncRequest = function() {
		if (sync_mirror_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		if (RC.Controller.isRunning() || RC.Controller.isReadonly()) {
			RC.Sync.register("Sync", 60);
		}
		sync_mirror_publisher.publish();
		RC.Sync.setProgress("Sync", 0.2, false);
	}

	this.sendRosNotification = function(cmd) {
		if (ros_notification_publisher == undefined) { T.debugWarn("ROS not initialized!"); return; }
		
		ros_notification_publisher.publish({
			data: cmd
		});
	}

	this.requestBehaviorSynthesis = function(root, system, goal, initial_condition, outcomes, result_cb, feedback_cb, timeout_cb) {
		if (synthesis_action_client == undefined) { T.debugWarn("ROS not initialized!"); return; }
		var goal = {
			request: {
				name: root,
				system: system,
				goal: goal,
				initial_condition: initial_condition,
				sm_outcomes: outcomes
			}
		};

		synthesis_action_client.send_goal(goal,
			function(result) { synthesis_action_result_callback(result, root, result_cb); },
			function(feedback) { synthesis_action_feedback_callback(feedback, root, feedback_cb); },
			RC.Controller.onboardTimeout * 1000,
			function() { synthesis_action_timeout_callback(timeout_cb); }
		);
	}

	this.DEBUG_synthesis_action_result_callback = function(result, root) {
		synthesis_action_result_callback(result, root);
	}

}) ();