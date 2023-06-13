CommandLib = new (function() {
	var that = this;

	var command_library = [
		{
			desc: "commands",
			match: /^(cmd|commands|help)$/,
			impl: function(args) { 
				UI.Tools.printAvailableCommands();
				UI.Tools.notifyRosCommand('commands');
			},
			text: "Lists all available commands."
		},
		{
			desc: "findStateUsage [state_type]",
			match: /^findStateUsage[( ]["']?([a-zA-Z0-9_]+\.[a-zA-Z0-9_]+)["']?\)?$/,
			impl: function(args) {
				Scripts.findStateUsage(args[1]);
				UI.Tools.notifyRosCommand('findStateUsage');
			},
			text: "Searches in all behaviors for instantiations of the given type.",
			completions: [function() { return WS.Statelib.getTypeList(); }]
		},
		{
			desc: "statemachine [new_name]",
			match: /^statemachine[( ]?["']?([a-zA-Z0-9_]+)?["']?\)?$/,
			impl: function(args) {
				if (UI.Statemachine.isReadonly()) return;
				Tools.createStatemachine(args[1]);
				UI.Tools.notifyRosCommand('statemachine');
			},
			text: "Creates a new state machine with the given name out of all selected states.",
			completions: [function() { return UI.Statemachine.getSelectedStates().map(function(s) { return s.getStateName(); }); }]
		},
		{
			desc: "synthesize [sm_name] i: [initial] g: [goal]",
			match: /^synthesize ([a-zA-Z0-9_]+) i: ?(.+?) g: ?(.+?)$/,
			impl: function(args) {
				if (UI.Statemachine.isReadonly()) return;
				var initial_condition = args[2];
				var goal = args[3];
				var path = UI.Statemachine.getDisplayedSM().getStatePath() + "/" + args[1];
				RC.PubSub.requestBehaviorSynthesis(path, UI.Settings.getSynthesisSystem(), goal, initial_condition, ['finished', 'failed'], function() { UI.Tools.notifyRosCommand('synthesize'); });
			},
			text: "Synthesizes a new state machine.",
			completions: [
				function() { return UI.Statemachine.getDisplayedSM().getStates().filter(function(s) { return s instanceof Statemachine; }).map(function(sm) { return sm.getStateName(); }) },
				function() { return ['step', 'stand_prep', 'stand', 'manipulate', 'walk']; },
				function() { return ['step', 'stand_prep', 'stand', 'manipulate', 'walk']; }
			]
		},
		{
			desc: "note [text]",
			match: /^note ?([^\n]+)?$/,
			impl: function(args) {
				if (UI.Statemachine.isReadonly()) return;
				var text = (args[1] != undefined)? args[1] : '';
				var note = new Note(text);
				note.setContainerPath(UI.Statemachine.getDisplayedSM().getStatePath());
				Behavior.addCommentNote(note);
				UI.Statemachine.refreshView();
				UI.Tools.notifyRosCommand('note');
			},
			text: "Adds a new note to the currently displayed state machine."
		},
		{
			desc: "autoconnect",
			match: /^autoconnect$/,
			impl: function(args) {
				if (UI.Statemachine.isReadonly()) return;
				Tools.autoconnect();
				UI.Tools.notifyRosCommand('autoconnect');
			},
			text: "Automatically connects obvious outcomes."
		},
		{
			desc: "autolayout",
			match: /^autolayout$/,
			impl: function(args) {
				if (UI.Statemachine.isReadonly()) return;
				UI.Statemachine.applyGraphLayout();
				UI.Statemachine.refreshView();
				UI.Tools.notifyRosCommand('autolayout');
			},
			text: "Applies a force-based graph layout to arrange states."
		},
		{
			desc: "save",
			match: /^save$/,
			impl: function(args) {
				UI.Menu.saveBehaviorClicked();
				T.show(); // make sure terminal does not get collapsed by saving
			},
			text: "Saves the current behavior."
		},
		{
			desc: "load [behavior]",
			match: /^load ([^\n]+)$/,
			impl: function(args) {
				if (RC.Controller.isRunning()) {
					T.logWarn('Unable to load a behavior while executing another one.');
					return;
				}
				var manifest = WS.Behaviorlib.getByName(args[1]).getBehaviorManifest();
				IO.BehaviorLoader.loadBehavior(manifest);
				UI.Menu.toDashboardClicked();
			},
			text: "Loads the behavior with the given name.",
			completions: [
				function() { 
				    return WS.Behaviorlib.getBehaviorList().map(function(element, index){return element.getBehaviorName();});
				}
			]
		},
		{
			desc: "update [behavior]",
			match: /^update ([^\n]+)$/,
			impl: function(args) {
				if (RC.Controller.isReadonly()) {
					T.logWarn("Cannot update a behavior while executing another one.");
					return;	
				}
				var be_name = args[1];
				if (be_name == Behavior.getBehaviorName()) {
					T.logWarn("Cannot update the behavior which is currently loaded. Please use 'load "+be_name+"' instead.");
					return;
				}
				WS.Behaviorlib.updateEntry(WS.Behaviorlib.getByName(be_name), function() {
					// TODO: update behavior state machine where required
					UI.Tools.notifyRosCommand('update');
				});
			},
			text: "Updates the implementation of a behavior in the background (except the one currently loaded).",
			completions: [
				function() { return WS.Behaviorlib.getBehaviorList().map(function(element, index){return element.getBehaviorName();})
				.filter(function(be) { return be != Behavior.getBehaviorName(); }); }
			]
		},
		{
			desc: "attach [autonomy_level]",
			match: /^attach ?(-?\d+)?$/,
			impl: function(args) {
				var selection_box = document.getElementById("selection_rc_autonomy");
				var autonomy_level = (args[1] != undefined)? parseInt(args[1]) : parseInt(selection_box.options[selection_box.selectedIndex].value);
				if (!RC.Controller.isActive()) {
					if (!RC.Controller.isExternal()) {
						T.logWarn('No behavior running to attach to.');
						return;
					}
					RC.PubSub.sendAttachBehavior(autonomy_level);

					UI.RuntimeControl.displayBehaviorFeedback(4, "Attaching to behavior...");
				} else {
					T.logInfo('Already attached, only updating autonomy level.');
					UI.Tools.notifyRosCommand('attach');
				}
				UI.Menu.toControlClicked();

				// update the autonomy level selection box to reflect the newly set level
				for (var optionIndex = 0; optionIndex < selection_box.options.length; optionIndex++) {
				    if (parseInt(selection_box.options[optionIndex].value) == autonomy_level) {
				        selection_box.selectedIndex = optionIndex;
				        break;
				    }
				}
				UI.RuntimeControl.updateAutonomySelectionBoxColor();
			},
			text: "Attaches the GUI to a running behavior if possible."
		},
		{
			desc: "lock [level]",
			match: /^lock ?(-?\d+)?$/,
			impl: function(args) {
				if (!RC.Controller.isRunning() || !RC.Controller.isActive()) {
					T.logWarn('No unlocked behavior running, ignoring command.');
					return;
				}
				var idx = (args[1] != undefined)? parseInt(args[1]) : 0;
				var sel = document.getElementById("selection_rc_lock_layer");
				idx = (idx < 0)? idx + sel.options.length : idx;
				if (idx < 0 || idx >= sel.options.length) {
					T.logWarn('Selected lock level out of range, ignoring command.');
					return;
				}
				sel.selectedIndex = idx;
				UI.RuntimeControl.behaviorLockClicked();
			},
			text: "Locks the currently running behavior."
		},
		{
			desc: "unlock",
			match: /^unlock$/,
			impl: function(args) {
				if (!RC.Controller.isRunning() || RC.Controller.needSwitch()) {
					T.logWarn('No locked behavior without changes running, ignoring command.');
					return;
				}
				UI.RuntimeControl.behaviorLockClicked();
			},
			text: "Unlocks the currently locked behavior."
		},
		{
			desc: "switch",
			match: /^(switch|goforit)$/,
			impl: function(args) {
				if (!RC.Controller.isRunning() || RC.Controller.isActive() || !RC.Controller.needSwitch()) {
					T.logWarn('No locked behavior running which requires a switch, ignoring command.');
					return;
				}
				UI.RuntimeControl.behaviorLockClicked();
			},
			text: "Applies runtime modifications to the currently locked behavior."
		},
		{
			desc: "edit [path]",
			match: /^edit ?([^\n]+)?$/,
			impl: function(args) {
				var path = (args[1] != undefined)? args[1] : '/';
				var sm = (path == '/')? Behavior.getStatemachine()
					: Behavior.getStatemachine().getStateByPath(path);
				if (sm == undefined || !(sm instanceof Statemachine)) {
					T.logWarn('Given path ' + path + ' does not refer to a statemachine.');
					return;
				}
				UI.Statemachine.setDisplayedSM(sm);
				UI.Menu.toStatemachineClicked();
				UI.Tools.notifyRosCommand('edit');
			},
			text: "Opens the container given by the specified path in the editor."
		}
	];


	this.load = function() {
		return command_library;
	}

}) ();
