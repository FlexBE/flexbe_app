console.log("### Test Cases Loaded ###");

test_simpleSM = function() {
	UI.Menu.toStatemachineClicked();

	state = new State("Log_Anything", Statelib.getFromLib("LogState"));
	state.getPosition().x += 100;
	state.getPosition().y += 20;
	UI.Statemachine.getRoot().addState(state);

	UI.Statemachine.refreshView();
}

test_simpleCP = function() {
	UI.Menu.toStatemachineClicked();

	var state1 = new State("Log_Anything", Statelib.getFromLib("LogState"));
	state1.getPosition().x += 100;
	state1.getPosition().y += 20;
	Behavior.getStatemachine().addState(state1);

	var state2 = new State("Bla", Statelib.getFromLib("CalculationState"));
	state2.getPosition().x += 400;
	state2.getPosition().y += 30;
	Behavior.getStatemachine().addState(state2);

	UI.Statemachine.beginTransition(state1, "done");
	UI.Statemachine.connectTransition(state2);
}

test_runtimeControlDisplay = function() {
	UI.Menu.toStatemachineClicked();

	state = new State("Calc_Pose", Statelib.getFromLib("CalculationState"));
	state.getPosition().x += 100;
	state.getPosition().y += 40;
	UI.Statemachine.getRoot().addState(state);

	state = new State("Grasp_Object", Statelib.getFromLib("GraspTemplateState"));
	state.getPosition().x += 250;
	state.getPosition().y += 40;
	UI.Statemachine.getRoot().addState(state);

	state = new State("Log_Success", Statelib.getFromLib("LogState"));
	state.getPosition().x += 400;
	state.getPosition().y += 10;
	UI.Statemachine.getRoot().addState(state);

	state = new State("Log_Failure", Statelib.getFromLib("LogState"));
	state.getPosition().x += 400;
	state.getPosition().y += 80;
	UI.Statemachine.getRoot().addState(state);

	UI.Menu.toStatemachineClicked();
}


test_executeAll = function() {

}

test_normalRun = function() {
	var manifest = Behaviorlib.getByName("FlexBE Test Behavior").getBehaviorManifest();
	BehaviorLoader.loadBehavior(manifest);
	UI.Settings.connectRosbridgeClicked();
	UI.Menu.toControlClicked();
	setTimeout(function () {
		UI.RuntimeControl.startBehaviorClicked();
	}, 2000);
}

test_synthesisResult = function() {
	RC.PubSub.DEBUG_synthesis_action_result_callback({
		error_code: {
			value: 1
		},
		states: [
			{
				state_path: "/",
				state_class: ":STATEMACHINE",
				initial_state_name: "Entry_Msg",
				behavior_class: "",
				parameter_names: [],
				parameter_values: [],
				input_keys:[],
				output_keys: [],
				position: [0,0],
				outcomes: ['finished', 'failed'],
				transitions: [],
				autonomy: [],
				cond_outcome: [],
				cond_transition: [],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Wait_A_Bit",
				state_class: "WaitState",
				initial_state_name: "",
				behavior_class: "",
				parameter_names: ['wait_time'],
				parameter_values: ['2'],
				position: [0,0],
				outcomes: ['done'],
				transitions: ['Con1'],
				autonomy: [2],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Entry_Msg",
				state_class: "LogState",
				initial_state_name: "",
				behavior_class: "",
				parameter_names: ['text'],
				parameter_values: ['"Now at inner behavior..."'],
				position: [0,0],
				outcomes: ['done'],
				transitions: ['Testprint_Behavior'],
				autonomy: [0],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Con1",
				state_class: ":STATEMACHINE",
				initial_state_name: "Con",
				behavior_class: "",
				parameter_names: [],
				parameter_values: [],
				input_keys:[],
				output_keys: [],
				position: [],
				outcomes: ['finished'],
				transitions: ['Con2'],
				autonomy: [],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Con1/Con",
				state_class: ":STATEMACHINE",
				initial_state_name: "Test_1",
				behavior_class: "",
				parameter_names: [],
				parameter_values: [],
				input_keys:[],
				output_keys: [],
				position: [0,0],
				outcomes: ['finished'],
				transitions: ['finished'],
				autonomy: [],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Con2",
				state_class: ":STATEMACHINE",
				initial_state_name: "Con",
				behavior_class: "",
				parameter_names: [],
				parameter_values: [],
				input_keys:[],
				output_keys: [],
				position: [0,0],
				outcomes: ['finished'],
				transitions: ['Con2'],
				autonomy: [],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Con2/Con",
				state_class: ":STATEMACHINE",
				initial_state_name: "Test_2",
				behavior_class: "",
				parameter_names: [],
				parameter_values: [],
				input_keys:[],
				output_keys: [],
				position: [0,0],
				outcomes: ['finished'],
				transitions: ['finished'],
				autonomy: [],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Con1/Con/Test_1",
				state_class: "WaitState",
				initial_state_name: "",
				behavior_class: "",
				parameter_names: ['wait_time'],
				parameter_values: ['2'],
				position: [0,0],
				outcomes: ['done'],
				transitions: ['finished'],
				autonomy: [2],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Con2/Con/Test_2",
				state_class: "WaitState",
				initial_state_name: "",
				behavior_class: "",
				parameter_names: ['wait_time'],
				parameter_values: ['2'],
				position: [0,0],
				outcomes: ['done'],
				transitions: ['finished'],
				autonomy: [2],
				userdata_keys: [],
				userdata_remapping: []
			},
			{
				state_path: "/Print_State",
				state_class: "CalculationState",
				initial_state_name: "",
				behavior_class: "",
				parameter_names: ['calculation'],
				parameter_values: ['lambda x: 0'],
				position: [0,0],
				outcomes: ['done'],
				transitions: ['finished'],
				autonomy: [0],
				userdata_keys: ['input_value', 'output_value'],
				userdata_remapping: ['my_input', 'output_value']
			},
			{
				state_path: "/Testprint_Behavior",
				state_class: ":BEHAVIOR",
				initial_state_name: "",
				behavior_class: "FlexBETestprintBehaviorSM",
				parameter_names: [],
				parameter_values: [],
				position: [0,0],
				outcomes: ['finished'],
				transitions: ['Wait_A_Bit'],
				autonomy: [-1],
				userdata_keys: [],
				userdata_remapping: []
			}
		]
	},
	"/Synthesis_Test");
	UI.Menu.toStatemachineClicked();
}