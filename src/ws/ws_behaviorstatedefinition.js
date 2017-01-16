WS.BehaviorStateDefinition = function(manifest, outcomes, input_keys, output_keys, bsm_loaded_callback) {
	var that = this;

	var autonomy = [];
	for (var i = 0; i < outcomes.length; ++i) {
		autonomy.push(-1);
	};
	var path = manifest.rosnode_name + "." + IO.Filesystem.getFileName(manifest.codefile_name, false);
	var behavior_name = manifest.name;
	var behavior_manifest = manifest;
	var behavior_tag_list = manifest.tags.replace(/[,;]/g, " ").replace(/\s+/g, " ").split(" ");
	var bsm_parsing_result = undefined;

	IO.BehaviorLoader.parseBehaviorSM(behavior_manifest, function(parsing_result) {
		bsm_parsing_result = parsing_result;
		if (bsm_loaded_callback != undefined) bsm_loaded_callback();
	});

	this.__proto__ = new WS.StateDefinition(manifest.class_name, new WS.Documentation(manifest.description),
		path, [], outcomes, input_keys, output_keys, [], autonomy, []);

	this.getBehaviorName = function() { return behavior_name; }
	this.getBehaviorManifest = function() { return behavior_manifest; }
	this.getBehaviorDesc = function() { return behavior_manifest.description; }
	this.getBehaviorTagList = function() { return behavior_tag_list; }
	this.cloneBehaviorStatemachine = function() {
		return IO.ModelGenerator.buildStateMachine(bsm_parsing_result.container_name, bsm_parsing_result.container_sm_var_name, 
												bsm_parsing_result.sm_defs, bsm_parsing_result.sm_states, true);
	}
};