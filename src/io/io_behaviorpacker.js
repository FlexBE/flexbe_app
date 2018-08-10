IO.BehaviorPacker = new (function() {
	var that = this;

	var path = require('path');

	this.loadBehaviorCode = function(callback) {
		var names = Behavior.createNames();
		var package_name = names.rosnode_name;
		ROS.getPackagePath(package_name, (folder_path) => {
			if (folder_path == undefined) {
				return;
			}
			var file_path = path.join(folder_path, names.file_name);
			IO.Filesystem.checkFileExists(folder_path, names.file_name, (exists) => {
				if (exists) {
					IO.Filesystem.readFile(file_path, callback);
				} else {
					process.nextTick(() => {
						callback();
					});
				}
			});
		});
	}

}) ();