IO.BehaviorSaver = new (function() {
	var that = this;

	var path = require('path');

	var storeBehaviorCode = function(generated_code, callback) {
		var create_callback = function(folder) {
			IO.Filesystem.createFile(folder, names.file_name, generated_code, function() {
				callback();
			});
		};
		var names = Behavior.createNames();
		var package_name = names.rosnode_name;
		ROS.getPackagePath(package_name, (package_path) => {
			var folder_path = path.join(package_path, 'src', package_name);
			var file_path = path.join(folder_path, names.file_name);
			var file_tmp_path = path.join(folder_path, names.file_name_tmp);
			if (RC.Controller.isConnected()) {
				IO.Filesystem.checkFileExists(folder_path, names.file_name_tmp, function(tmp_exists) {
					if (!tmp_exists) {
						IO.Filesystem.checkFileExists(folder_path, names.file_name_tmp, function(src_exists) {
							if (src_exists) {
								IO.Filesystem.getFileContent(folder_path, names.file_name, function(content_onboard) {
									IO.Filesystem.createFile(folder_path, names.file_name_tmp, content_onboard, function() { 
										create_callback(folder_path);
									});
								});
							} else {
								create_callback(folder_path);
							}
						});
					} else {
						create_callback(folder_path);
					}
				});
			} else {
				create_callback(folder_path);
			}
			
		});
	}

	var storeBehaviorManifest = function(generated_manifest, callback) {
		var names = Behavior.createNames();
		var package_name = names.rosnode_name;
		if (names.manifest_path != undefined) {
			var folder_path = path.dirname(names.manifest_path);
			var file_name = path.basename(names.manifest_path);
			IO.Filesystem.createFile(folder_path, file_name, generated_manifest, function() { 
				callback();
			});
		} else {
			ROS.getPackagePath(package_name, (package_path) => {
				var folder_path = path.join(package_path, 'manifest');
				IO.Filesystem.createFile(folder_path, names.manifest_name, generated_manifest, function() { 
					callback();
				});
			});
		}
	}

    String.prototype.format = function() {
        var formatted = this;
        for( var arg in arguments ) {
            formatted = formatted.replace("{" + arg + "}", arguments[arg]);
        }
        return formatted;
    };

	var saveSuccessCallback = function() {
		T.logInfo("Save successful!");
		UI.Panels.Terminal.hide();
		UI.Settings.updateBehaviorlib();
		UI.Tools.notifyRosCommand('save');
    }

    var saveOnRobot = function() {
        var names = Behavior.createNames();
        var package_name = names.rosnode_name;
        var ssh_username = UI.Settings.getRobotUsername();
        var ssh_hostname = UI.Settings.getRobotHostname();
        var ssh_package_path = ""

        if(ssh_username === "" || ssh_hostname === "" /*|| ssh_password === ""*/) {
            T.logError("SSH configurations are not well defined, fix them from Configuration Tab -> Code Generation Box");
            return;
        }

        var ssh_password = window.prompt("Please enter {0}:{1} password: ".format(ssh_username, ssh_hostname));
        if(ssh_password == null || ssh_password == "") {
            return;
        }

        ROS.getPackagePath(package_name, (package_path) =>
        {
            try
            {
                var SSHClient = require('ssh2').Client;
                conn = new SSHClient();
                conn.on('ready', function() {
                  var ros_distro = process.env.ROS_DISTRO;
                  conn.exec('bash -c "source .profile && rospack find {1}"'.format(ros_distro, package_name), function(err, stream) {
                    if (err) {
                        T.logError(err);
                    }
                    stream.on('close', function(code, signal) {
                      conn.end();
                    })
                    .on('data', function(remote_package_path) {
                      ssh_package_path = remote_package_path;
                      T.logInfo('Path to package on the robot: ' + ssh_package_path);
                      conn.end()
                      var scp_client = require('scp2')
                      var ssh_query = '{0}:{1}@{2}:{3}/'.format(
                          ssh_username,
                          ssh_password,
                          ssh_hostname,
                          ssh_package_path).replace(/\s+/, "");
                      scp_client.scp(package_path+"/", String(ssh_query), function(err) {
                        var err_str = String(err);
                        if( err_str != 'undefined') {
                            T.logError("scp failed "+err);
                        }
                      });
                    }).stderr.on('data', function(data) {
                      T.logError(data+", Make sure robot's ROS environment is configured properly in \".profile\" script");
                    })
                  });
                })
                .on('error', function(err) {
                        T.logError(err);
                        T.logError("Please check your ssh configuration and entered password.");
                 })
                 .connect({
                  host: ssh_hostname,
                  username: ssh_username,
                  password: ssh_password
                });
            }
            catch(ex) {
                T.logError("An exception occurred: "+ex.message+", Make sure you installed the dependencies properly!")
            }
        })
    }

	this.saveStateMachine = function() {
		T.clearLog();
		UI.Panels.Terminal.show();

		IO.BehaviorLoader.updateManualSections(() => {
			// generate sourcecode
			var generated_code = "";
			try {
				generated_code = IO.CodeGenerator.generateBehaviorCode();
				T.logInfo("Code generation completed.");
			} catch (err) {
				T.logError("Code generation failed: " + err);
				return;
			}

			// generate manifest
			var generated_manifest = "";
			try {
				generated_manifest = IO.ManifestGenerator.generateManifest();
				T.logInfo("Manifest generation completed.");
			} catch (err) {
				T.logError("Manifest generation failed: " + err);
				return;
			}

            // store in file
            storeBehaviorCode(generated_code, () => {
                // make sure code file exists before creating the manifest
                // this reduces the risk for orphan manifests
                storeBehaviorManifest(generated_manifest, () => {
                    if(UI.Settings.isSaveOnRobotEnabled()) {
                        //Save/Overwrite the behavior on the robot
                        saveOnRobot();
                    }
                    saveSuccessCallback();
                });
            });
        });
    }

}) ();
