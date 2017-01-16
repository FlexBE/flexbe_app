IO.Filesystem = new (function() {
	var that = this;

	var fs = require('fs');
	var path = require('path');

	var toArray = function (list) {
		return Array.prototype.slice.call(list || [], 0);
	}


	this.getFileName = function(file_path, extension) {
		var name_with_extension = path.basename(file_path);

		if (extension)
			return name_with_extension;

		return name_with_extension.replace(path.extname(name_with_extension), "");
	}

	this.createFolder = function(parent_path, name, callback) {
		fs.mkdir(path.join(parent_path, name), (err) => {
			if (err != undefined && err.code != 'EEXIST') {
				T.logError(err);
				return;
			}
			callback();
		});
	}

	this.createFile = function(parent_path, name, content, callback) {
		var file_path = path.join(parent_path, name);
		fs.writeFile(file_path, content, (err) => {
			if (err != undefined) {
				T.logError(err);
				return;
			}
			callback();
		});
	}

	this.readFile = function(file_path, callback) {
		fs.readFile(file_path, (err, data) => {
			if (err != undefined) {
				T.logError(err);
				return;
			}
			callback(String(data));
		});
	}

	this.checkFolderExists = function(parent_path, name, callback) {
		fs.stat(path.join(parent_path, name), (stats) => {
			callback(stats.isDirectory());
		});
	}

	this.checkFileExists = function(parent_path, name, callback) {
		fs.access(path.join(parent_path, name), (err) => {
			callback(err == undefined);
		});
	}

	this.isFolder = function(file_path) {
		return fs.statSync(file_path).isDirectory();
	}

	this.isFile = function(file_path) {
		return fs.statSync(file_path).isFile();
	}

	this.getFileContent = function(parent_path, name, callback) {
		that.getFolderContent(parent_path, function(entries) {
			var file_entry = toArray(entries).findElement(function(element) {
				return that.isFile(element) && path.basename(element) == name;
			});
			if (file_entry == undefined) {
				T.logError("file " + name + " not found in the specified folder");
				return;
			}

			that.readFile(file_entry, callback);
		});
	}

	this.getFolderContent = function(parent_path, callback) {
		fs.readdir(parent_path, (err, files) => {
			if (err != undefined) {
				T.logError(err);
				return;
			}
			for (var i=0; i<files.length; i++) {
				files[i] = path.join(parent_path, files[i]);
			}
			callback(files);
		});
	}


}) ();