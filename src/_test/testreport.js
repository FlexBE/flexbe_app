TestReport = new (function() {
    var that = this;

    var report = {'assertTrue': {}};

    this.runAllTests = function(callback) {
        var testCases = [
            StatelibTest.testWaitStateAvailable
        ];
        var done = function(code) {
            if (testCases.length == 0) {
                IO.Filesystem.createFile('/tmp', "flexbe_app_report.log", JSON.stringify(report), callback);
            } else {
                var test = testCases.shift();
                test(report, done);
            }
        }
        done();
    }

})();