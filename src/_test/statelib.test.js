StatelibTest = new (function() {
    var that = this;

    this.testWaitStateAvailable = function(report, done) {
        // flexbe_states/WaitState is found and properly imported
        var result = WS.Statelib.getClassList().contains("WaitState");
        report.assertTrue.test_statelib = result;
        done();
    }

})();