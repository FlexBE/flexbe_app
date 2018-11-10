window.onload = function() {
	var gui = require('nw.gui');

	Behavior.resetBehavior();
	
	// Initialize gui panel
	UI.Statemachine.initialize();
	UI.Menu.toDashboardClicked();
	UI.Dashboard.resetAllFields();
	UI.Dashboard.addBehaviorOutcome('finished');
	UI.Dashboard.addBehaviorOutcome('failed');
	ActivityTracer.resetActivities();
	UI.RuntimeControl.displayLockBehavior();
	
	RC.Controller.initialize();

	// Initialize runtime control
	if (!gui.App.argv.contains('--offline') && !gui.App.argv.contains('-o')) {
		RC.ROS.trySetupConnection();
	} else {
		T.logInfo("Running in offline mode: please connect to ROS manually if desired.");
	}

	// Restore local settings (including statelib)
	UI.Settings.restoreSettings();

	UI.Feed.initialize();

	if (gui.App.argv.contains('--run-tests')) {
		setTimeout(() => {
			TestReport.runAllTests(status =>  gui.App.quit());
		}, 5 * 1000);
	}
}
