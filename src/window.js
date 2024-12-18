window.onload = function() {
	const { app } = require('electron');
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
	if (!process.argv.contains('--offline') && !process.argv.contains('-o')) {
		RC.ROS.trySetupConnection();
	} else {
		T.logInfo("Running in offline mode: please connect to ROS manually if desired.");
	}

	// Restore local settings (including statelib)
	UI.Settings.restoreSettings();

	UI.Feed.initialize();

	if (process.argv.contains('--run-tests')) {
		setTimeout(() => {
			TestReport.runAllTests(status =>  app.quit());
		}, 5 * 1000);
	}

	else if (process.argv.contains('--check-behaviors')) {
		setTimeout(() => {
			CheckBehaviorsReport.checkAllBehaviors(app.quit);
		}, 5 * 1000);
	}
}
