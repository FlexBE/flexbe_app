CheckBehaviorsReport = new (function() {

    var report = {'assertTrue': {}};

    this.checkAllBehaviors = function(callback) {
        var behaviors = WS.Behaviorlib.getBehaviorList();

        var next = function(){
            if (behaviors.length == 0){
                IO.Filesystem.createFile('/tmp', "flexbe_app_behavior_report.log", JSON.stringify(report), callback);
            } else {
                var behavior = behaviors.shift();
                var m = WS.Behaviorlib.getByName(behavior).getBehaviorManifest();

                IO.BehaviorLoader.loadBehavior(m, function(error_string) {
                    console.log(error_string);
                    if (error_string != undefined) {
                        report.assertTrue["behavior_"+behavior] = false;
                        console.log(behavior+" is false");
                    }
                    else{
                        report.assertTrue["behavior_"+behavior] = true;
                        console.log(behavior+" is true");
                    }
                    next();
                });
            }
        }
        next();
    }
})();
