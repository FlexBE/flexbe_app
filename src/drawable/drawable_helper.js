Drawable.Helper = new (function() {
	var that = this;

	var ox = 0, oy = 0, lx = 0, ly = 0;
	var traversed_positions = [];

	this.intersectsAnyOther = function(target_drawing, target_object) {
		intersects = false;
		drawings = UI.Statemachine.getAllDrawings().filter(function (element) {
			return element.obj.getStateName != undefined;
		});
		for (var i = 0; i < drawings.length; ++i) {
			// not compare with state self
			if (drawings[i].obj.getStateName() == target_object.getStateName())
				continue;
			if (Raphael.isBBoxIntersect(target_drawing.getBBox(), drawings[i].drawing.getBBox()) ) {
				intersects = true;
				return drawings[i];
			}
		}
		return false;
	}

	this.placeBeneath = function(target_object, target_drawing, other_drawing) {
		var o = other_drawing.getBBox();
		var t = target_drawing.getBBox();
		var m = 10; // margin

		var pos_above = o.y - m - t.height;
		var pos_right = o.x2 + m;
		var pos_below = o.y2 + m;
		var pos_left  = o.x - m - t.width;

		var transformations = [];
		transformations.push({ x: pos_right - t.x, y: 0 });
		if (pos_above > 20)
			transformations.push({ x: 0, y: pos_above - t.y });
		transformations.push({ x: 0, y: pos_below - t.y });
		if (pos_left > 0)
			transformations.push({ x: pos_left - t.x, y: 0 });

		var trans = transformations.reduce(function(prev, cur, i, a) {
			if(Math.abs(prev.x) + Math.abs(prev.y) < Math.abs(cur.x) + Math.abs(cur.y)) {
				return (traversed_positions.findElement(function(element, j) {
						return element.x == t.x + prev.x && element.y == t.y + prev.y;
					}) == undefined)? prev : cur;	
			} else {
				return (traversed_positions.findElement(function(element, j) {
						return element.x == t.x + cur.x && element.y == t.y + cur.y;
					}) == undefined)? cur : prev;	
			}
		});

		traversed_positions.push({ x: t.x + trans.x, y: t.y + trans.y });

		target_drawing.translate(trans.x, trans.y);
		target_object.getPosition().x += trans.x;
		target_object.getPosition().y += trans.y;
	}

	this.initialIntersectCheck = function(target_drawing, target_object) {
		var intersects = false;
		var temp = 0;
		traversed_positions = [];
		do {
			var other = that.intersectsAnyOther(target_drawing, target_object);
			intersects = other !== false;
			
			if (intersects) {
				that.placeBeneath(target_object, target_drawing, other.drawing);
				//target_drawing.translate(20, 0);
				//target_object.getPosition().x += 20;
			}
			temp++;
		} while(intersects && temp < 15);
		traversed_positions = [];
	}

	this.snapToCenter = function(x, y, w, h) {
		var gridsize = UI.Settings.getGridsize();
		var offset = {x: UI.Statemachine.getPanShift().x % gridsize, y: UI.Statemachine.getPanShift().y % gridsize};
		return {
			x: Raphael.snapTo(gridsize, x, gridsize / 2 + 1) - w/2 + gridsize + offset.x,
			y: Raphael.snapTo(gridsize, y, gridsize / 2 + 1) - h/2 + offset.y
		};
	}

	// Raphael func
	// state - object representing the state
	this.viewStateProperties = function() {
		if (!UI.Statemachine.isConnecting())
			UI.Panels.StateProperties.displayStateProperties(this.data("state"));
		else
			UI.Statemachine.connectTransition(this.data("state"));
	}

	// Raphael func
	// state - object representing the statemachine
	this.enterStatemachine = function() {
		if (!UI.Statemachine.isConnecting()) {
			UI.Statemachine.setDisplayedSM(this.data("state"));
			UI.Panels.hidePanelIfActive(UI.Panels.STATE_PROPERTIES_PANEL);
		}
		else {
			UI.Statemachine.connectTransition(this.data("state"));
		}
	}

	// Raphael func
	// state - object representing the behavior
	this.enterBehavior = function() {
		if (!UI.Statemachine.isConnecting()) {
			UI.Statemachine.setDisplayedSM(this.data("state").getBehaviorStatemachine());
			UI.Panels.hidePanelIfActive(UI.Panels.STATE_PROPERTIES_PANEL);
		}
		else {
			UI.Statemachine.connectTransition(this.data("state"));
		}
	}

	// Raphael func
	// state - object representing the state
	// box - rectangle matching the size of the movable object
	this.moveFnc = function(dx, dy, x, y, evt) {
		if (UI.Statemachine.isConnecting()) return;
		lx = dx + ox;
		ly = dy + oy;
		lx = Math.min(Math.max(lx, 0), UI.Statemachine.getR().width - this.data("box").attr("width"));
		ly = Math.min(Math.max(ly, 0), UI.Statemachine.getR().height - this.data("box").attr("height"));
		var i_pos = evt.shiftKey? that.snapToCenter(lx, ly, this.data("box").attr("width"), this.data("box").attr("height")) : {x: lx, y: ly};
		UI.Statemachine.getDragIndicator().attr({x: i_pos.x, y: i_pos.y, opacity: 1,
				width: this.data("box").attr("width"),
				height: this.data("box").attr("height")});
		if(that.intersectsAnyOther(UI.Statemachine.getDragIndicator(), this.data("state")))
			UI.Statemachine.getDragIndicator().attr({'stroke': '#F00', 'fill': 'rgba(100%, 0%, 0%, 50%)'});
		else
			UI.Statemachine.getDragIndicator().attr({'stroke': '#000', 'fill': 'rgba(50%, 100%, 40%, 15%)'});
	}

	// Raphael func
	// state - object representing the state
	this.startFnc = function() {
		if (UI.Statemachine.isConnecting()) return;
		lx = this.data("state").getPosition().x + UI.Statemachine.getPanShift().x;
		ly = this.data("state").getPosition().y + UI.Statemachine.getPanShift().y;
		ox = this.data("state").getPosition().x + UI.Statemachine.getPanShift().x;
		oy = this.data("state").getPosition().y + UI.Statemachine.getPanShift().y;
	}

	// Raphael func
	// state - object representing the state
	this.endFnc = function(evt) {
		if (UI.Statemachine.isConnecting()) return;
		var state = this.data("state");
		var container = state.getContainer();
		var state_name = state.getStateName();
		var old_pos = state.getPosition();
		var new_pos = (UI.Statemachine.getDragIndicator().attr('width') > 1)?
			UI.Statemachine.getDragIndicator().attr(['x', 'y']):
			state.getPosition();
		new_pos.x -= UI.Statemachine.getPanShift().x;
		new_pos.y -= UI.Statemachine.getPanShift().y;

		UI.Statemachine.getDragIndicator().attr({x: 0, y: 0, opacity: 0, width: 1, height: 1});
		state.setPosition(new_pos);
		UI.Statemachine.refreshView();

		if(container == undefined) return;
		var container_path = container.getStatePath();

		var move_distance = Math.round(Math.sqrt(Math.pow(new_pos.x - old_pos.x, 2) + Math.pow(new_pos.y - old_pos.y, 2)));
		if (move_distance < 1) return;

		ActivityTracer.addActivity(ActivityTracer.ACT_STATE_CHANGE,
			"Moved " + state.getStateName().split('#')[0] + " for " + move_distance + " px",
			function() {
				var container = (container_path == "")? Behavior.getStatemachine() : Behavior.getStatemachine().getStateByPath(container_path);
				var state = container.getStateByName(state_name);
				if (state == undefined) state = container.getSMOutcomeByName(state_name);
				state.setPosition(old_pos);
				UI.Statemachine.refreshView();
			},
			function() {
				var container = (container_path == "")? Behavior.getStatemachine() : Behavior.getStatemachine().getStateByPath(container_path);
				var state = container.getStateByName(state_name);
				if (state == undefined) state = container.getSMOutcomeByName(state_name);
				state.setPosition(new_pos);
				UI.Statemachine.refreshView();
			}
		);
	}

	// Raphael func
	// state - object representing the state
	// label - name of the transition to begin
	this.beginTransition = function() {
		if (RC.Controller.isReadonly()) return;

		if (!UI.Statemachine.isConnecting())
			UI.Statemachine.beginTransition(this.data("state"), this.data("label"));
		else
			UI.Statemachine.connectTransition(this.data("state"));
	}

	// Raphael func
	// state - object representing the state
	this.connectTransition = function() {
		UI.Statemachine.connectTransition(this.data("state"));
	}

}) ();