Drawable.Transition = function(transition_obj, target_paper, readonly, drawings, highlight, outline, path_type, color) {
	var that = this;

	var paper = target_paper;
	var transition_colors = ['#998', '#99f', '#9d9', '#faa'];
	var transition_text_colors = ['#554', '#559', '#585', '#966'];
	var transition_highlight_color = '#FC5'

	var resetTransition = function() {
		if (RC.Controller.isReadonly()) return;
		
		UI.Statemachine.resetTransition(transition_obj);
	}

	var text_color = color;
	if (outline) {
		color = '#EEE';
	} else if (color == undefined) {
		color = highlight? transition_highlight_color : transition_colors[Math.max(0, transition_obj.getAutonomy())];
		text_color = transition_text_colors[Math.max(0, transition_obj.getAutonomy())];
		if (transition_obj.getFrom().getStateName() == "INIT") {
			color = '#000';
		}
	}

	from = drawings.findElement(function(element) {
		return element.obj instanceof State && element.obj.getStateName() == transition_obj.getFrom().getStateName();
	}).drawing;
	to = transition_obj.getTo()? drawings.findElement(function(element) {
		return element.obj instanceof State && element.obj.getStateName() == transition_obj.getTo().getStateName();
	}).drawing : UI.Statemachine.getMousePos();
	line = undefined;

	var bb1 = from.getBBox(),
		bb2 = to.getBBox(),
		p = [{x: bb1.x + bb1.width / 2, y: bb1.y - 1},
		{x: bb1.x + bb1.width / 2, y: bb1.y + bb1.height + 1},
		{x: bb1.x - 1, y: bb1.y + bb1.height / 2},
		{x: bb1.x + bb1.width + 1, y: bb1.y + bb1.height / 2},
		{x: bb2.x + bb2.width / 2, y: bb2.y - 1},
		{x: bb2.x + bb2.width / 2, y: bb2.y + bb2.height + 1},
		{x: bb2.x - 1, y: bb2.y + bb2.height / 2},
		{x: bb2.x + bb2.width + 1, y: bb2.y + bb2.height / 2}],
		d = {}, dis = [];

	for (var i = 0; i < 4; i++) {
		for (var j = 4; j < 8; j++) {
			var dx = Math.abs(p[i].x - p[j].x),
				dy = Math.abs(p[i].y - p[j].y);
			if ((i == j - 4) || (((i != 3 && j != 6) || p[i].x < p[j].x) && ((i != 2 && j != 7) || p[i].x > p[j].x) && ((i != 0 && j != 5) || p[i].y > p[j].y) && ((i != 1 && j != 4) || p[i].y < p[j].y))) {
				dis.push(dx + dy);
				d[dis[dis.length - 1]] = [i, j];
			}
		}
	}
	if (dis.length == 0) {
		var res = [0, 4];
	} else {
		res = d[Math.min.apply(Math, dis)];
	}
	var x1 = p[res[0]].x,
		y1 = p[res[0]].y,
		x4 = p[res[1]].x,
		y4 = p[res[1]].y;
	if (to == from) x1 += 30;
	dx = Math.max(Math.abs(x1 - x4) / 2, 10);
	dy = Math.max(Math.abs(y1 - y4) / 2, 10);

	var keep_ends = UI.Settings.isTransitionModeCentered()
		|| UI.Settings.isTransitionModeCombined() && (Math.abs(x1 - x4) < 2 || Math.abs(y1 - y4) < 2);

	if (!keep_ends) {
		if (transition_obj.getFrom().getStateName() != "INIT") {
			if (res[0] <= 1) { // vertical
				x1 += (bb1.width / 2) / paper.width * (p[res[1]].x - paper.width / 2);
			} else {
				y1 += (bb1.height / 2) / paper.height * (p[res[1]].y - paper.height / 2);
			}
		}
		if (res[1] <= 5) { // vertical
			x4 += (bb2.width / 2) / paper.width * (p[res[0]].x - paper.width / 2);
		} else {
			y4 += (bb2.height / 2) / paper.height * (p[res[0]].y - paper.height / 2);
		}
	}

	var x2 = [x1, x1, x1 - dx, x1 + dx][res[0]].toFixed(3),
		y2 = [y1 - dy, y1 + dy, y1, y1][res[0]].toFixed(3),
		x3 = [0, 0, 0, 0, x4, x4, x4 - dx, x4 + dx][res[1]].toFixed(3),
		y3 = [0, 0, 0, 0, y1 + dy, y1 - dy, y4, y4][res[1]].toFixed(3);
	var path;
	if (path_type == Drawable.Transition.PATH_CURVE) {
		path = ["M", x1.toFixed(3), y1.toFixed(3), "C", x2, y2, x3, y3, x4.toFixed(3), y4.toFixed(3)].join(",");
	} else {
		path = ["M", x1.toFixed(3), y1.toFixed(3), "L", x4.toFixed(3), y4.toFixed(3)].join(",");
	}

	var line = paper.path(path)
		.attr({stroke: color, fill: "none", 'arrow-end': 'classic-wide-long', 'stroke-width': highlight? 4 : 2});
	if (!readonly) line
		.attr({'cursor': 'pointer'})
		.data("transition", transition_obj)
		.click(resetTransition);
	if (outline) line
		.toBack();

	var set_obj = paper.set();
	set_obj.push(line);
	var text_set = paper.set();
	var bbox = line.getBBox();

	if (transition_obj.getOutcome() && !outline) {

		var text_obj = paper.text(bbox.x + bbox.width / 2 + 10, bbox.y + bbox.height / 2, transition_obj.getOutcome())
			.attr({'font-size': 10, stroke: 'none', 'font-family': 'Arial,Helvetica,sans-serif', 'font-weight': 400, fill: text_color});
		if (!readonly) text_obj
			.attr({'cursor': 'pointer'})
			.data("transition", transition_obj)
			.click(resetTransition);

		var textbb = text_obj.getBBox();
		var text_bg = paper.ellipse(textbb.x - 7 + (textbb.width + 14) / 2,
									textbb.y - 3 + (textbb.height + 6) / 2,
									(textbb.width + 14) / 2,
									(textbb.height + 6) / 2)
			.attr({'fill': 'rgba(100%, 100%, 100%, 80%)', 'stroke': color});

		text_obj.toFront();
		text_set.push(text_obj);
		text_set.push(text_bg);
	}
	text_set.attr();
	set_obj.push(text_set);

	this.drawing = set_obj;
	this.obj = transition_obj;

	var merge_server = that;
	var merge_clients = [];

	this.merge = function(other) {
		var target = (that.obj.getAutonomy() <  other.obj.getAutonomy())? that.getMergeServer() : other.getMergeServer();
		var remove = (that.obj.getAutonomy() >= other.obj.getAutonomy())? that.getMergeServer() : other.getMergeServer();
		
		if (target == remove) return;

		var remove_shift_height = remove.calcShiftHeight();
		var target_shift_height = target.calcShiftHeight();

		remove.drawing[0].attr({opacity: 0}).unclick();
		target.drawing[1].translate(0, - remove_shift_height);
		target.getMergeClients().forEach(function(c) { c.drawing[1].translate(0, - remove_shift_height); });

		remove.setMergeServer(target);
		var new_clients = remove.getMergeClients();
		new_clients.push(remove);
		new_clients.forEach(function(c) {
			c.drawing[1].translate(0, target_shift_height);
			if (c.drawing[1][1] != undefined) {
				c.drawing[1][1].toFront();
				c.drawing[1][0].toFront();
			}
		});
		target.concatMergeClients(new_clients);
		remove.resetMergeClients();
	}

	this.getMergeServer = function() {
		return merge_server;
	}
	this.setMergeServer = function(new_server) {
		merge_server = new_server;
	}

	this.getMergeClients = function() {
		return merge_clients;
	}
	this.concatMergeClients = function(new_clients) {
		merge_clients = merge_clients.concat(new_clients);
	}
	this.resetMergeClients = function() {
		merge_clients = [];
	}

	this.calcShiftHeight = function() {
		return (that.drawing[1].getBBox().height + merge_clients.reduce(function(h, c) {
			return h + c.drawing[1].getBBox().height;
		}, 0)) / 2 + 1;
	}

};

Drawable.Transition.PATH_CURVE = 0;
Drawable.Transition.PATH_STRAIGHT = 1;