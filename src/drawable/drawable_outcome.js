Drawable.Outcome = function(outcome_obj, target_paper, readonly, outline) {
	var that = this;

	var paper = target_paper;

	var color = outline? '#999' : '#000';

	var dot = paper.set();
	var dot_box = paper.rect(0, 0, 35, 20).attr({opacity: 0});
	if (outcome_obj.getContainer().isConcurrent()) {
		dot.push(paper.text(8, 8, '&').attr({fill: color}));
	} else {
		dot.push(paper.circle(8, 8, 5).attr({fill: color, stroke: color}));
	}
	var outer_dot = paper.circle(8, 8, 8).attr({'fill-opacity': 0, fill: '#FFF', stroke: color});
	if (!readonly) outer_dot
		.data("state", outcome_obj)
		.click(Drawable.Helper.connectTransition);

	if(!readonly && UI.Statemachine.isConnecting()) outer_dot.attr({'cursor': 'pointer'});

	dot.push(dot_box);
	dot.push(outer_dot);
	dot.push(paper.text(8, 25, outcome_obj.getStateName().split('#')[0]).attr({fill: color}));

	if (!readonly) {
		var drag_box = paper.image('img/move-icon.png', 20, 0, 15, 15)
			.attr({cursor: 'move', 'stroke-width': 1})
			.data("state", outcome_obj)
			.data("box", dot_box)
			.drag(Drawable.Helper.moveFnc, Drawable.Helper.startFnc, Drawable.Helper.endFnc);
		dot.push(drag_box);
	}

	dot.translate(outcome_obj.getPosition().x, outcome_obj.getPosition().y);

	this.drawing = dot;
	this.obj = outcome_obj;

	if (!readonly)
		Drawable.Helper.initialIntersectCheck(dot, outcome_obj);
};