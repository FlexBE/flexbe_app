Drawable.Note = function(note_obj, paper) {
	var that = this;

	var note = paper.set();

	var ox = 0, oy = 0, lx = 0, ly = 0;

	var moveFnc = function(dx, dy, x, y, evt) {
		lx = dx + ox;
		ly = dy + oy;
		lx = Math.min(Math.max(lx, 0), UI.Statemachine.getR().width - this.getBBox().width);
		ly = Math.min(Math.max(ly, 0), UI.Statemachine.getR().height - this.getBBox().height);
		note.attr({x: lx, y: ly});
	}
	var startFnc = function() {
		lx = note_obj.getPosition().x;
		ly = note_obj.getPosition().y;
		ox = note_obj.getPosition().x;
		oy = note_obj.getPosition().y;
	}
	var endFnc = function(evt) {
		note_obj.setPosition({x: lx, y: ly});
	}

	var wrapText = function(text, width) {
		var words = text.split(" ");
		var wrapped = "";
		var t = paper.text(100, 100).attr('text-anchor', 'start');

		for (var i=0; i<words.length; i++) {
			t.attr("text", wrapped + " " + words[i]);
			if (t.getBBox().width > width) {
				wrapped += "\n" + words[i];
			} else {
				wrapped += " " + words[i];
			}
		}

		t.remove();
		return wrapped;
	}

	this.editNote = function() {
		var editor = document.getElementById("note_editor"),
			text_input = document.getElementById("input_note_editor_text"),
			delete_btn = document.getElementById("button_note_editor_delete"),
			save_btn = document.getElementById("button_note_editor_save"),
			important_cb = document.getElementById("cb_note_editor_important");
		var color = note_obj.isImportant()? '#600' : '#000';

		editor.style.display = "block";
		editor.style.backgroundColor = "#ccc";
		editor.style.borderLeft = "3px solid " + color;
		text_input.style.color = color
		text_input.value = note_obj.getContent();
		important_cb.checked = note_obj.isImportant();

		text_input.focus();

		var hide = function() {
			editor.style.display = "none";
			save_btn.removeEventListener('click', saveCB);
			important_cb.removeEventListener('change', importantCB);
			delete_btn.removeEventListener('click', deleteCB);
		}
		var saveCB = function() {
			if (text_input.value == "") {
				Behavior.removeCommentNote(note_obj);
			} else {
				note_obj.setContent(text_input.value);
			}
			hide();
			UI.Statemachine.refreshView();
		};
		var importantCB = function(evt) {
			note_obj.setImportant(evt.target.checked);
			color = note_obj.isImportant()? '#600' : '#000';
			editor.style.borderLeft = "3px solid " + color;
			text_input.style.color = color
			txt.attr({'fill': color});
			line.attr({'fill': color});
		};
		var deleteCB = function() {
			Behavior.removeCommentNote(note_obj);
			hide();
			UI.Statemachine.refreshView();
		}

		save_btn.addEventListener('click', saveCB);
		important_cb.addEventListener('change', importantCB);
		delete_btn.addEventListener('click', deleteCB);
	}

	var color = note_obj.isImportant()? '#600' : '#000';

	var bg = paper.rect(0, 0, 213, 0)
		.attr({'fill': 'rgba(0, 0, 0, .1)', 'stroke-opacity': 0});
	var txt = paper.text(10, 5, wrapText(note_obj.getContent(), 200))
		.attr({"text-anchor": 'start', 'fill': color, 'font-size': '10', 'font-family': 'monospace'});

	bg.attr({'height': txt.getBBox().height + 10, 'width': txt.getBBox().width + 15});

	var line = paper.rect(0, 0, 3, bg.getBBox().height)
		.attr({'stroke-opacity': 0, 'fill': color});

	note.push(bg);
	note.push(txt);
	note.push(line);

	txt.translate(10, 5 + txt.getBBox().height / 2);

	note.attr({x: note_obj.getPosition().x, y: note_obj.getPosition().y})
		.drag(moveFnc, startFnc, endFnc)
		.dblclick(that.editNote);

	this.drawing = note;
	this.obj = note_obj;

};