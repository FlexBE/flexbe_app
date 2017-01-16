UI.Panels.SelectBehavior = new (function() {
	var that = this;

	var be_list = [];
	var be_list_displayed = [];
	var selection_callback = undefined;
	var caret_position = 0;

	var filterBehaviorList = function(filter_exp) {
		if (filter_exp == "") {
			displayBehaviors(be_list, true);
			updateTags();
			return;
		}

		var tag_update_args = {tag: undefined, modifier: undefined};
		var be_list_tagged = be_list.clone();
		filter_exp = filter_exp.replace(/(?:^|\s)([+-])([^\s]*)(?=$|\s)/gi, function(match, modifier, tag) {
			if (tag != "") {
				var need = modifier == "+";
				var relevant = false;
				be_list_tagged_clone = be_list_tagged.clone().filter(function(b) {
					var has_tag = WS.Behaviorlib.getByName(b).getBehaviorTagList().contains(tag);
					relevant = relevant || has_tag;
					return need == has_tag;
				});
				if (relevant) {
					be_list_tagged = be_list_tagged_clone;
				} else {
					tag_update_args = {tag: tag, modifier: modifier};
				}
			}
			return "";
		}).trim();

		var begin_list = be_list_tagged.filter(function(element) {
			return element.toLowerCase().indexOf(filter_exp) == 0;
		});
		var contain_list = be_list_tagged.filter(function(element) {
			return element.toLowerCase().indexOf(filter_exp) > 0;
		});

		displayBehaviors(begin_list, true);
		displayBehaviors(contain_list, false);

		updateTags(tag_update_args.tag, tag_update_args.modifier);
	}

	var updateTags = function(tag, modifier) {
		var tag_list = [];
		var filter_exp = document.getElementById("input_behavior_filter").value;
		be_list_displayed.forEach(function(b) {
			WS.Behaviorlib.getByName(b).getBehaviorTagList().forEach(function(t) {
				if (t == "" || filter_exp.indexOf("+"+t) != -1 || filter_exp.indexOf("-"+t) != -1) return;
				var entry = tag_list.findElement(function(e) { return e.tag == t; })
				if (entry != undefined) {
					entry.count++;
				} else {
					tag_list.push({tag: t, count: 1});
				}
			});
		});

		var sorted_tags = [];
		if (tag != undefined) {
			sorted_tags = sorted_tags.concat(tag_list.filter(function(element) {
				return element.tag.toLowerCase().indexOf(tag) == 0;
			}));
			sorted_tags = sorted_tags.concat(tag_list.filter(function(element) {
				return element.tag.toLowerCase().indexOf(tag) > 0;
			}));
			sorted_tags = sorted_tags.concat(tag_list.filter(function(element) {
				return element.tag.toLowerCase().indexOf(tag) < 0;
			}));
		} else {
			sorted_tags = tag_list.sort(function(a, b) { return b.count - a.count; });
		}

		var tag_panel = document.getElementById("behavior_tag_filter");
		tag_panel.innerHTML = "";
		var tag_width = 0;
		for (var i = 0; i < sorted_tags.length; i++) {
			var t = sorted_tags[i].tag;
			var element = document.createElement("div");
			element.setAttribute("class", "tag");
			element.setAttribute("title", sorted_tags[i].count + " behavior" + ((sorted_tags[i].count != 1)? "s" : ""));
			element.innerText = t;
			element.addEventListener("click", function() {
				if (tag != undefined) {
					var txt = document.getElementById("input_behavior_filter").value;
					txt = txt.replace(modifier + tag, modifier + this.innerText);
					document.getElementById("input_behavior_filter").value = txt;
					document.getElementById("input_behavior_filter").focus();
					var idx = txt.indexOf(modifier + this.innerText);
					document.getElementById("input_behavior_filter").setSelectionRange(idx, idx + (modifier + this.innerText).length + 1);
					//updateTags();
				} else {
					var txt = document.getElementById("input_behavior_filter").value;
					var added = ((txt == "")? "" : " ") + "+" + this.innerText;
					document.getElementById("input_behavior_filter").value += added;
					document.getElementById("input_behavior_filter").focus();
					var idx = (txt + added).indexOf(added);
					document.getElementById("input_behavior_filter").setSelectionRange(idx, idx + added.length + 1);
				}
				that.behaviorFilterChanged();
			});
			tag_panel.appendChild(element);
			if (tag_width + element.clientWidth + 4 + 30 < tag_panel.clientWidth) {
				tag_width += element.clientWidth + 4;
			} else {
				tag_panel.removeChild(tag_panel.lastChild);
				var dots = document.createElement("i");
				dots.innerText = " ...";
				var hidden = sorted_tags.length - i;
				dots.setAttribute("title", hidden + " more tag" + ((hidden == 1)? "" : "s"));
				tag_panel.appendChild(dots);
				break;
			}
		}
		if (tag_panel.children.length == 0) {
			tag_panel.innerHTML = "<i>no tags</i>";
		}
	}

	var displayBehaviors = function(behaviors, clear) {
		var panel = document.getElementById('panel_select_behavior_selection');
		if (clear) {
			panel.innerHTML = "";
			be_list_displayed = [];
		}

		behaviors.forEach(function(b) {
			var m = WS.Behaviorlib.getByName(b).getBehaviorManifest();
			be_list_displayed.push(b);

			behavior_div = document.createElement("div");
			behavior_div.setAttribute("class", "panel_select_behavior_selection_behavior");
			behavior_div.innerHTML =
				  '<b>' + m.name + '</b><br />'
				+ '<i>' + m.description + '</i>';

			behavior_div.addEventListener('click', function() {
				selection_callback(m);
				that.hide();
			});

			panel.appendChild(behavior_div);
		});
	}


	this.setSelectionCallback = function(callback) {
		selection_callback = callback;
	}

	this.show = function() {
		if (selection_callback == undefined) {
			T.debugWarn("No behavior selection callback set!");
		}
		UI.Panels.setActivePanel(UI.Panels.SELECT_BEHAVIOR_PANEL);
		be_list = WS.Behaviorlib.getBehaviorList();
		displayBehaviors(be_list, true);
		updateTags();
	}

	this.hide = function() {
		UI.Panels.hidePanelIfActive(UI.Panels.SELECT_BEHAVIOR_PANEL);
		selection_callback = undefined;
		document.getElementById("input_behavior_filter").value = "";
		document.activeElement.blur();
	}

	this.behaviorFilterChanged = function() {
		filterBehaviorList(document.getElementById("input_behavior_filter").value.toLowerCase());
		caret_position = document.getElementById("input_behavior_filter").selectionStart;
	}

	this.behaviorSelectCancelClicked = function() {
		that.hide();
	}

}) ();