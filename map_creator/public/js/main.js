$(function() {

  var PROPORTION = 10;

  var content = $("#content");
  var grid = $("#grid");
  var page = new fabric.Canvas("page");
  var line = $("<div class='line'></div>");
  var cell = $("<div class='cell'></div>");

  var input_map_width = $("#input_map_width");
  var input_map_height = $("#input_map_height");
  var input_item_width = $("#input_item_width");
  var input_item_height = $("#input_item_height");

  var input_file_name = $("#filename");

  var button_save = $("#button_save");
  var button_clear = $("#button_clear");
  var button_obstacle = $("#button_obstacle");
  var button_obstacle_remove = $("#button_obstacle_remove");
  var button_object = $("#button_object")
  var button_object_remove = $("#button_object_remove")
  var button_add_land_robot = $("#button_add_land_robot");
  var button_add_aerial_robot = $("#button_add_aerial_robot");
  var button_remove_robot = $("#button_remove_robot");

  var width = content.width();
  var height = content.height();
  var object_size = 20;
  var object_list = [];
  var obstacle_list = [];
  var robot_list = [];
  var robot_type = { land: 0, aerial: 1 };

  var name_list = { 1: 'A', 2: 'B', 3: 'C' }

  var item_config = {
    width: object_size,
    height: object_size,
    hasControls: false,
  };

  var text_config = {
    evented: false,
    selectable: false,
    textAlign: 'left',
    fontSize: 22,
    hasControls: false,
    originX: 'center',
    originY: 'center'
  };

  var obstacle_config = {
    top: 10,
    left: 10,
    width: object_size,
    height: object_size,
    fill: '#832106',
    hasControls: false
  };

  var robot_land_config = {
    top: 40,
    left: 10,
    radius: object_size / 2,
    fill: '#0935BD',
    hasControls: false
  };

  var robot_aerial_config = {
    top: 40,
    left: 10,
    width: object_size,
    height: object_size,
    fill: '#0935bd',
    hasControls: false
  }

  var bound_value = function(value, pattern) {

    var rest = value % pattern;

    if (value < pattern) {
      // return pattern;
      return 0;
    } else if ( rest <= ( pattern / 2 ) ) {
      return value - rest;
    } else {
      return value + ( pattern - rest );
    }
  };

  var getRandomArbitrary = function(min, max) {
    return Math.random() * (max - min) + min;
  };

  var update_size = function() {
    input_map_width.val( (width / PROPORTION) + "m" );
    input_map_height.val( (height / PROPORTION) + "m" );

    grid.width( width + 50 );
    grid.height( height + 50 );

    draw_grid();

    page.setWidth( width );
    page.setHeight( height );
  };

  var init = function() {

    update_size();

    $(window).resize(function() {

      width = content.width();
      height = content.height();

      update_size();
    });
  };

  var config_buttons = function() {

    button_object.on("click", function() {

      var obj_id = object_list.length;

      var start_config = { top: 0, left: 0 };
      var start_config_text = { top: 10, left: 10 }
      var end_config = { top: 0, left: 40 }
      var end_config_text = { top: 20, left: 60 }

      var object = {
        id: obj_id,
        start: new fabric.Rect(
          $.extend({}, item_config, {
            fill: '#008000'
          }, start_config)),
        start_text: new fabric.Text('' + (obj_id + 1),
          $.extend({}, text_config, start_config_text)),
        end: new fabric.Rect(
          $.extend({}, item_config, {
            fill: '#FF0000'
          }, end_config)),
        end_text: new fabric.Text('' + (obj_id + 1),
          $.extend({}, text_config, end_config_text))
      };

      object_list.push( object );

      page.add( object['start'], object['start_text'], object['end'], object['end_text'] )
    });

    button_object_remove.on("click", function() {

      object = object_list.pop();

      object['start'].remove();
      object['start_text'].remove();
      object['end'].remove();
      object['end_text'].remove();
    });

    button_add_land_robot.on("click", function() {

      var robot_id = robot_list.length;
      var robot = new fabric.Circle( $.extend({}, robot_land_config) );

      // var robot_text = new fabric.Text('' + (obj_id + 1), $.extend({}, text_config, start_config_text))

      robot.type = robot_type.land;

      var robot_dict = {
        id: robot_id,
        rob: robot,
        // rob_text: new fabric.Text(name_list[robot_id + 1], text_config),
      };

      robot_list.push( robot_dict );

      // page.add( robot_dict['rob'], robot_dict['rob_text'] );
      page.add( robot_dict['rob'] );
    });

    button_add_aerial_robot.on("click", function() {

      var robot_id = robot_list.length;
      var robot = new fabric.Triangle( $.extend({}, robot_aerial_config) );

      robot.type = robot_type.aerial;

      var robot_dict = {
        id: robot_id,
        rob: robot,
        // rob_text: new fabric.Text(name_list[robot_id + 1], text_config),
      };

      robot_list.push( robot_dict );

      page.add( robot_dict['rob'] );
    });

    button_remove_robot.on("click", function() {

      active_object = page.getActiveObject();

      for (var i = 0; i < robot_list.length; i++) {
        if ( active_object == robot_list[i] ) {
          robot_list.pop(i).remove();
          break;
        }
      }
    });

    button_obstacle.on("click", function() {

      var obstacle = new fabric.Rect( $.extend({}, obstacle_config) );

      obstacle_list.push( obstacle );
      page.add( obstacle );
    });

    button_obstacle_remove.on("click", function() {

      active_object = page.getActiveObject();

      for (var i = 0; i < obstacle_list.length; i++) {
        if ( active_object == obstacle_list[i] ) {
          obstacle_list.pop(i).remove()
          break;
        }
      }
    });

    button_clear.on("click", function() {
      while( obstacle_list.length ) {
        obstacle_list.pop().remove();
      }
    });

    button_save.on("click", function() {

      var map_width = width / 2 / PROPORTION;
      var map_height = height / 2 / PROPORTION;
      var item_depth_prop = 5 / PROPORTION;

      // Obstacles Configurations
      var obstacle_descriptions = []

      for(var i = 0; i < obstacle_list.length; i++) {

        var obstacle = obstacle_list[i];

        var obstacle_x = obstacle.getLeft() / 2 / PROPORTION;
        var obstacle_y = obstacle.getTop() / 2 / PROPORTION;

        var local_width = object_size / 2 / PROPORTION;
        var local_height = object_size / 2 / PROPORTION;

        obstacle_x += local_width / 2;
        obstacle_y += local_width / 2;

        obstacle_descriptions.push({
          x: obstacle_x,
          y: obstacle_y,
          width: local_width,
          height: local_width
        });
      }

      // Robots Configuration
      var robot_descriptions = [];

      for (var i = 0; i < robot_list.length; i++) {
        var robot = robot_list[i]['rob'];

        var robot_x = robot.getLeft() / 2 / PROPORTION;
        var robot_y = robot.getTop() / 2 / PROPORTION;

        if ( robot.type == robot_type.land ) {
          var robot_width = (robot.radius * 2) / PROPORTION;
          var robot_height = (robot.radius * 2) / PROPORTION;
        } else if ( robot.type == robot_type.aerial ) {
          var robot_width = (robot.width) / PROPORTION;
          var robot_height = (robot.height) / PROPORTION;
        }

        var robot_depth = 10 / PROPORTION;

        robot_width = Math.ceil( robot_width ) / 2;
        robot_height = Math.ceil( robot_height ) / 2;

        robot_x += robot_width / 2;
        robot_y += robot_height / 2;

        robot_descriptions.push({
          x: robot_x,
          y: robot_y,
          width: robot_width,
          height: robot_height,
          depth: robot_depth,
          type: robot.type
        });
      };

      var object_descriptions = [];

      for(var i = 0; i < object_list.length; i++) {

        var object = object_list[i];

        var id = object['id'];
        var start = object['start'];
        var end = object['end'];

        var start_x = start.getLeft() / 2 / PROPORTION;
        var start_y = start.getTop() / 2 / PROPORTION;

        var end_x = end.getLeft() / 2 / PROPORTION;
        var end_y = end.getTop() / 2 / PROPORTION;

        var local_width = object_size / 2 / PROPORTION;
        var local_width = object_size / 2 / PROPORTION;

        start_x += local_width / 2;
        start_y += local_width / 2;

        end_x += local_width / 2;
        end_y += local_width / 2;

        object_descriptions.push({
          id: id,
          width: local_width,
          height: local_width,
          depth: item_depth_prop,
          start: {
            x: start_x,
            y: start_y
          },
          end: {
            x: end_x,
            y: end_y
          }
        });
      }

      var file_name = input_file_name.val()

      var data = {
        "size": [map_width, map_height],
        "object_list": object_descriptions,
        "obstacle_list": obstacle_descriptions,
        "robot_list": robot_descriptions,
        "file_name": file_name
      };

      $.post("/save", data, function(data) {
        alert("Map Saved");
      });
    });
  };

  var draw_grid = function() {

    var line = $("<div class='line'></div>");
    var cell = $("<div class='cell'></div>");
    var size = 20;
    var i_max = height / size;
    var j_max = width / size;

    grid.empty();

    for(var i = 1; i <= i_max + 1; i++) {

      var current_line = line.clone();

      for(var j = 1; j <= j_max + 1; j++) {

        var current_cell = cell.clone();

        current_line.append( current_cell );
      }

      grid.append( current_line );
    }
  };

  init();
  config_buttons();
  draw_grid();

  setInterval(function() {

    for (var i = 0; i < object_list.length; i++) {

      var object = object_list[i];

      var start = object['start'];
      var start_text = object['start_text'];
      var end = object['end'];
      var end_text = object['end_text'];

      start.set({
        left: bound_value( start.getLeft(), 20 ),
        top: bound_value( start.getTop(), 20 )
      });

      start_text.set({
        left: start.getLeft() + 10,
        top: start.getTop() + 8
      });

      end.set({
        left: bound_value( end.getLeft(), 20 ),
        top: bound_value( end.getTop(), 20 )
      });

      end_text.set({
        left: end.getLeft() + 10,
        top: end.getTop() + 8
      });
    }

    for(var i = 0; i < obstacle_list.length; i++) {

      var obstacle = obstacle_list[i];

      obstacle.set({
        left: bound_value( obstacle.getLeft() , 20),
        top: bound_value( obstacle.getTop() , 20)
      });
    }

    for (var i = 0; i < robot_list.length; i++) {
      var robot = robot_list[i];

      var rob = robot['rob'];
      // var rob_text = robot['rob_text'];

      rob.set({
        left: bound_value( rob.getLeft() , 20),
        top: bound_value( rob.getTop() , 20)
      });

      // rob_text.set({
      //   left: rob.getLeft() + 10,
      //   top: rob.getTop() + 8
      // });
    };

    page.renderAll();

  }, 2000);
});

