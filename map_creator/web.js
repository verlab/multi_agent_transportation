// Config
var express = require("express");
var logfmt = require("logfmt");
var jsonfile = require("jsonfile");
var app = express();
var port = 5000;

app.use(express.static(__dirname + "/public"));
app.use(express.urlencoded());
app.use(logfmt.requestLogger());

// Request
app.get("/", function(req, res) {
  res.render("./public/index.html");
});

app.post("/save", function(req, res) {
  var data = req.body;

  var file_name = data['file_name'];
  delete data['file_name'];

  file_name = "./maps_aerial/{0}.json".replace('{0}', file_name)

  jsonfile.writeFile(file_name, data, function(err) {});

  res.send(200);
});

// Server
app.listen(port, function() {
  console.log("Server on " + port);
});
