var exec = require('child_process').exec; 
var express =require('express'); 
var app = express(); 
app.use('/static', express.static('static')); 
app.use(express.static(__dirname + '/public'));
app.get('/', function(req, res, next) {
    res.status = 200;
	res.sendfile(__dirname + '/index.html');
});
app.get('/api/sum/:a/:b', function(req, res, next) {
    var a = +req.params.a;
    var b = +req.params.b;
    var sum = a + b;
    // console.log('Sum of ' + a + ' and ' + b + ' eq ' + sum);
    exec('ls -la', function(err, stdout, stderr) {
        //console.log('result of "ls -la" is:');
        //console.log(stdout);
    });
    res.status = 200;
    res.end(JSON.stringify({
        status: 'success',
        result: sum,
    }));
});
var dataRes = "";
app.get('/api/acc', function(req, res, next) {
	
	exec('/var/www/html/print.out', function(err, stdout, stderr) {
        //console.log('/var/www/html/print.out');
        //console.log(stdout);
		dataRes = stdout;
    });
	
	var tagsList = dataRes.split(';');
	var err = 0;
	if (tagsList.length<3) err = -1;
	
	
    res.status = 200;
    res.end(JSON.stringify({
        status: 'success',
        result0: tagsList[0],//Math.random(),
        result1: tagsList[1],//Math.random(),
        result2: tagsList[2],//Math.random(),
    }));
});
var dataRes2 = '{"status":"error"}';
app.get('/api/datasensor', function(req, res, next) {
	
    exec('/home/pi/Project/flyLinuxServer/client/client', function(err, stdout, stderr) {
		dataRes2 = stdout;
    });
	
    res.status = 200;
    res.end(dataRes2);
});
app.get('/api/random', function(req, res, next) {
    res.status = 200;
    res.end(JSON.stringify({
        status: 'success',
        result: Math.random(),
    }));
});
app.listen(4444); console.log('App is listening on port 4444');
