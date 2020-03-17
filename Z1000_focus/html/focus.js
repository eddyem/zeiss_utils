Foc = function(){
// const REQ_PATH=window.location.hostname+":4444/";
const REQ_PATH="http://ztcs.sao.ru:4444/";
const DEBUG = false; // set to true for debug
var targspeeds = [ 220, 500, 800, 1200 ]; // four target speeds
var minVal=0.01, maxVal=76.5, curVal = 3.0, curSpeed = 1;
var timeout_upd, timeout_msg;
// ID
function $(id){ return document.getElementById(id); }
// Request: req_SRT - request string, _onOK - function to run if all OK
function sendrequest(req_STR, _onOK){
    var request = new XMLHttpRequest(), timeout_id;
    request.open("POST", REQ_PATH + req_STR, true);
    request.onreadystatechange=function(){
        if (request.readyState == 4){
            if (request.status == 200){
                clearTimeout(timeout_id);
                if(_onOK) _onOK(request);
            }
            else{
                clearTimeout(timeout_id);
                blockMsg("Request Error");
            }
        }
    }
    request.send(req_STR);
    timeout_id = setTimeout(function(){blockMsg("Request timeout");}, 3000);
}
function Log(x){
    if(DEBUG) console.log(x);
}
// show message blocking all
function blockMsg(txt, bgcolor){
    $("shadow").style.display = "block";
    if(!bgcolor) bgcolor = "gray";
    $("shadow").style.backgroundColor = bgcolor;
    $("shadow").innerHTML = txt.replace("\n", "<br>");
}
function btnsStat(moving){
    var color = moving ? "red" : "green";
    $("Fstop").disabled = !moving;
    $("Fstop").style.backgroundColor = moving ? "green" : "red";
    $("Fset").disabled = moving;
    $("Fset").style.backgroundColor = color;
    $("F-").disabled = moving;
    $("F-").style.backgroundColor = color;
    $("F+").disabled = moving;
    $("F+").style.backgroundColor = color;
}
// parse answer for status request
function chkStatus(req){
    var msg = req.responseText;
    Log("Get status message: " + msg);
    if(msg == "OK" || msg == "moving"){
        $("shadow").innerHTML = "";
        $("shadow").style.display = "none";
        btnsStat(msg == "moving");
    }else blockMsg(msg);
};
// parse answer for command requests
function chkCmd(req){
    var msg = req.responseText;
    Log("Get cmd answer: " + msg);
    if(msg != "OK") alert(msg);
}
    /*
function ch_status(txt, bgcolor){
    function rmmsg(){clearTimeout(timeout_msg);document.body.removeChild($("_msg_div"));}
    clearTimeout(timeout_msg);
    if(!bgcolor) bgcolor = "red";
    if(!$("_msg_div")){ // добавляем блок для вывода сообщений
        var div = document.createElement('div'), s = div.style;
        div.id = "_msg_div";
        div.class = "btm";
        document.body.appendChild(div);
    };
    $("_msg_div").style.backgroundColor = bgcolor;
    $("_msg_div").innerHTML = txt.replace("\n", "<br>");
    $("_msg_div").addEventListener("click", rmmsg, true)
    timeout_msg = setTimeout(rmmsg, 5000);
}*/
var first = true;
function chF(req){
    Log(req.responseText);
    curVal = Number(req.responseText);
    if(first){
        $('focSet').value = curVal;
        first = false;
    }
    $('curFval').innerHTML = Number(Math.round(curVal*100.)/100.).toFixed(2);
    //$('focSlider').value = curVal;
}
function getdata(){
    clearTimeout(timeout_upd);
    sendrequest("focus", chF);
    sendrequest("status", chkStatus);
    timeout_upd = setTimeout(getdata, 1000);
}
// set limits
function getLimits(req){
    var lims = {};
    req.responseText.split('\n').forEach(function(value){
        var keypair = value.split('=');
        lims[keypair[0]] = keypair[1];
    });
    if(lims["focmin"]){ // focmin=2.75
        $('focSet').min = minVal = Number(lims["focmin"]);
        Log("focmin: " + minVal);
    }
    if(lims["focmax"]){ // focmax=76
        $('focSet').max = maxVal = Number(lims["focmax"]);
        Log("focmax: " + maxVal);
    }
    if(lims["maxspeed"] && lims["minspeed"]){
        var minspd = Number(lims["minspeed"]); // minspeed=350
        var maxspd = Number(lims["maxspeed"]); // maxspeed=1200
        var w = (maxspd - minspd)/3;
        for(i = 0; i < 4; ++i){
            targspeeds[i] = Math.round(minspd + w*i);
            Log("targspeeds["+i+"]="+targspeeds[i]);
        }
    }
}
// init all things
function FirstRun(){
    blockMsg("init", "black");
    // first we should get all params
    chSpd($('speed').value);
    var F = $('focSet');
    F.value = curVal;
    F.min = minVal;
    F.max = maxVal;
    sendrequest("limits", getLimits);
    getdata();
}
// send new focus value
function SetFocus(){
    var set = $('focSet').value;
    if(set < minVal || set > maxVal){
        alert("Wrong focus value");
        return;
    }
    Log("Set focus: " + set);
    sendrequest("goto="+set, chkCmd);
}
function Move(dir){
    Log("Move to " + ((dir > 0) ? "+" : "-") + " with speed " + curSpeed);

    var cmd = "targspeed=" + ((dir > 0) ? "" : "-") + targspeeds[curSpeed-1];
    sendrequest(cmd, chkCmd);
    Log("send request " + cmd);
}
function Stop(){
    sendrequest("stop", chkCmd);
    Log("Stop");
}
// slider or input field changed
function change(val){
    if(val < minVal) val = minVal;
    else if(val > maxVal) val = maxVal;
    //$('focSlider').value = val;
    $('focSet').value = val;
    Log("Chfocval: " + val);
}
function chSpd(val){
    if(val < 1) val = 1;
    else if(val > 4) val = 4;
    $('speed').value = val;
    curSpeed = val;
    Log("Chspd: " + val);
}
// update value in input field by current
function update(){
    $('focSet').value = curVal;
}

return{
    Run: FirstRun,
    SetFocus: SetFocus,
    Move: Move,
    Stop: Stop,
    Ch: change,
    chSpd: chSpd,
    upd: update,
    }
}();
