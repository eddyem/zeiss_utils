Foc = function(){
// const REQ_PATH=window.location.hostname+":4444/";
const REQ_PATH="http://dasha.sao.ru:4444/";
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
// show message blocking all
function blockMsg(txt, bgcolor){
    $("shadow").style.display = "block";
    if(!bgcolor) bgcolor = "red";
    $("shadow").innerHTML = txt.replace("\n", "<br>");
}
// parse answer for status request
function chkStatus(req){
    var msg = req.responseText;
    console.log("Get status message: " + msg);
    if(msg == "OK"){
        $("shadow").innerHTML = "";
        $("shadow").style.display = "none";
    }else blockMsg(msg);
};
// parse answer for command requests
function chkCmd(req){
    var msg = req.responseText;
    console.log("Get cmd answer: " + msg);
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
    console.log(req.responseText);
    curVal = Number(req.responseText);
    if(first){
        $('focSet').value = curVal;
        first = false;
    }
    $('curFval').innerHTML = curVal;
    //$('focSlider').value = curVal;
}
function getdata(){
    clearTimeout(timeout_upd);
    sendrequest("focus", chF);
    sendrequest("status", chkStatus);
    timeout_upd = setTimeout(getdata, 1000);
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
    getdata();
}
// send new focus value
function SetFocus(){
    var set = $('focSet').value;
    if(set < minVal || set > maxVal){
        alert("Wrong focus value");
        return;
    }
    console.log("Set focus: " + set);
    sendrequest("goto="+set, chkCmd);
}
function Move(dir){
    console.log("Move to " + ((dir > 0) ? "+" : "-") + " with speed " + curSpeed);
    var targspeeds = [ 130, 400, 800, 1200 ];
    var cmd = "targspeed=" + ((dir > 0) ? "" : "-") + targspeeds[curSpeed-1];
    sendrequest(cmd, chkCmd);
    console.log("send request " + cmd);
}
function Stop(){
    sendrequest("stop", chkCmd);
    console.log("Stop");
}
// slider or input field changed
function change(val){
    if(val < minVal) val = minVal;
    else if(val > maxVal) val = maxVal;
    //$('focSlider').value = val;
    $('focSet').value = val;
    console.log("Chfocval: " + val);
}
function chSpd(val){
    if(val < 1) val = 1;
    else if(val > 4) val = 4;
    $('speed').value = val;
    curSpeed = val;
    console.log("Chspd: " + val);
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
