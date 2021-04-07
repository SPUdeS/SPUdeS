function requestDisplacementAngles() {
    let request = new XMLHttpRequest();
    let positions = [
        parseFloat(document.getElementById("position_x").value),
        parseFloat(document.getElementById("position_y").value),
        parseFloat(document.getElementById("position_z").value),
        parseFloat(document.getElementById("position_a").value),
        parseFloat(document.getElementById("position_b").value),
        parseFloat(document.getElementById("position_c").value)
    ]
    let data = {type_: "target", data_: positions}
    request.open('POST', '/NewDisplacementRequest', true)
    request.setRequestHeader('content-type', 'application/json')
    request.send(JSON.stringify(data));
}

function requestSweep(DOF) {
    let request = new XMLHttpRequest()
    let data = {type_: "sweep", data_: DOF}
    request.open('POST', '/NewDisplacementRequest', true)
    request.setRequestHeader('content-type', 'application/json')
    request.send(JSON.stringify(data));
}

function requestInitPlatform() {
    let request = new XMLHttpRequest()
    let data = {type_: "initialization", data_: [0,0,0,0,0,0]}
    request.open('POST', '/NewDisplacementRequest', true)
    request.setRequestHeader('content-type', 'application/json')
    request.send(JSON.stringify(data));
}

function getImagePath() {
    return ("../static/img/plot.png") ?  "../static/img/plot.png"  : "../static/img/plotHome.png"
}