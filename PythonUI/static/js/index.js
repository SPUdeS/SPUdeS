const pi = 3.14159265359

function requestDisplacementAngles() {
    let request = new XMLHttpRequest();
    let positions = [
        parseFloat(document.getElementById("position_x").value),
        parseFloat(document.getElementById("position_y").value),
        parseFloat(document.getElementById("position_z").value),
        (pi/180)*parseFloat(document.getElementById("position_a").value),
        (pi/180)*parseFloat(document.getElementById("position_b").value),
        (pi/180)*parseFloat(document.getElementById("position_c").value)
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

function requestShowoff() {
    let request = new XMLHttpRequest()
    request.open('POST', '/NewShowoffRequest', true)
    request.setRequestHeader('content-type', 'application/json')
}

function updateCameraNumber() {
    let request = new XMLHttpRequest()
    let data = {cameraNumber: document.getElementById("cameraNumber").value}
    request.open('POST', '/UpdateCamera', true)
    request.setRequestHeader('content-type', 'application/json')
    request.send(JSON.stringify(data));
}
