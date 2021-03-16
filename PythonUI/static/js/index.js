function requestNewAngle() {
    var request = new XMLHttpRequest();
    var result = document.getElementById('newAngle');
    request.onreadystatechange = function() {
        if(this.readyState == 4 && this.status == 200) {
            result.innerHTML = this.responseText;
        } else {
            result.innerHTML = "There was an error in the request"
        }
    };
    request.open('POST', '/sendAngle', true)
    request.setRequestHeader('content-type', 'application/x-www-form-urlencoded;charset=UTF-8')
    request.send("angle=" + document.getElementById('angle').value);
};

function requestHomingAngle() {
    var request = new XMLHttpRequest();
    var result = document.getElementById('Homing')
    request.onreadystatechange = function() {
        if(this.readyState == 4 && this.status == 200) {
            result.innerHTML = this.responseText;
        } else {
            result.innerHTML = "There was an error in requesting the Homing Angle"
        }
    };
    alert("Homing button was pressed. Homing now!")
    request.open('POST', '/HomingAnglePage', true)
    request.setRequestHeader('content-type', 'application/x-www-form-urlencoded;charset=UTF-8')
    request.send("Homing=" + document.getElementById('HomingAngle').value);
}

function requestMovingUp() {
    var request = new XMLHttpRequest();
    var result = document.getElementById('MovingUp')
    request.onreadystatechange = function() {
        if(this.readyState == 4 && this.status == 200) {
            result.innerHTML = this.responseText;
        } else {
            result.innerHTML = "There was an error in requesting the Moving Up button"
        }
    };
    alert("Moving Up button was pressed. Moving up now!")
    request.open('POST', '/MovingUpPage', true)
    request.setRequestHeader('content-type', 'application/x-www-form-urlencoded;charset=UTF-8')
    request.send("MovingUp=" + document.getElementById('MoveUp').value);
}