function myFunction() {
    var btn = document.createElement('input');
    btn.setAttribute('type', 'button');
    btn.setAttribute('value', 'BUTTON CLICKED');
    btn.onclick = sf;
    document.body.appendChild(btn);
};