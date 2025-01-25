function updateValvePosition(x, y) {
    const valve = document.getElementById("valve");
    valve.setAttribute("x", x);
    valve.setAttribute("y", y);
}

function sendRequest(action) {
    fetch('/action', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: action })
    })
    .then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Error:', error));
}
