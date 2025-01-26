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

// Example of fetching position updates from the ESP32
setInterval(() => {
    fetch('/valve-position')
        .then(response => response.json())
        .then(data => {
            // Update the valve position with new coordinates
            updateValvePosition(data.x, data.y);
        })
        .catch(error => console.error('Error fetching valve position:', error));
}, 100); // Fetch every 0.1 second
