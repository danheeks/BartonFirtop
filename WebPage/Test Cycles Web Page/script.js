let cycleInput = document.getElementById("cycleInput");
let cycleDone = document.getElementById("cycleDone");
let status = document.getElementById("status");
let startBtn = document.getElementById("startBtn");
let stopBtn = document.getElementById("stopBtn");

let cycleCount = 0;
let maxCycles = 0;
let interval;

startBtn.addEventListener("click", () => {
    maxCycles = parseInt(cycleInput.value) || 0;
    cycleCount = 0;
    cycleDone.value = cycleCount;
    status.value = "Running...";
    
    if (maxCycles > 0) {
        interval = setInterval(() => {
            if (cycleCount < maxCycles) {
                cycleCount++;
                cycleDone.value = cycleCount;
            } else {
                clearInterval(interval);
                status.value = "Completed";
            }
        }, 1000); // Simulating 1 cycle per second
    }
});

stopBtn.addEventListener("click", () => {
    clearInterval(interval);
    status.value = "Stopped";
});

function updateData(status, cycles_done) {
    const cycleDone = document.getElementById("cycleDone");
    cycleDone.setText(cycles_done);
    const txtStatus = document.getElementById("status");
    txtStatus.setText(status);
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
            updateData(data.status, data.cycles_done);
        })
        .catch(error => console.error('Error fetching valve position:', error));
}, 100); // Fetch every 0.1 second
