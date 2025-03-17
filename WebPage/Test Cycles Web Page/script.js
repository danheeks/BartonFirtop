let cycleInput = document.getElementById("cycleInput");
let waitTime = document.getElementById("waitTime");
let cycleDone = document.getElementById("cycleDone");
let status = document.getElementById("status");
let startBtn = document.getElementById("startBtn");
let stopBtn = document.getElementById("stopBtn");

let ws = new WebSocket("ws://" + window.location.host + "/ws");

ws.onmessage = function(event) {
    let data = JSON.parse(event.data);
    cycleDone.value = data.cycleDone;
    status.value = data.status;
};

startBtn.addEventListener("click", () => {
    let cycles = parseInt(cycleInput.value) || 0;
    if (cycles > 0) {
        ws.send("start:" + cycles);
        status.value = "Starting...";
    }
});

stopBtn.addEventListener("click", () => {
    ws.send("stop");
    status.value = "Stopping...";
});

waitTime.addEventListener("change", () => {
    let waitValue = parseInt(waitTime.value) || 1;
    ws.send("waitTime:" + waitValue);
});
