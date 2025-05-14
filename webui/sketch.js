var cols, rows;
var scl = 20;
var w = 1800;
var h = 700;

var flying = 0;

var terrain = [];
var canvas;


var layer;
var rocket;

function setup() {
    frameRate(120);
    canvas = createCanvas(200, 400, WEBGL);
    let container = document.getElementById("p5-container");
    canvas.parent(container);
    new ResizeObserver(() => {
        let w = container.clientWidth;
        let h = container.clientHeight;
        resizeCanvas(w, h);
    }).observe(container);

    rocket = loadModel("art.stl", );

    cols = w / scl;
    rows = h / scl;

    for (var x = 0; x < cols; x++) {
        terrain[x] = [];
        for (var y = 0; y < rows; y++) {
            terrain[x][y] = 0; //specify a default value for now
        }
    }
    var yoff = 0;
    for (var y = 0; y < rows; y++) {
        var xoff = 0;
        for (var x = 0; x < cols; x++) {
            terrain[x][y] = map(noise(xoff, yoff), 0, 1, -100, 100);
            xoff += 0.2;
        }
        yoff += 0.2;
    }

}

var PORT;
async function transact(port, command) {
    const encoder = new TextEncoder();
    if (!port.writable) {
        console.error("port not writable");
        return {
            error: "port not writable"
        };
    }
    const writer = port.writable.getWriter();
    try {
        await writer.write(encoder.encode(command + "\r\n"));
    } catch (error) {
        console.error(error);
        return {
            error: error.toString()
        }
    } finally {
        writer.releaseLock();
    }

    let response = "";
    while (port.readable) {
        const reader = port.readable.getReader();
        try {
            while (true) {
                const { value, done } = await reader.read();
                let decoded = new TextDecoder().decode(value);
                response += decoded;
                if (done || decoded.endsWith("\n")) {
                    break;
                }
            }
        } catch (error) {
            console.error(error);
            return {
                error: error.toString()
            }
        } finally {
            console.log("reader lock released");
            reader.releaseLock();
        }
        return response;
    }
}

function quatToAxisAngle(q) {
    // Normalize quaternion if needed
    let mag = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= mag;
    q.x /= mag;
    q.y /= mag;
    q.z /= mag;

    let angle = 2 * Math.acos(q.w);
    let s = Math.sqrt(1 - q.w * q.w);

    let x = q.x, y = q.y, z = q.z;

    // Avoid divide by zero
    if (s > 0.001) {
        x = q.x / s;
        y = q.y / s;
        z = q.z / s;
    }

    return { axis: createVector(z, y, x), angle: angle };
}

var quats = [{ w: 1, x: 0, y: 0, z: 0 }];
const ctx = document.getElementById('myChart');
let chart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Altitude',
            data: [],
            fill: false,
            borderColor: 'rgb(75, 192, 192)',
            tension: 0.1
        }]
    },
    options: {
        plugins: {
            legend: {
                display: false
            },
        }
    }
});

async function parseLines(data) {
    let lines = data.split("\n");
    for (let line of lines.slice(0, -1)) {
        if (line.trim() === "") continue;
        let parts = line.split(",");
        if (parts.length < 2) continue;
        let key = parts[0];
        if (key === "bno") {
            let state = parts[1].split("(")[0];
            let mv = parts[2].slice(0, -1);
            document.querySelector("#status").innerText = state;
            document.querySelector("#pyro-mv").innerText = mv + "mV";
            let data = parts.slice(3).join(",");
            quats.push(...eval(data));
        } else if (key === "pressure") {
            let prs = eval(parts.slice(1, 9).join(","));
            let tmps = eval(parts.slice(9).join(","));
            
            let alts = prs.map((p, i) => {
                let t = tmps[i];
                let a = 44330 * (1 - Math.pow(p / 102100, 0.1903));
                return a;
            }
            );
            let avg = alts.reduce((a, b) => a + b) / alts.length;
            altGauge.setValue(avg);
            chart.data.labels.push("");
            chart.data.datasets[0].data.push(avg);

            if (chart.data.labels.length > 50) {
                chart.data.labels = chart.data.labels.slice(1);
                chart.data.datasets[0].data = chart.data.datasets[0].data.slice(1);
            }
            
            chart.update();
        } else if (key === "bmi323") {
            let [xacc, yacc, zacc] = eval(parts.slice(1,4).join(","));
            let total = Math.sqrt(xacc * xacc + yacc * yacc + zacc * zacc);
            accGauge.setValue(total);


            let [x, y, z] = eval(parts.slice(4).join(","));
            let dt = 0.0001;
            let dQ = {x: x * dt, y: y * dt, z: z * dt, w: 1};
            let last = quats[quats.length - 1];
            let q = multiplyQuat(last, dQ);
            quats.push(q);
        }
    }
    return lines[lines.length - 1];
}

function multiplyQuat(q1, q2) {
    let { x: x1, y: y1, z: z1, w: w1 } = q1;
    let { x: x2, y: y2, z: z2, w: w2 } = q2;
    return {
        x: x1 * w2 + w1 * x2 + y1 * z2 - z1 * y2,
        y: y1 * w2 + w1 * y2 + z1 * x2 - x1 * z2,
        z: z1 * w2 + w1 * z2 + x1 * y2 - y1 * x2,
        w: w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    }
        
}

async function readTask(port) {
    const reader = port.readable.getReader({ mode: "byob" });
    let response = "";
    // Listen to data coming from the serial device.
    while (true) {
        const { value, done } = await reader.read(new Uint8Array(102));
        let decoded = new TextDecoder().decode(value);
        response += decoded;
        response = await parseLines(response);
        if (done) {
            // Allow the serial port to be closed later.
            reader.releaseLock();
            break;
        }
    }
}

document.querySelector("#arm").addEventListener("click", async () => {
    await transact(PORT, "arm,avionics");
});
document.querySelector("#disarm").addEventListener("click", async () => {
    await transact(PORT, "disarm,avionics");
});
document.querySelector("#fire").addEventListener("click", async () => {
    await transact(PORT, "fire,avionics,1,1");
});

document.querySelector("#request-access").addEventListener("click", async () => {
    await navigator.serial.requestPort();
    const ports = await navigator.serial.getPorts();
    if (ports.length === 0) {
        console.log("No serial ports available.");
    } else {
        await ports[0].open({ baudRate: 115200 });
        PORT = ports[0];
    }

    // let pong = await transact(PORT, "ping");
    // console.log(pong);
    await readTask(PORT);
});

function draw() {
    background(38, 40, 56);
    if (!layer) {
        layer = createGraphics(w, h, WEBGL);

        layer.translate(0, 150, 0);
        layer.rotateX(PI / 3);
        layer.fill(28, 40, 56);
        layer.translate(-w / 2, -h / 2);
        for (var y = 0; y < rows - 1; y++) {
            layer.beginShape(TRIANGLE_STRIP);
            layer.stroke(25, 31, 16 + y * 10);
            for (var x = 0; x < cols; x++) {
                layer.vertex(x * scl, y * scl, terrain[x][y]);
                layer.vertex(x * scl, (y + 1) * scl, terrain[x][y + 1]);
            }
            layer.endShape();
        }
    }

    translate(0, 0, -100);
    image(layer, -w / 2, -h / 2);
    translate(0, 0, 100);

    // Torus
    push();
    rotateWithFrameCount();
    fill(185, 20, 77);
    // torus(100, 20);
    scale(2);
    lights();
    model(rocket);
    pop();
}

// Rotate 1 degree per frame along all three axes
function rotateWithFrameCount() {
    if (quats.length > 0) {
        let q = quats[0];
        if (quats.length > 1) {
            quats = quats.slice(1);
        } else {
            console.log("underfull");
        }
        let { axis, angle } = quatToAxisAngle(q);
        if (!isNaN(angle)) {
            rotate(angle, axis);
        }
    }
    if (quats.length > 60) {
        quats = quats.slice(1);
        console.log("overfull");
    }
}

var map = L.map('map', {
    center: [55.8616893, -4.246423],
    zoom: 17,
    attributionControl: false,
    zoomControl: false,
});
var Stadia_AlidadeSmoothDark = L.tileLayer('https://tiles.stadiamaps.com/tiles/alidade_smooth_dark/{z}/{x}/{y}{r}.{ext}', {
    minZoom: 0,
    maxZoom: 20,
    attribution: '&copy; <a href="https://www.stadiamaps.com/" target="_blank">Stadia Maps</a> &copy; <a href="https://openmaptiles.org/" target="_blank">OpenMapTiles</a> &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
    ext: 'png'
});
map.addLayer(Stadia_AlidadeSmoothDark);

var marker = L.marker([55.8616893, -4.246423]).addTo(map);


var Gauge = window.Gauge;

// var cpuGauge = Gauge(document.getElementById("cpuSpeed"), {
//     max: 100,
//     // custom label renderer
//     label: function(value) {
//       return Math.round(value) + " m/s"
//     },
//     value: 50,
// });
var accGauge = Gauge(document.getElementById("acc"), {
    max: 100,
    // custom label renderer
    label: function(value) {
      return Math.round(value) + " m/s²"
    },
    value: 0,
});
var altGauge = Gauge(document.getElementById("alt"), {
    max: 5000,
    // custom label renderer
    label: function (value) {
        return Math.round(value) + "m"
    },
    value: 0,
});
