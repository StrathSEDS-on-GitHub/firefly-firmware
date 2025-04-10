const KEYS = {
    "id": "u64",
    "rf_freq": "u64",
    "cr": "u64",
    "sf": "u64",
    "bw": "u64",
    "power": "u64"
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

navigator.serial.addEventListener("connect", (e) => {
    console.log(e);
});

navigator.serial.addEventListener("disconnect", (e) => {
    console.log(e);
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

    let pong = await transact(PORT, "ping");
    console.log(pong);
    if (!pong.startsWith("pong")) {
        document.write("Did not receive pong");
    }

});

document.querySelector("#reconfigure").addEventListener("click", async () => {
    for (let key in KEYS) {
        console.log(key);
        let value = document.querySelector(`#${key}`).value;
        if (key === "cr") {
            value = value.split("/")[1];
        }
        let command = `config,${key},${value}`;
        console.log(command);
        let response = await transact(PORT, command);
        if (!response.startsWith("ok")) {
            console.log(response);
            document.write("Did not receive ok");
            break;
        }
    }
})

document.querySelector("#logs").addEventListener("click", async () => {
    let logs = await transact(PORT, "logs");
    console.log(logs);
})