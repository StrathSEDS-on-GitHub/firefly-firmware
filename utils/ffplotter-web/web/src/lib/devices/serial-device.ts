import type { DeviceConfig, DeviceInfo, Line, Role } from "$lib";

export class SerialDevice {
    port: SerialPort;
    portReady: boolean = false;
    lineHandlers: ((line: Line) => void)[] = [];
    replyHandlers: Map<number, (data: string) => void> = new Map();

    constructor(port: SerialPort) {
        port.open({ baudRate: 115200 }).then(() => this.portReady = true).catch((error) => {
            console.error("Failed to open serial port:", error);
            throw new Error("Failed to open serial port: " + error.message);
        });
        this.port = port;
        this.readTask();
    }

    addLineHandler(addLine: (line: Line) => void) {
        this.lineHandlers.push(addLine);
    }

    async readTask(): Promise<void> {
        while (true) {
            await this.waitForPortReady();
            if (!this.port.readable) {
                console.error("Port is not readable");
                continue;
            }
            let response = "";
            const reader = this.port.readable.getReader();
            const decoder = new TextDecoder();
            try {
                while (true) {
                    const { value, done } = await reader.read();
                    let decoded = decoder.decode(value);
                    response += decoded;
                    if (done || decoded.endsWith("\n")) {
                        break;
                    }
                }
            } catch (error) {
                throw new Error("Error reading from serial port: " + error);
            } finally {
                reader.releaseLock();
            }

            for (let line of response.split("\n")) {
                line = line.trim();
                if (line.length === 0) continue;

                this.lineHandlers.forEach(it => it({ kind: "rx", content: line }));

                if (line.startsWith("[REPLY#")) {
                    let number = parseInt(line.substring(7, line.indexOf("]")));
                    let data = line.substring(line.indexOf("]") + 1).trim();
                    if (this.replyHandlers.has(number)) {
                        this.replyHandlers.get(number)!(data);
                    } else {
                        console.warn(`No handler for reply number ${number}`);
                    }
                }
            }
        }
    }

    async waitForPortReady(): Promise<void> {
        if (this.portReady) {
            return Promise.resolve();
        }
        return new Promise((resolve, _reject) => {
            const checkReady = () => {
                if (this.portReady) {
                    resolve();
                } else {
                    setTimeout(checkReady, 100);
                }
            };
            checkReady();
        });
    }

    async sendCommand(command: string): Promise<void> {
        await this.waitForPortReady();
        if (!this.port.writable) {
            console.error("port not writable");
            return Promise.reject(new Error("Port is not writable"));
        }
        const writer = this.port.writable.getWriter();
        const encoder = new TextEncoder();
        console.log(command);
        this.lineHandlers.forEach(it => it({ kind: "tx", content: command }));

        try {
            await writer.write(encoder.encode(command));
        } catch (error) {
            console.error(error);
            return Promise.reject(new Error("Failed to write to serial port: " + error));
        } finally {
            writer.releaseLock();
        }
    }


}
