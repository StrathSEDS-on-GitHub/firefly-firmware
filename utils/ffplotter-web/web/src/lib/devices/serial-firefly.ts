import type { DeviceConfig, DeviceInfo, Error, Firefly, Role } from "$lib";
import type { SerialDevice } from "./serial-device";

export class SerialFirefly implements Firefly {
    device: SerialDevice;

    constructor(device: SerialDevice) {
        this.device = device;
    }

    async setConfig(config: DeviceConfig): Promise<void> {
        for (const [key, value] of Object.entries(config)) {
            if (typeof value !== "number") {
                return Promise.reject(new Error(`Invalid config value for ${key}: ${value}`));
            }
            const reqId = Math.floor(Math.random() * 10000);
            const command = `config,${key},${value},${reqId}`;
            await this.device.sendCommand(command);
            await new Promise<void>((resolve, reject) => {
                this.device.replyHandlers.set(reqId, (data: string) => {
                    if (data.startsWith("ok")) {
                        resolve();
                    } else {
                        console.error(`Failed to set config ${key}: ${data}`);
                        reject(new Error(`Failed to set config ${key}: ${data}`));
                    }
                });
            });
        }
    }

    async getConfig(): Promise<DeviceConfig> {
        const reqId = Math.floor(Math.random() * 10000);
        await this.device.sendCommand(`dumpconfig,${reqId}`);

        return new Promise((resolve, reject) => {
            this.device.replyHandlers.set(reqId, (data: string) => {
                const parts = data.split(",");
                let config: any = {};
                for (let part of parts) {
                    const [key, value] = part.split("=");
                    if (key && value) {
                        Object.assign(config, { [key.trim()]: value.trim() });
                    }
                }

                if (Object.keys(config).length !== 6) {
                    console.error("Invalid device config format:", data);
                    return reject(new Error("Invalid device config format"));
                }
                resolve({
                    cr: parseFloat(config["cr"]),
                    sf: parseFloat(config["sf"]),
                    rf_freq: parseFloat(config["rf_freq"]),
                    bw: parseFloat(config["bw"]),
                    power: parseFloat(config["power"]),
                    id: parseFloat(config["id"])
                });
            });
        });
    }

    async getInfo(): Promise<DeviceInfo> {
        const reqId = Math.floor(Math.random() * 10000);
        await this.device.sendCommand(`info,${reqId}`);

        return new Promise((resolve, reject) => {
            this.device.replyHandlers.set(reqId, (data: string) => {
                const parts = data.split(",");
                if (parts.length !== 3) {
                    console.error("Invalid device info format:", data);
                    return reject(new Error("Invalid device info format"));
                }
                const [hardware, firmware, role] = parts;
                let info = {
                    hardware, firmware, role: (role as Role)
                };
                resolve(info);
            });
        });

    }

}