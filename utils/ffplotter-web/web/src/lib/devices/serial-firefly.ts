import type { DeviceConfig, DeviceInfo, Error, Firefly, Role } from "$lib";
import type { SerialDevice, SerialDeviceInterface } from "./serial-device";

export class SerialFirefly implements Firefly {
    device: SerialDeviceInterface;
    info: DeviceInfo | null = null;

    constructor(device: SerialDeviceInterface) {
        this.device = device;
    }

    arm(): void {
        this.device.sendCommand("uarm");
    }
    disarm(): void {
        this.device.sendCommand("udisarm");
    }
    testFire1(): void {
        this.device.sendCommand("ufire,1");
    }
    testFire2(): void {
        this.device.sendCommand("ufire,2");
    }

    async setConfig(config: DeviceConfig): Promise<void> {
        for (const [key, value] of Object.entries(config)) {
            if (typeof value !== "number") {
                return Promise.reject(new Error(`Invalid config value for ${key}: ${value}`));
            }
            const reqId = Math.floor(Math.random() * 10000);
            const command = `config,${key},${value},${reqId}`;
            let resp = await this.device.transactRetry(reqId, command, 100);
            if (!resp.startsWith("ok")) {
                throw new Error(`Failed to set config ${key}: ${resp}`);
            }
        }
    }

    async getConfig(): Promise<DeviceConfig> {
        const reqId = Math.floor(Math.random() * 10000);
        let data = await this.device.transactRetry(reqId, `dumpconfig,${reqId}`, 100);

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
            // throw new Error("Invalid device config format");
            return {
                cr: 0,
                sf: 0,
                rf_freq: 0,
                bw: 0,
                power: 0,
                id: 0
            }
        }
        return ({
            cr: parseFloat(config["cr"]),
            sf: parseFloat(config["sf"]),
            rf_freq: parseFloat(config["rf_freq"]),
            bw: parseFloat(config["bw"]),
            power: parseFloat(config["power"]),
            id: parseFloat(config["id"])
        });
    }

    async getInfo(): Promise<DeviceInfo> {
        if (this.info) {
            return Promise.resolve(this.info);
        }

        const reqId = Math.floor(Math.random() * 10000);
        let data = await this.device.transactRetry(reqId, `info,${reqId}`, 1000);
        let parts = data.split(",");
        if (parts.length !== 3) {
            console.error("Invalid device info format:", data);
            // throw new Error("Invalid device info format");
            parts = ["unknown", "unknown", "Cansat"];
        }
        const [hardware, firmware, role] = parts;
        let info = {
            hardware, firmware, role: (role as Role)
        };
        this.info = info;
        return info;

    }

    isRemote(): boolean {
        return false;
    }

}