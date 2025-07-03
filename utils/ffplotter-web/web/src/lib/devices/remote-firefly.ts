import type { DeviceConfig, DeviceInfo, Firefly, Role } from "$lib";
import type { SerialFirefly } from "./serial-firefly";

export class RemoteFirefly implements Firefly {
    role: Role;
    connection: SerialFirefly;
    deviceInfo: DeviceInfo | null = null;

    constructor(role: Role, connection: SerialFirefly) {
        this.role = role;
        this.connection = connection;
    }
    arm(): void {
        this.connection.device.sendCommand(`arm,${this.role}`);
    }
    disarm(): void {
        this.connection.device.sendCommand(`disarm,${this.role}`);
    }
    testFire1(): void {
        this.connection.device.sendCommand(`fire,${this.role},1,100`);
    }
    testFire2(): void {
        this.connection.device.sendCommand(`fire,${this.role},2,100`);
    }

    getConfig(): Promise<DeviceConfig> {
        throw new Error("Method not implemented.");
    }
    setConfig(config: DeviceConfig): Promise<void> {
        throw new Error("Method not implemented.");
    }

    getInfo(): Promise<DeviceInfo> {
        if (this.deviceInfo) {
            return Promise.resolve(this.deviceInfo);
        }
        let reqId = Math.floor(Math.random() * 10000);
        this.connection.device.sendCommand(`remote-info,${this.role},${reqId}`);

        console.log("Requesting remote info for role:", this.role, "with ID:", reqId);

        return Promise.resolve({
            hardware: "unknown",
            firmware: "unknown",
            role: this.role
        });

        return new Promise((resolve, reject) => {
            setTimeout(() => {
                if (!this.connection.device.replyHandlers.has(reqId)) {
                    this.connection.device.replyHandlers.delete(reqId);
                    reject(new Error("Remote info request timed out"));
                }
            }, 1000);
            this.connection.device.replyHandlers.set(reqId, (data: string) => {
                this.connection.device.replyHandlers.delete(reqId);

                const parts = data.split(",");
                if (parts.length < 3) {
                    console.error("Invalid remote info format:", data);
                    return reject(new Error("Invalid remote info format"));
                }
                const hardware = parts[0].trim();
                const firmware = parts[1].trim();
                const role = parts[2].trim() as Role;

                let info = { hardware, firmware, role };
                this.deviceInfo = info;
                resolve(info);
            });
        });
    }

    isRemote(): boolean {
        return true;
    }

}