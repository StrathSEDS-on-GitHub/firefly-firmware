import type { DeviceConfig, DeviceInfo, Firefly } from "$lib";

export class RemoteFirefly implements Firefly {
    getConfig(): Promise<DeviceConfig> {
        throw new Error("Method not implemented.");
    }
    setConfig(config: DeviceConfig): Promise<void> {
        throw new Error("Method not implemented.");
    }
    getInfo(): Promise<DeviceInfo> {
        throw new Error("Method not implemented.");
    }

}