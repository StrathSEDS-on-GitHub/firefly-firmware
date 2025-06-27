// place files you want to import through the `$lib` alias in this folder.

export type Error = {
    error: true,
    message: string;
}

export type Role = "GroundMain" | "Cansat" | "Avionics";
export const fireflyVid = 0x16c0;
export const fireflyPid = 0x27dd;


export type DeviceInfo = {
    hardware: string;
    firmware: string;
    role: Role;
}

export type Line = {
    kind: "tx" | "rx";
    content: string;
}

export type DeviceConfig = {
    cr: number;
    sf: number;
    rf_freq: number;
    bw: number;
    power: number;
    id: number;
}

export async function getSerialPorts(): Promise<Error | SerialPort[]> {
    if (!("serial" in navigator)) {
        return Promise.reject(new Error('Web Serial API is not supported in this browser.'));
    }
    return navigator.serial.getPorts();
}

export interface Firefly {
    getConfig(): Promise<DeviceConfig>;
    setConfig(config: DeviceConfig): Promise<void>;
    getInfo(): Promise<DeviceInfo>;
}