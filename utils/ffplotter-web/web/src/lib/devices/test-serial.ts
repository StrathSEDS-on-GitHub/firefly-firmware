import type { Line } from "$lib";
import { SerialDevice, type SerialDeviceInterface } from "./serial-device";

export class TestSerial implements SerialDeviceInterface {
    lineHandlers: ((line: Line) => void)[] = [];
    replyHandlers: Map<number, (data: string) => void> = new Map();
    port = null;

    addLineHandler(addLine: (line: Line) => void) {
        this.lineHandlers.push(addLine);
    }
    sendCommand(command: string): Promise<void> {
        this.lineHandlers.forEach(it => it({ kind: "tx", content: command }));
        this.lineHandlers.forEach(it => it({ kind: "rx", content: "response" + command }));
        return Promise.resolve();
    }
    transactRetry(reqId: number, command: string, timeout: number): Promise<string> {
        this.lineHandlers.forEach(it => it({ kind: "tx", content: reqId + command }));
        this.lineHandlers.forEach(it => it({ kind: "rx", content: "response" + reqId +  command }));

        return Promise.resolve("a,x=1,y=2,z=3,q=4,r=5");
    }

}