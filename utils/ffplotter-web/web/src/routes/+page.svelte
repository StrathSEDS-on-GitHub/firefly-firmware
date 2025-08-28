<script lang="ts">
	import { getSerialPorts, type Firefly, type Role } from '$lib';
	import '@fortawesome/fontawesome-free/css/all.min.css';
	import type { PageProps } from './$types';
	import UsbPopup from '$lib/usbpopup.svelte';
	import Devices from '$lib/devices.svelte';
	import Terminal from '$lib/terminal.svelte';
	import { SerialFirefly } from '$lib/devices/serial-firefly';
	import { SerialDevice } from '$lib/devices/serial-device';
	import { RemoteFirefly } from '$lib/devices/remote-firefly';

	let { data }: PageProps = $props();

	let devices: Firefly[] = $state([]);
	let canvas: HTMLCanvasElement | null = $state(null);
	let canvasContainer: HTMLDivElement | null = $state(null);

	(async () => {
		let x = await getSerialPorts();
		if ('error' in x) {
			console.error(x.message);
			alert(`Error: ${x.message}`);
		} else {
			x.map((it) => addFirefly(new SerialFirefly(new SerialDevice(it))));
		}
	})();

	setInterval(() => {
		if (canvas) {
			if (canvas.width != canvasContainer?.clientWidth) {
				canvas.width = canvasContainer?.clientWidth || 0;
				canvas.height = canvasContainer?.clientHeight || 0;
			}
			data.power('myplot');
		}
	}, 16);

	const addFirefly = (firefly: Firefly) => {
		if (firefly instanceof SerialFirefly) {
			console.log(`Adding Firefly device: ${firefly.device.port}`);
			let dev = firefly.device;
			let port = dev.port;
			port.addEventListener('disconnect', () => {
				console.log(`Port ${port} disconnected`);
				devices = devices.filter((device) => {
					return (
						device !== firefly && (device instanceof SerialFirefly || (device as RemoteFirefly).connection !== firefly)
					);
				});
				console.log(`Devices after disconnect: ${devices.length}`);
			});
		}
		devices.push(firefly);
	};
</script>

{#if devices.length > 0}
	<div class="flex">
		<main class="h-[100vh] grow flex overflow-hidden">
			{#each devices as device}
				{#if device instanceof SerialFirefly}
					<Terminal firefly={device} />
				{/if}
			{/each}
			<div bind:this={canvasContainer} class="w-full">
				<canvas id="myplot" bind:this={canvas}></canvas>
			</div>
		</main>
		<Devices {devices} {addFirefly} />
	</div>
{:else}
	<UsbPopup addPort={addFirefly} />
{/if}
