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
	//leaflet css
	import { circle, map, marker, tileLayer } from 'leaflet';
	import { onMount } from 'svelte';
	import { TestSerial } from '$lib/devices/test-serial';

	let { data }: PageProps = $props();

	let devices: Firefly[] = $state([]);
	let canvas: HTMLCanvasElement | null = $state(null);
	let canvasContainer: HTMLDivElement | null = $state(null);
	let files: string[] = $state([]);

	let leafletMap: HTMLDivElement | null = $state(null);
	let mapContainer: HTMLDivElement | null = $state(null);

	navigator.storage.getDirectory().then(async (root) => {
		const entries = root.values();
		for await (const entry of entries) {
			files.push(entry.name);
		}
	});

	(async () => {
		let x = await getSerialPorts();
		if ('error' in x) {
			console.error(x.message);
			alert(`Error: ${x.message}`);
		} else {
			x.map((it) => addFirefly(new SerialFirefly(new SerialDevice(it))));
		}
	})();

	const render = () => {
		if (canvas) {
			if (canvas.width != canvasContainer?.clientWidth) {
				canvas.width = canvasContainer?.clientWidth || 0;
				canvas.height = canvasContainer?.clientHeight || 0;
			}
			data.power('myplot');
		}

		if (leafletMap) {
			leafletMap.style.width = (mapContainer?.clientWidth || '0') + 'px';
			leafletMap.style.height = (mapContainer?.clientHeight || '0') + 'px';
		}
		requestAnimationFrame(render);
	};
	requestAnimationFrame(render);

	const addFirefly = (firefly: Firefly) => {
		if (firefly instanceof SerialFirefly) {
			console.log(`Adding Firefly device: ${firefly.device.port}`);
			let dev = firefly.device;
			let port = dev.port;
			port.addEventListener('disconnect', () => {
				console.log(`Port ${port} disconnected`);
				devices = devices.filter((device) => {
					return (
						device !== firefly &&
						(device instanceof SerialFirefly || (device as RemoteFirefly).connection !== firefly)
					);
				});
				console.log(`Devices after disconnect: ${devices.length}`);
			});
		}
		devices.push(firefly);
	};

	let addMarker: (coord: [number, number], color: string) => void = $state(() => {});

	setTimeout(() => {
		var m = map('map').setView([55.434, -5.687], 14);
		console.log(m);
		tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
			maxZoom: 19
		}).addTo(m);
		addMarker = ([lat, long]: [number, number], color: string) => {
			circle([lat, long], { color, radius: 1 }).addTo(m);
		};
	}, 300);
</script>

{#if devices.length > 0}
	<div class="flex">
		<main class="h-[100vh] grow flex overflow-hidden">
			{#each devices as device}
				{#if device instanceof SerialFirefly}
					<Terminal firefly={device} />
				{/if}
			{/each}
			<div class="flex flex-col w-full">
				<div bind:this={canvasContainer} class="w-full h-1/2">
					<canvas id="myplot" bind:this={canvas}></canvas>
				</div>
				<div class="w-full h-1/2" bind:this={mapContainer}>
					<div id="map" bind:this={leafletMap}></div>
				</div>
			</div>
		</main>
		<div class="flex flex-col justify-between">
			<Devices {devices} parse={data.parse} {addFirefly} {addMarker} />
			<div class="p-4 flex flex-col max-h-100 w-110">
				<h2 class="text-teal-500 text-2xl mb-5">
					<i class="fa-solid fa-folder p-1"></i>
					Previous logs
				</h2>
				
				<ul class="flex flex-col gap-1 max-h-full overflow-y-scroll">
					{#each files as file_name}
						<!-- svelte-ignore a11y_click_events_have_key_events -->
						<!-- svelte-ignore a11y_no_static_element_interactions -->
						<!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
						<li
							class="bg-gray-100 p-2 text-sm text-teal-900 cursor-pointer"
							onclick={(evt) => {
								navigator.storage.getDirectory().then(async (root) => {
									const fileHandle = await root.getFileHandle(file_name);
									const file = await fileHandle.getFile();

									const fileURL = URL.createObjectURL(file);
									const downloadLink = document.createElement('a');
									downloadLink.href = fileURL;
									downloadLink.download = file_name + '.log';
									document.body.appendChild(downloadLink);
									downloadLink.click();
								});
							}}
						>
							<i class="fa-solid fa-file"></i>
							{file_name}
						</li>
					{/each}
				</ul>
			</div>
		</div>
	</div>
{:else}
	<UsbPopup addPort={addFirefly} />
{/if}
