<script lang="ts">
	import { SerialDevice } from '$lib/devices/serial-device';
	import Device from '$lib/device.svelte';
	import Button from '$lib/button.svelte';
	import { fireflyPid, fireflyVid, type Firefly } from '$lib';
	import { slide } from 'svelte/transition';
	import { SerialFirefly } from './devices/serial-firefly';
	let {
		devices,
		addFirefly
	}: { devices: Firefly[]; addFirefly: (port: Firefly) => void } = $props();
</script>

<div
	class="w-100 flex flex-col
	bg-gray-900 z-50 border-l-1 border-gray-950 p-4 overflow-y-auto"
>
	<h2 class="text-teal-200 text-2xl mb-5">
		<i class="fa-brands fa-usb p-1"></i>
		Attached Fireflies
	</h2>

	<div transition:slide>
		{#each devices as device}
			<Device {device} />
		{/each}
	</div>

	<div transition:slide class="w-full">
		<Button
			extraClasses="w-full"
			onclick={() => {
				if (navigator.serial) {
					navigator.serial
						.requestPort({ filters: [{ usbVendorId: fireflyVid, usbProductId: fireflyPid }] })
						.then((port) => {
							addFirefly(new SerialFirefly(new SerialDevice(port)));
						})
						.catch((error) => {
							console.error('Error selecting port:', error);
						});
				} else {
					alert('Web Serial API is not supported in this browser.');
				}
			}}><i class="fa-solid fa-plus"></i> Add</Button
		>
	</div>
</div>
