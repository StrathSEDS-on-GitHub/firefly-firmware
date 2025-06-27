<script lang="ts">
	import { SerialDevice } from '$lib/devices/serial-device';
	import Device from '$lib/device.svelte';
	import Button from '$lib/button.svelte';
	import { fireflyPid, fireflyVid, type Firefly } from '$lib';
	import { slide } from 'svelte/transition';
	import { SerialFirefly } from './devices/serial-firefly';
	import { RemoteFirefly } from './devices/remote-firefly';
	let {
		devices,
		addFirefly,
		parse
	}: {
		devices: Firefly[];
		addFirefly: (port: Firefly) => void;
		parse: (line: string) => Map<String, any>;
	} = $props();
</script>

<div
	class="w-110 flex flex-col
	bg-gray-900 z-50 border-l-1 border-gray-950 p-4 overflow-y-auto"
>
	<h2 class="text-teal-200 text-2xl mb-5">
		<i class="fa-brands fa-usb p-1"></i>
		Attached Fireflies
	</h2>

	<div transition:slide>
		{#each devices as device}
			{#if device instanceof SerialFirefly}
				<Device {addFirefly} {parse} {device} />
			{/if}
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

	<hr class="my-4 border-teal-300" />

	<h2 class="text-teal-200 text-2xl mb-5">
		<i class="fa-solid fa-tower-broadcast p-1"></i>
		Remote Fireflies
	</h2>

	<div transition:slide>
		{#if devices.filter((d) => d instanceof RemoteFirefly).length === 0}
			<p class="text-gray-400">No remote Fireflies found. Check configuration.</p>
		{:else}
			{#each devices as device}
				{#if device instanceof RemoteFirefly}
					<Device {addFirefly} {parse} {device} />
				{/if}
			{/each}
		{/if}
	</div>
</div>
