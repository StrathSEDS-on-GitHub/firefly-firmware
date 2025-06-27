<script lang="ts">
	import ffultra from '../assets/ffultra.png';
	import Button from '$lib/button.svelte';
	import { fireflyPid, fireflyVid } from '$lib';
	let { addPort } = $props();
</script>

<div class="absolute top-0 left-0 w-full h-full flex items-center justify-center bg-gray-900 z-50">
	<div class="p-8 rounded-xl border-teal-300 border-2 flex flex-col items-center justify-center">
		<h2 class="text-xl text-teal-300">No Fireflies detected.</h2>
		<img src={ffultra} alt="Firefly patch" class="max-w-50 max-h-50" />
		<p class="text-gray-200">Please connect a Firefly and press authorize.</p>

		<Button
			onclick={() => {
				if (navigator.serial) {
					navigator.serial
						.requestPort({ filters: [{ usbVendorId: fireflyVid, usbProductId: fireflyPid }] })
						.then((port) => {
							addPort(port);
						})
						.catch((error) => {
							console.error('Error selecting port:', error);
						});
				} else {
					alert('Web Serial API is not supported in this browser.');
				}
			}}
			extraClasses="px-4 py-2 mt-4"
		>
			Authorize
		</Button>
	</div>
</div>
