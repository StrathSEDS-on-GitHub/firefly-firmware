<script lang="ts">
	import type { Firefly, Line } from '$lib';
	import { fly, slide } from 'svelte/transition';
	import type { SerialDevice } from './devices/serial-device';
	import type { SerialFirefly } from './devices/serial-firefly';

	const MAX_LINES = 1000;

	let { firefly }: { firefly: SerialFirefly } = $props();
	let lines = $state<Line[]>([]);
	let textBuffer: HTMLDivElement;
	firefly.device.addLineHandler((line: Line) => {
		lines = [...lines, line];
		if (lines.length > MAX_LINES) {
			lines = lines.slice(lines.length - MAX_LINES / 2);
		}

		if (textBuffer.scrollHeight - textBuffer.scrollTop <= textBuffer.clientHeight + 10) {
			setTimeout(() => {
				textBuffer.scroll({
					top: textBuffer.scrollHeight,
					behavior: 'instant'
				});
			});
		}
	});
	let deviceInfo = $state('Loading...');
	// FIXME: weird race condition
	setTimeout(async () => {
		while (true) {
			try {
				const info = await firefly.getInfo();
				deviceInfo = `${info.hardware} (${info.role})`;
				break;
			} catch (e) {
				console.error('Error fetching device info:', e);
				await new Promise((resolve) => setTimeout(resolve, 1000));
			}
		}
	}, 50);
</script>

<div class="w-full h-full flex flex-col font-mono">
	<div class="w-full h-10 bg-gray-300 text-teal-500 p-2">{deviceInfo}</div>
	<div class="scrollbar flex flex-col grow bg-gray-200 overflow-y-scroll" bind:this={textBuffer}>
		{#each lines as line}
			<div
				class={`text-gray-900 break-words flex items-center p-2 ${line.kind === 'tx' ? 'bg-gray-100/25' : 'bg-gray-100/75'}`}
			>
				<i
					class={`mr-4 text-xs fas ${line.kind === 'tx' ? 'fa-right-from-bracket text-orange-300' : 'fa-right-to-bracket text-teal-500'}`}
				></i>
				{line.content}
			</div>
		{/each}
	</div>

	<input
		name="line"
		class="w-full h-10 bg-gray-100 text-gray-900 p-2"
		placeholder="Enter command..."
		onkeydown={(ev) => {
			if (ev.key === 'Enter') {
				firefly.device.sendCommand(ev.currentTarget.value);
				ev.currentTarget.value = '';
				ev.preventDefault();
			}
		}}
	/>
</div>
