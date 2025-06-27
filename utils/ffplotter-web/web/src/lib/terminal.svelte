<script lang="ts">
	import type { Firefly, Line } from '$lib';
	import { fly, slide } from 'svelte/transition';
	import type { SerialDevice } from './devices/serial-device';
	import type { SerialFirefly } from './devices/serial-firefly';

	const MAX_LINES = 100;

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
		const info = await firefly.getInfo();
		deviceInfo = `${info.hardware} (${info.role})`;
	}, 50);
</script>

<div class="w-full h-full flex flex-col font-mono">
	<div class="w-full h-10 bg-gray-700 text-teal-300 p-2">{deviceInfo}</div>
	<div class="scrollbar flex flex-col grow bg-gray-800 overflow-y-scroll" bind:this={textBuffer}>
		{#each lines as line}
			<div
				class={`text-gray-200 break-words flex items-center p-2 ${line.kind === 'tx' ? 'bg-gray-900/25' : 'bg-gray-900/75'}`}
			>
				<i
					class={`mr-4 text-xs fas ${line.kind === 'tx' ? 'fa-right-from-bracket text-orange-300' : 'fa-right-to-bracket text-teal-300'}`}
				></i>
				{line.content}
			</div>
		{/each}
	</div>

	<input
		name="line"
		class="w-full h-10 bg-gray-900 text-gray-100 p-2"
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
