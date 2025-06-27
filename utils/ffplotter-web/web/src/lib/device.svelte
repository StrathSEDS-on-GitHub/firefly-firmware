<script lang="ts">
	import type { DeviceConfig, Firefly } from '$lib';
	import { fade, fly, slide } from 'svelte/transition';
	import Button from './button.svelte';

	let { device }: { device: Firefly } = $props();
	let configExpanded = $state(false);
	let syncedConfig: DeviceConfig | null = $state(null);
	let editableConfig: DeviceConfig | null = $state(null);

	const syncColor = (value: keyof DeviceConfig) => {
		if (!editableConfig || !syncedConfig) {
			return '';
		}

		if (editableConfig[value] === syncedConfig[value]) {
			return 'text-teal-500';
		} else {
			return 'text-yellow-500';
		}
	};
</script>

<div transition:fly={{ y: 100 }}>
	{#await device.getInfo()}
		<div class="p-3 mb-2 bg-gray-800 rounded-lg">
			<h3 class="text-teal-300 text-lg">
				<i class="fas fa-circle-notch fa-spin"></i> Still loading
			</h3>
		</div>
	{:then info}
		<div class="p-3 mb-2 bg-gray-800 rounded-lg">
			<div class="flex justify-between">
				<div class="flex">
					<h3 class="text-teal-300 text-lg">
						<i class="fas fa-plug text-teal-300"></i>
						{info.hardware}
					</h3>
				</div>
				<div class="flex">
					<h3 class="text-gray-300 text-lg font-mono">{info.role}</h3>
				</div>
			</div>
			<p class="text-gray-300 text-sm">Firmware {info.firmware}</p>
			<div
				class={`flex flex-col text-teal-300 border-2 border-teal-500
            rounded-md p-2 mt-5`}
				transition:slide
			>
				<div class="flex justify-between items-center">
					<button
						class="cursor-pointer text-sm transition-all hover:bg-teal-900 w-fit rounded p-1"
						onclick={async () => {
							if (!editableConfig || !syncedConfig) {
								syncedConfig = await device.getConfig();
								editableConfig = { ...syncedConfig };
							}
							configExpanded = !configExpanded;
						}}
					>
						<i
							class={`fas fa-chevron-right mr-1 transition-all ${configExpanded ? 'rotate-90' : ''}`}
						></i>
						Config
					</button>

					{#if configExpanded}
						<div transition:fade>
							<Button
								extraClasses="px-1"
								onclick={async () => {
									syncedConfig = await device.getConfig();
									editableConfig = { ...syncedConfig };
								}}
							>
								<i class="fas fa-refresh text-xs"></i>
							</Button>
						</div>
					{/if}
				</div>
				{#if configExpanded}
					<div transition:slide>
						<hr class="border-teal-500 w-full my-3" />
						{#if editableConfig === null}
							<h3 class="text-teal-300 text-xs p-2">
								<i class="fas fa-circle-notch fa-spin"></i> Still loading
							</h3>
						{:else}
							<div class="grid grid-cols-[10px_1fr_1fr] gap-2">
								<i
									class={`fas fa-dot-circle self-center text-[0.65rem] m-0 transition-colors duration-300 ${syncColor('rf_freq')}`}
								></i>
								<p class="text-sm text-gray-50 self-center">Radio Frequency</p>
								<input
									name="frequency"
									type="number"
									class="w-full text-sm bg-gray-900 p-2 mr-2 rounded-xl"
									bind:value={editableConfig.rf_freq}
								/>
								<i class={`fas fa-dot-circle self-center text-[0.65rem] m-0 ${syncColor('id')}`}
								></i>
								<p class="text-sm text-gray-50 self-center">Board ID</p>
								<input
									name="id"
									type="number"
									class="w-full text-sm bg-gray-900 p-2 mr-2 rounded-xl"
									bind:value={editableConfig.id}
								/>
								<i class={`fas fa-dot-circle self-center text-[0.65rem] m-0 ${syncColor('bw')}`}
								></i>
								<p class="text-sm text-gray-50 self-center">Bandwidth</p>
								<input
									name="bw"
									type="number"
									class="w-full text-sm bg-gray-900 p-2 mr-2 rounded-xl"
									bind:value={editableConfig.bw}
								/>
								<i class={`fas fa-dot-circle self-center text-[0.65rem] m-0 ${syncColor('cr')}`}
								></i>
								<p class="text-sm text-gray-50 self-center">Coding rate</p>
								<input
									name="cr"
									type="number"
									class="w-full text-sm bg-gray-900 p-2 mr-2 rounded-xl"
									bind:value={editableConfig.cr}
								/>
								<i class={`fas fa-dot-circle self-center text-[0.65rem] m-0 ${syncColor('sf')}`}
								></i>
								<p class="text-sm text-gray-50 self-center">Spreading Factor</p>
								<input
									name="sf"
									type="number"
									class="w-full text-sm bg-gray-900 p-2 mr-2 rounded-xl"
									bind:value={editableConfig.sf}
								/>
							</div>
						{/if}

						<div class="flex flex-row-reverse mt-3">
							<Button
								extraClasses="px-2"
								onclick={async () => {
									if (editableConfig) {
										await device.setConfig(editableConfig);
										syncedConfig = await device.getConfig();
										editableConfig = { ...syncedConfig };
									}
								}}
							>
								<i class="fas fa-save mr-1"></i> Save
							</Button>
						</div>
					</div>
				{/if}
			</div>
		</div>
	{/await}
</div>
