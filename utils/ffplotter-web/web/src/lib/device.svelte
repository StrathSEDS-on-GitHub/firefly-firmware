<script lang="ts">
	import type { DeviceConfig, DeviceInfo, Firefly, Role } from '$lib';
	import { fade, fly, slide } from 'svelte/transition';
	import Button from './button.svelte';
	import { RemoteFirefly } from './devices/remote-firefly';
	import type { SerialFirefly } from './devices/serial-firefly';

	let {
		device,
		addFirefly,
		parse
	}: {
		device: Firefly;
		addFirefly: (ff: Firefly) => void;
		parse: (line: string) => Map<String, any>;
	} = $props();
	let configExpanded = $state(false);
	let controlsExpanded = $state(false);
	let syncedConfig: DeviceConfig | null = $state(null);
	let editableConfig: DeviceConfig | null = $state(null);

	let deviceInfo: DeviceInfo | null = $state(null);
	let linkInfo: DeviceInfo | null = $state(null);

	let remotes = $state<Role[]>([]);

	const isRemote = device instanceof RemoteFirefly;

	(async () => {
		while (true) {
			try {
				deviceInfo = await device.getInfo();
				break;
			} catch (e) {
				console.error('Error fetching device info:', e);
			}
		}
	})();

	(async () => {
		if (!isRemote) {
			return;
		}
		while (true) {
			try {
				linkInfo = await device.connection.getInfo();
				break;
			} catch (e) {
				console.error('Error fetching device info:', e);
			}
		}
	})();

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

	if (!isRemote) {
		(device as SerialFirefly).device.addLineHandler((line) => {
			try {
				let row: Map<String, any> = parse(line.content);
				if (row && row.get('src')) {
					let src = row.get('src').String;
					if (!remotes.includes(src)) {
						remotes = [...remotes, src];
						addFirefly(new RemoteFirefly(src as Role, device as SerialFirefly));
					}
				}
			} catch (e) {}
		});
	}
</script>

<div transition:fly={{ y: 100 }}>
	{#if !deviceInfo}
		<div class="p-3 mb-2 bg-gray-800 rounded-lg">
			<h3 class="text-teal-300 text-lg">
				<i class="fas fa-circle-notch fa-spin"></i> Still loading
			</h3>
		</div>
	{:else}
		<div class="p-3 mb-2 bg-gray-800 rounded-lg">
			<div class="flex justify-between">
				<div class="flex items-baseline">
					<h3 class="text-teal-300 text-lg">
						<i class={`fas ${isRemote ? 'fa-satellite-dish' : 'fa-plug'} text-teal-300`}></i>
						{deviceInfo.hardware}
					</h3>
					{#if linkInfo}
						<span class="text-gray-400 text-xs ml-1">
							via {linkInfo.role}
						</span>
					{/if}
				</div>
				<div class="flex flex-col">
					<h3 class="text-gray-300 text-lg font-mono">{deviceInfo.role}</h3>

					<div class=""></div>
				</div>
			</div>
			<p class="text-gray-300 text-sm">Firmware {deviceInfo.firmware}</p>

			<div class="border-1 border-red-400 px-2 py-2 mt-2 rounded text-red-300" transition:slide>
				<button
					class="cursor-pointer text-sm transition-all hover:bg-teal-900 w-fit rounded p-1"
					onclick={async () => {
						controlsExpanded = !controlsExpanded;
					}}
				>
					<i
						class={`fas fa-chevron-right mr-1 transition-all ${controlsExpanded ? 'rotate-90' : ''}`}
					></i>
					Controls
				</button>

				{#if controlsExpanded}
					<div class="flex flex-wrap gap-2 justify-center p-2">
						<Button
							extraClasses="[&&]:bg-red-400 [&&]:hover:bg-red-300 px-4 py-1 font-mono min-w-40"
							onclick={() => device.arm()}>ARM</Button
						>
						<Button
							extraClasses="[&&]:bg-red-400 [&&]:hover:bg-red-300 px-4 py-1 font-mono min-w-40"
							onclick={() => device.disarm()}>DISARM</Button
						>
						<hr class="border-red-300 my-1 min-w-7/8" />
						<Button
							extraClasses="[&&]:bg-red-400 [&&]:hover:bg-red-300 px-4 py-1 font-mono min-w-40"
                            onclick={() => device.testFire1()}
							>TEST FIRE 1</Button
						>
						<Button
							extraClasses="[&&]:bg-red-400 [&&]:hover:bg-red-300 px-4 py-1 font-mono min-w-40"
                            onclick={() => device.testFire2()}
							>TEST FIRE 2</Button
						>
					</div>
				{/if}
			</div>

			<!-- Config -->
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
	{/if}
</div>
