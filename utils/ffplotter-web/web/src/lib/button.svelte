<script lang="ts">
	const { onclick = (() => {}), children, extraClasses = "" } = $props();
</script>

<button
	class={`bg-teal-500 text-white rounded hover:bg-teal-600 transition-colors cursor-pointer ${extraClasses}`}
	onclick={onclick}
>
	{@render children()}
</button>
