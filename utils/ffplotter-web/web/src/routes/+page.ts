import type { PageLoad } from './$types';
import init, { power, parse, init_panic_hook } from "../../pkg/ffplotter_web";

export const load: PageLoad = async (params) => {
    await init("../../pkg/ffplotter_web_bg.wasm");

	init_panic_hook();

	return {
		power, parse
	}
};