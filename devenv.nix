{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:

{
  languages.rust = {
    enable = true;
    channel = "nightly";
    components = [
      "rustc"
      "cargo"
      "rust-analyzer"
    ];
    targets = [
      "wasm32-unknown-unknown"
      "thumbv7em-none-eabihf"
    ];
  };

  languages.javascript = {
    enable = true;
    npm.enable = true;
  };

  packages = with pkgs; [
    openocd
    gcc-arm-embedded
    dfu-util
    libllvm
    wasm-pack
  ];
}
