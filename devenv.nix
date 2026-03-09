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
    targets = [ "thumbv7em-none-eabihf" ];
  };

  packages = with pkgs; [
    openocd
    gcc-arm-embedded
  ];
}
