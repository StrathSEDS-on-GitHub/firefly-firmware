{
  description = "firefly-firmware flake";
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";

    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs =
    {
      self,
      nixpkgs,
      fenix,
      ...
    }@inputs:
    let
      supportedSystems = [ "x86_64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
      pkgsFor = nixpkgs.legacyPackages;
      fenixFor = fenix.packages;
    in
    {
      devShells = forAllSystems (system: {
        default =
          let
            rust-toolchain =
              with fenixFor.${system};
              combine [
                default.toolchain
                complete.rust-src
                targets.thumbv7em-none-eabihf.latest.rust-std
              ];
          in
          pkgsFor.${system}.mkShell {

            buildInputs = with pkgsFor.${system}; [
              rust-toolchain
              rust-analyzer
              openocd
              gcc-arm-embedded
            ];
          };
      });
    };
}
