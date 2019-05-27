{ # Use master branch of the overlay by default
  mozillaOverlay ? import (builtins.fetchTarball https://github.com/mozilla/nixpkgs-mozilla/archive/master.tar.gz),
  rustManifest ? builtins.fetchurl "https://static.rust-lang.org/dist/channel-rust-nightly.toml"
}:

let
  pkgs = import <nixpkgs> { overlays = [ mozillaOverlay ]; };
in
with pkgs;
let
  rustPlatform = recurseIntoAttrs (callPackage ./nix/rustPlatform.nix {
    inherit rustManifest;
  });
  stabilizer = callPackage ./nix/stabilizer.nix { inherit rustPlatform; };
in
stdenv.mkDerivation {
  name = "stabilizer-dist";
  buildInputs = [ stabilizer ];
  src = ./.;
  dontBuild = true;

  installPhase =
    let
      firmwareBinary = "$out/lib/stabilizer.elf";
    in ''
      mkdir -p $out/bin $out/lib $out/nix-support

      ln -s ${stabilizer}/lib/stabilizer ${firmwareBinary}

      echo file binary-dist ${firmwareBinary} >> $out/nix-support/hydra-build-products
    '';
}
