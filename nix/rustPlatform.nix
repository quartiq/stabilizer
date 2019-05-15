{ recurseIntoAttrs, stdenv, lib,
  makeRustPlatform,
  fetchurl, patchelf,
  rustManifest ? ./channel-rust-nightly.toml
}:

let
  targets = [
    "thumbv7em-none-eabihf"
  ];
  rustChannel =
    lib.rustLib.fromManifestFile rustManifest {
      inherit stdenv fetchurl patchelf;
    };
  rust =
    rustChannel.rust.override {
      inherit targets;
    };
in
makeRustPlatform {
  rustc = rust;
  cargo = rust;
}
