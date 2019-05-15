{ stdenv, rustPlatform, cacert, git, cargo-vendor }:

with rustPlatform;
let
  sha256 = "1m4cxf6c4lh28xv4iagp20ni97cya1f12yg58q0m733qahk8gncb";
  fetchcargo = import ./fetchcargo.nix {
    inherit stdenv cacert git cargo-vendor;
    inherit (rust) cargo;
  };
  stabilizerDeps = fetchcargo {
    name = "stabilizer";
    src = ../.;
    inherit sha256;
  };
in

buildRustPackage rec {
  name = "stabilizer";
  version = "0.0.0";

  src = ../.;
  cargoSha256 = sha256;

  buildInputs = [ stabilizerDeps ];
  patchPhase = ''
    cat >> .cargo/config <<EOF
    [source.crates-io]
    replace-with = "vendored-sources"

    [source.vendored-sources]
    directory = "${stabilizerDeps}"
    EOF
  '';

  buildPhase = ''
    export CARGO_HOME=$(mktemp -d cargo-home.XXX)
    cargo build --release
  '';

  doCheck = false;
  installPhase = ''
    mkdir -p $out/lib
    cp target/thumbv7em-none-eabihf/release/stabilizer $out/lib/
  '';
}
