# Stabilzer User Manual

This folder hosts the source used for generating Stabilizer's user manual.

The user manual is generated using `mdbook`, which can be installed via cargo:

```bash
cargo install mdbook
cargo install mdbook-toc
cargo install mdbook-linkcheck
```

To build the user manual locally, build docs for the firmware, copy them into the book source
directory, and then serve the book:

```bash
cargo doc --no-deps \
    -p miniconf -p idsp \
    -p ad9959 -p ad9912 -p stream -p platform -p signal_generator \
    -p encoded_pin -p serial_settings -p urukul \
    -p stabilizer
mv target/thumbv7em-none-eabihf/doc book/src/firmware
mdbook serve book
```

Once the `mdbook serve` command is run, the manual can be found on a web browser at
`localhost:3000`.
