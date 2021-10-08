# Stabilzer User Manual

This folder hosts the source used for generating Stabilizer's user manual.

The user manual is generated using `mdbook`, which can be installed via cargo:
```
cargo install mdbook
cargo install mdbook-toc
cargo install mdbook-linkcheck
```

To build the user manual locally, build docs for the firmware, copy them into the book source
directory, and then sere the book:
```
cargo doc --no-deps -p miniconf -p stabilizer -p dsp -p ad9959
mv target/thumbv7em-none-eabihf/doc book/src/firmware
cd book
mdbook serve
```

Once the `mdbook serve` command is run, the manual can be found on a web browser at
`localhost:3000`.
