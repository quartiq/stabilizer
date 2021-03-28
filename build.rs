fn main() {
    println!("cargo:rerun-if-changed=memory.x");

    cc::Build::new()
        .file("src/startup.S")
        .compile("startup");
    println!("cargo:rerun-if-changed=src/startup.S");
}

