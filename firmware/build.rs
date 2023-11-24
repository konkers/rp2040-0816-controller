use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Copy memory.x to our output directory.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    // Put memory.x on the linker search path.
    println!("cargo:rustc-link-search={}", out.display());
    // Rebuild on changes to memory.x
    println!("cargo:rerun-if-changed=memory.x");

    // Copypastad from other projects.  Difficult to find an explanation of why
    // this is needed or what exactly it does.  Affects alignment.
    println!("cargo:rustc-link-arg-bins=--nmagic");

    // Pull in third_party linker scripts.
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
