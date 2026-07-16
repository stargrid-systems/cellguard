//! Adds the local `memory.x` to the linker search path so the boot/app flash
//! split takes effect at link time.

fn main() {
    println!("cargo:rustc-link-search=.");
}
