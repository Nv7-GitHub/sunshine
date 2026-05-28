fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let core_dir = format!("{}/../../sunshine_core", manifest_dir);

    cc::Build::new()
        .files([
            format!("{}/src/utils.c",        core_dir),
            format!("{}/src/kalman.c",       core_dir),
            format!("{}/src/derot_filter.c", core_dir),
            format!("{}/src/control.c",      core_dir),
            format!("{}/src/brain.c",        core_dir),
        ])
        .include(format!("{}/include", core_dir))
        .flag_if_supported("-lm")
        .compile("sunshine_core");

    println!("cargo:rerun-if-changed={}/src", core_dir);
    println!("cargo:rerun-if-changed={}/include", core_dir);
}
