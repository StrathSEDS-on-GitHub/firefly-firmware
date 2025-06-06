use time::macros::format_description;

#[cfg(not(any(feature = "target-maxi", feature = "target-mini", feature= "target-ultra")))]
compile_error! {
    "You need to set a target to build for.
    
    E.g
        `cargo build --features target-mini`

    Supported targets: target-mini, target-maxi, target-ultra"
}


// Also error if more than one target is set
#[cfg(any(
    all(feature = "target-maxi", feature = "target-mini"),
    all(feature = "target-maxi", feature = "target-ultra"),
    all(feature = "target-mini", feature = "target-ultra"),
))]
compile_error! {
    "You can only set one target at a time.
    
    E.g
        `cargo build --features target-mini`

    Supported targets: target-mini, target-maxi, target-ultra"
}

fn main() {
    let current_time = time::OffsetDateTime::now_utc();
    let formatted = current_time.format(format_description!("[day],[month],[year],[hour],[minute],[second]")).unwrap();

    println!("cargo:rustc-env=BUILD_TIME={}", formatted);
}
