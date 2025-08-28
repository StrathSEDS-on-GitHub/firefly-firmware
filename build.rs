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

    let current_sha = match std::process::Command::new("git")
        .args(["describe", "--always", "--dirty=-dev"])
        .output() {
            Ok(output) => String::from_utf8_lossy(&output.stdout).trim().to_string(),
            Err(_) => "unknown".to_string(),
        };

    println!("cargo:rustc-env=BUILD_TIME={}", formatted);
    println!("cargo:rustc-env=GIT_SHA={}", current_sha);
}
