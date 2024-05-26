#[cfg(not(any(feature = "target-maxi", feature = "target-mini")))]
compile_error! {
    "You need to set a target to build for.
    
    E.g
        `cargo build --features target-mini`

    Supported targets: target-mini, target-maxi"
}

#[cfg(all(feature = "target-mini", feature = "target-maxi"))]
compile_error!("You can only build for one target at a time");

fn main() {}
