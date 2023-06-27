// Use all the autocxx types which might be handy.
use autocxx::prelude::*;

include_cpp! {
    #include "mylibrary.h"
    safety!(unsafe_ffi)
    generate!("my_special_function") // allowlist a function
}

fn main() {
    println!("hi: {}", ffi::my_special_function(1.0));
}