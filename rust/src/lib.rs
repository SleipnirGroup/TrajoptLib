use autocxx::prelude::*;

include_cpp! {
    #include "mylibrary.h"
    safety!(unsafe_ffi)
    generate!("my_special_function") // allowlist a function
}

pub fn my_ffi_func() -> f64 {
    ffi::my_special_function(1.0)
}