#![no_std]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

pub mod hardware;
pub mod net;

/// Macro to reduce rightward drift when calling the same closure-based API
/// on multiple structs simultaneously, e.g. when accessing DMA buffers.
/// This could be improved a bit using the tuple-based style from `mutex-trait`.
#[macro_export]
macro_rules! flatten_closures {
    ($fn:ident, $e:ident, $fun:block) => {
        $e.$fn(|$e| $fun ).unwrap()
    };
    ($fn:ident, $e:ident, $($es:ident),+, $fun:block) => {
        $e.$fn(|$e| flatten_closures!($fn, $($es),*, $fun)).unwrap()
    };
}
