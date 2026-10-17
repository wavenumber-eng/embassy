//! USB device controller backends.
//!
//! KHCI USBFS and ChipIdea USBHS are distinct engines. Keep controller-specific
//! public APIs in their own backend; this module does not select by family.

#[cfg(feature = "mcxa156")]
pub mod khci;
