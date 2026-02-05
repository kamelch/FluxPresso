// Native build stub to avoid "Nothing to build" when running `pio run -e native`.
// Unit tests are in /test and use the Unity runner (PIO_UNIT_TESTING).
// We provide a minimal main() only for non-testing native builds.

#if !defined(PIO_UNIT_TESTING)
int main() {
    return 0;
}
#endif
