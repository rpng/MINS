set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
# Release only: the default x64-windows triplet builds every package in BOTH debug and
# release, roughly doubling the vcpkg time. We only ship release, so skip the debug halves.
set(VCPKG_BUILD_TYPE release)
