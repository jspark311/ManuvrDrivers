# Composite device drivers

This directory is for hardware drivers that are combinations of the more
elemental drivers that this repo is generally concerned with. Most projects
will not use any of these drivers.

It is acceptable for drivers to composite other composite drivers, if that is
for some reason desirable.

These drivers will not be pulled into a project by inclusion of `ManuvrDrivers.h`.
If needed, they should be included on a direct basis by the programs that want them.

### ManuvrPMU
