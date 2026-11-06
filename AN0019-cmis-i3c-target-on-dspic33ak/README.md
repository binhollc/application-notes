# AN0019: Building a CMIS I3C target on a Microchip dsPIC33AK and testing it with the Binho Supernova

Draft, revision 0.1. A CMIS optical-module register model on the dsPIC33AK256MPS306 I3C
target, answering write + repeated START + read by arming from the receive DMA, with the
host-side checks run from a Binho Supernova.

| File | What it is |
|---|---|
| `AN0019.md` | The note |
| `AN0019.pdf` | Draft PDF |
| `assets/cmis_target_check.py` | Host checks: reads, mixed, ccc, daa, i2c |
| `assets/cmis_mci.py` | Shared CMIS utility, byte copy of `_shared/tools/cmis_mci.py` |
| `assets/cmis_image.h` | CMIS 5.3 Appendix C register image, as built into the target |
