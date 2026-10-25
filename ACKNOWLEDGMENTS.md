# Acknowledgments / Teşekkürler

## Arduino PID Library (PID_v1)

Brett Beauregard's [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library) is a well-known reference for PID controller design and Arduino use. Its current 1.2.x source states that it uses the MIT License.

**dengebot is not a fork of PID_v1, and does not link to or depend on PID_v1.** A manual comparison of dengebot 1.6.1's controller implementation and public API against PID_v1 1.2.1 did not identify verbatim source-code blocks or distinctive implementation copied from PID_v1. Both implementations use standard PID ideas, such as proportional/integral/derivative terms, output limiting and integral windup protection. These ideas alone do not establish code derivation.

We acknowledge Brett Beauregard's educational contributions and the PID_v1 project as prior art/inspiration. This attribution does not imply endorsement, authorship of dengebot, or inclusion of PID_v1 source code.

If any actual PID_v1 code is incorporated in future releases, preserve the original copyright and applicable MIT license notice alongside that code, and update this document accordingly.
