v0.2.0 - Jan 04, 2016
=====================
- Fixed installation of shared library symlink.
- Switched LFE channel interpolation to FIR filter by default.
- Added command line option and context flag to select IIR filter for LFE
  channel interpolation.
- Fixed two bugs that could cause decoder to crash when processing invalid
  input.
- Fixed handling of sync loss when decoding MA streams with multiple frequency
  bands.
- Fixed decoding of MA streams with multiple frequency bands when sampling
  frequency of the first band is not 96 kHz.
- Fixed decoding to standard output on Windows causing junk to be appended
  after normal PCM data.


v0.1.0 - Nov 27, 2015
=====================
- Initial public release.
