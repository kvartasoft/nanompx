# Conformance / interop vectors
#
# Place binary NanoMPX packet streams and matching s24le PCM references here.
# Naming: <name>.nmpx + <name>.s24 (+ optional <name>.json metadata).
#
# Generate a PCM reference with the CLI after building:
#   nanompx_cli encode --mode pcm -i tone.s24 -o tests/vectors/tone_pcm.nmpx
