# Wolverine voice pools

This transcript inventories the two voice pools currently installed under `C:\Games\X-Men Origins Wolverine\Binaries\TeachingAudio`. The CSV beside this note preserves the filename-encoded transcript for every clip and records the source MP3 and installed WAV byte sizes: [wolverine-voice-pools.csv](wolverine-voice-pools.csv).

## Identification

The installed WAVs use generic `clip-NN.wav` names. The Windows Recycle Bin still has original-name records for matching MP3s; those original filenames encode the lines. I matched each WAV to a deleted MP3 by decoded duration and 10 ms RMS audio-envelope correlation. Every selected match has correlation 1.000. The CSV records both byte sizes, duration, original filename and evidence. Ambiguous size/duration collisions were resolved by comparing the decoded audio envelopes.

No audio binaries are included here. This is a text transcript and local provenance record for later adapter work.

## Pool inventory

The installed directory contains 13 Phase 1 clips and 11 Phase 2 clips (24 total). `src/runtime/teaching_audio.h` loads each phase directory at runtime. The older synthetic cue test in `tools/teaching/sequence_test.cpp` sets counts to 13 and 7 and exercises seven Phase 2 indices; that test expectation does not match the currently installed Phase 2 directory. This note records the observed installation and does not change runtime behavior.

The transcript text is preserved exactly as encoded in the original MP3 filenames, including their capitalization, spelling and punctuation quirks. It is source evidence rather than a fresh waveform transcription.

This is a transcription/provenance record only. Geralt audio playback and sequence integration remain deferred under [PROJECT-CONTEXT.md](PROJECT-CONTEXT.md).
The source MP3 sizes in the CSV agree with the byte counts stored in each matching recoverable Recycle Bin payload.
