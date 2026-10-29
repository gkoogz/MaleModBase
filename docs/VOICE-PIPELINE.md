# Reusable voice development pipeline

Core owns permanent semantic line IDs and script/recording validation. Character
scripts own the words and take revisions. Adapters own native playback, ID-to-
filename bindings and installation. No provider, voice clone or synthesis call
is part of this workflow, and private recordings never enter Git.

## Permanent IDs

`profiles/voice-lines.json` starts with the 24 recovered sequence slots:
MM-VOICE-0001 through MM-VOICE-0013 are phase1 variants1–13;
MM-VOICE-0014 through MM-VOICE-0024 are phase2 variants1–11.
`profiles/voice-lines.lock.json` prevents deleting or reassigning published IDs.
Retire a line instead of deleting or recycling its ID. Sorting the CSV, remaking
a take or changing character never renumbers it. For wording changes to the same
cue, retain its ID and increment the character script revision. A different cue
gets a new ID; `next-id` returns the next unused number including retired rows.
Append newly published IDs to the lock after review; never rewrite older slots.

## Create or remake a character script

```powershell
python tools/voice_pipeline.py check-catalog
python tools/voice_pipeline.py scaffold --character future_character --output build/voices/future_character.csv
python tools/voice_pipeline.py next-id
```

CSV columns are `line_id,character_id,cue,variant,text,revision,status,recording`.
The scaffold leaves text/recordings blank and status `needs_recording`. Fill in
the intended words. An adapter can preserve an existing character's text exactly
when migrating a script. Keep the ID and cue columns fixed.

Keep new recordings outside Git. Use a relative name such as
`geralt/MM-VOICE-0001/revision-2.wav`, update revision and recording, and mark
`recorded`. After listening and approving the actual take, mark `approved`.
The current PCM contract is nonempty mono 44,100 Hz 16-bit uncompressed WAV;
truncated audio, escaping paths and unsupported formats are rejected. The audio
root is explicit. Never leave a rejected recording bound to `needs_recording`.

```powershell
python tools/voice_pipeline.py check --script path/to/character.csv
python tools/voice_pipeline.py check --script path/to/character.csv --audio-root path/to/private/audio --require-approved --output build/voices/approved-delivery.json
```

Every active ID must be present once for one character. Cue reassignment,
duplicate/missing/unknown IDs and incomplete approval fail validation. Approved
delivery includes each ID, revision, cue/variant, PCM properties and file hash.
It is an offline handoff, not an installation or observed native playback pass.
Adapters must verify those bytes, map IDs to native assets, run their own audio
and gameplay acceptance, and install explicitly. No old-character audio fallback.

`tools/verify.py` and the shared CI template run catalog validation; unittest
discovery includes the pipeline regression tests. Future adapters can use the
same standard-library-only module without a game, graphics SDK or TTS service.

## Geralt adoption

The Witcher adapter stores `v2/voice-lines-geralt.csv`, `v2/voice-profile.json`
and a voice-only Core revision/hash lock. The recovered 24-line text is retained
exactly, and all rows currently need replacement recordings. Its prepared voice
pack tool maps the same permanent IDs to the existing two native sequence pools.
The rejected Geralt takes and active Witcher voice copies were deleted at the
user's request; Wolverine's own installed recordings were not changed.
The original filename transcript in `docs/wolverine-voice-pools.csv` is historical
source evidence, not an active Geralt audio manifest. Idle lines were not part
of that recovered 24-line generation plan; new idle scripts need new IDs rather
than fabricated transcripts or silently copied Wolverine recordings.
