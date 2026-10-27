"""Refresh the exact canonical body-query math inside its per-instance wrapper.

The wrapper owns validated mesh input and an engine world-query boundary. This
tool copies numerical declarations directly from the pinned source; only static
storage duration changes where functions become per-character class methods.
"""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

PIN = "f9e19c41ec5412c974c3f3405f14562cc39598a2"
NAMES = ["FluidEmptyBox", "FluidGrow", "FluidMerge", "FluidBuildBvh",
         "FluidRefitBvh", "FluidSegmentBox", "FluidSegmentTriangle",
         "FluidBodySweep", "FluidResolveBodyAnchor", "FluidResolveSplat"]
STATIC = {"FluidEmptyBox", "FluidGrow", "FluidMerge", "FluidSegmentBox", "FluidSegmentTriangle"}


def extract(wolverine, output, manifest):
    commit = subprocess.check_output(["git", "-C", str(wolverine), "rev-parse", "HEAD"], text=True).strip()
    if commit != PIN:
        raise ValueError("Canonical Wolverine source pin changed")
    source_file = wolverine / "src/runtime/fluid_collision.h"
    raw = source_file.read_bytes()
    source = raw.decode()
    functions, declarations = [], []
    for name in NAMES:
        match = re.search(r"^static [^\n]*\b" + name + r"\(", source, re.M)
        if not match:
            raise ValueError("Missing canonical declaration: " + name)
        start = match.start()
        end = source.index("{", start) + 1
        depth = 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        original = source[start:end]
        functions.append(original if name in STATIC else original.replace("static ", "", 1))
        declarations.append({"name": name, "sourceSHA256": hashlib.sha256(original.encode()).hexdigest()})
    wrapper = output.read_text()
    begin = wrapper.index("static FluidAabb FluidEmptyBox(")
    end = wrapper.index("\npublic:\n std::function", begin)
    content = wrapper[:begin] + "\n".join(functions) + wrapper[end:]
    output.write_text(content, newline="\n")
    record = {"sourceCommit": PIN, "sourceSHA256": hashlib.sha256(raw).hexdigest(),
              "declarations": declarations, "headerSHA256": hashlib.sha256(content.encode()).hexdigest(),
              "boundaries": ["per-character storage", "validated native skinned mesh", "native world projection callback"]}
    if manifest:
        manifest.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wolverine", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path(__file__).parents[1] / "include/malemod/clinical/canonical_body_collision.hpp")
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()
    extract(args.wolverine, args.output, args.manifest)
