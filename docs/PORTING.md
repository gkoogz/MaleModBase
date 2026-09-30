# Porting workflow

Pin a MaleModBase commit in each game's adapter repository (submodule or a
versioned release dependency). Do not copy the shared assets on every edit.
Resolve semantic joints and characterize the target skeleton before exporting
skin weights. A palette index without the captured engine palette is not a
portable skeleton.

For Witcher, first establish the installed game/REDkit versions, supported
character import format, body bind pose, joint hierarchy, material slots,
cloth/secondary-motion support and allowed runtime integration mechanisms.
The separate Witcher repository now owns a verified native export/import/cook
pipeline and a bare-body appearance override whose bare state was confirmed by
the user. Broader gameplay, anatomical fitting and live deformation are separate
gates. The extraction itself does not install into a game; deployment belongs
to the adapter and occurs only when requested by the user.

Retarget in character-local bind space; fit the pelvis seam and body attachment
with explicit landmarks. Validate source joint indices against a real palette.
Transfer morphology through the cage-to-surface bindings. Export material maps
according to target shader semantics rather than reusing D3D bytecode.

Run source/reference shape comparisons before changing numerical code. The
published tests depended on captures not present in Git. Obtain approved
fixtures or generate equivalent reference captures in the original checkout.
Then verify contacts, seam invariance, winding, animation poses, elapsed-time
hitches, multiple characters and target material rendering. In-game testing
and release packaging follow offline validation.

For progressive pelvic expansion, consume `surface.pelvic-collar` from Base.
Read PELVIC-COLLAR.md before implementing a spoke-specific fit or runtime bridge.
Shared numerical changes must pass the common fixtures and return to all spokes
through an explicitly pinned revision, followed by each spoke's parity gates.
