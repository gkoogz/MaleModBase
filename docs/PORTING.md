# Porting workflow

Pin a MaleModBase commit in each game's adapter repository (submodule or a
versioned release dependency). Do not copy the shared assets on every edit.
Resolve semantic joints and characterize the target skeleton before exporting
skin weights. A palette index without the captured engine palette is not a
portable skeleton.

For Witcher, first establish the installed game/REDkit versions, supported
character import format, body bind pose, joint hierarchy, material slots,
cloth/secondary-motion support and allowed runtime integration mechanisms.
No REDkit import, Witcher mesh format or scripting capability is asserted by
this draft. E:/SteamLibrary/steamapps/common/The Witcher 3 is recorded only as
the user-provided installation path. Nothing has been installed there.

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
