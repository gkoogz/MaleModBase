# Independent cage shape transport

`shape_transport.py` consumes the existing source-verified AuthoredShape coarse
stage and the source-verified SourceRestFrame measurement. The original independent
section fit improved resizing but failed the user's next shape test: neighbouring
shaft fits gave an uneven contour and sharp distal transition. It also transported
coarse-default donor displacements into a differently posed large reference export.

The current shaft transport uses one measured radius ratio and axial span ratio
in the adapter's calibrated export frame. All eight shaft joints share uniform
radial scale. Axial changes move their rest centres along the measured export axis;
both crown joints receive exactly the same similarity transform about flex .76.
Their weighted blend cannot distort the free crown. Only the two lobe supports
retain independent Procrustes/RMS fits. Defaults are identity. A lattice includes branch coordinates,
UI defaults and endpoints for overall, width, length and scrotum.

This repairs the previous ratio approximation, which scaled an entire curved
chain nonuniformly and multiplied dependent dimensions. It does not implement
the final Wolverine prepared surface, fairing, logical sections, egg fitting or
coupled UnifiedCollar. Source glans enlargement uses an affine crown-base anchor
at flex .76 and a folded attachment transition; native cages only approximate
that transition. Source units remain uncalibrated; adapters supply fit conversion.
Short-profile previous-length fallback uses the measured neutral length for this
offline lattice rather than inventing a constant or claiming gameplay-history parity.
The exported reference's existing curve is retained; this is dimension transport,
not reconstruction of Wolverine's full logical surface at every control value.

Witcher adopts the module through its pinned Base revision and supplies measured
native bone frames, independent pelvis parenting, graph delivery and cooking.
Wolverine remains authoritative on its existing prepared-surface implementation;
its future Base adapter can consume AuthoredShape and this optional cage transport
without changing the original vertex solver. Other spokes may use the same module
only with their own calibrated frames and separate native/gameplay verification.
