# A volumetric signed distance field (tesseract::geometry::SignedDistanceField).

# The serialized field, in exactly one of two standard VDB encodings; the unused field is left
# empty. Each holds axis-aligned, uniformly scaled FloatGrids: one, except for a multi-level field
# (below). Exactly one must be non-empty: fromMsg() rejects a message with neither payload set and
# one with both payloads set.
#   vdb  - OpenVDB FloatGrid, via tesseract::geometry::write/readSignedDistanceFieldVDB
#   nvdb - NanoVDB FloatGrid(s), via tesseract::geometry::write/readSignedDistanceFieldNVDB
uint8[] vdb
uint8[] nvdb

# How the field is held, by the number of band clamps (each in grid units; see the sparse
# SignedDistanceField constructor). NanoVDB has no field for them, so they travel here.
#   0  - a dense grid: one FloatGrid in vdb or nvdb, read into a full box
#   1  - a sparse field: the one FloatGrid in nvdb, read in place
#   >1 - a multi-level field: nvdb holds one FloatGrid per level, finest first, one clamp each
# A sparse or multi-level field always travels in nvdb.
float64[] band_clamps

# Scale is not part of the blob (carried like the URDF attribute).
float64[3] scale

# How the field reads between samples (tesseract::geometry::SignedDistanceFieldInterpolation): the
# distance, and the gradient, which may be read differently. Not part of either blob.
uint8 TRILINEAR=0
uint8 TRICUBIC=1
uint8 interpolation
uint8 gradient_interpolation
