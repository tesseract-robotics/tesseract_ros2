# A volumetric signed distance field (tesseract::geometry::SignedDistanceField).

# The serialized field, in exactly one of two standard VDB encodings; the unused field is left
# empty. Each is a single axis-aligned, uniformly scaled FloatGrid. At least one must be non-empty.
#   vdb  - OpenVDB FloatGrid, via tesseract::geometry::write/readSignedDistanceFieldVDB
#   nvdb - NanoVDB FloatGrid, via tesseract::geometry::write/readSignedDistanceFieldNVDB
uint8[] vdb
uint8[] nvdb

# Scale is not part of the blob (carried like the URDF attribute).
float64[3] scale
