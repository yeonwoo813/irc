uint8 status
float32 angle
bool hurdle_ready
# Decision input: image-space distance to the line/hurdle intersection.
# -1 when unavailable or hurdle processing is suppressed; logging only.
float64 hurdle_intersection_distance_px -1.0
