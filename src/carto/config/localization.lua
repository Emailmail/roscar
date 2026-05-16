-- Pure localization: loads pre-built map, scan-to-submap matching only.
-- Inherits all mapping.lua settings, then overrides for localization.

include "mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

return options
