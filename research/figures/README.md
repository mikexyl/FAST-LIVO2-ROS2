# Retained experiment evidence

Git contains reports, rendered PNG/PDF figures, configuration, compact JSON/CSV
metrics, audit scripts and textual provenance. Full run artifacts remain in the
local experiment directories and on workstation 148 at the paths documented in
each report.

Rerun recordings, point arrays, trajectories, evo ZIP archives, dataset ground
truth, per-frame JSONL logs and compressed source/log archives are excluded from
new commits. Existing tracked artifacts are preserved. A report's link to one
of these files requires the corresponding retained local artifacts; it is not
an available GitHub download. Exclusion from Git does not delete local files.

Reproduce the processing with the configurations and scripts documented in the
parent research directory. Evaluation reads ground truth only in its evaluation
stage. See `RESULTS-SUMMARY.md` for completed experiments and their limitations.
