# Collaborative LiDAR Mapping with Multi-Layer BEVs and Belief Sharing on Robot-Decoupled Submap Graphs

This is the technical report for the fixed-circle, geometry-maturation pipeline: persistent-map EllipseLIO, point-cloud multilayer BEVs, geometric verification, PCM, persistent distributed CBS with GPU VGICP factors, and submap-based trajectory/map reconstruction.

- [Technical report — PDF](output/pdf/technical-report.pdf)
- [Editable LaTeX](latex/main.tex) and [build instructions](latex/README.md)
- [Portable source package](technical-report-source.zip)
- [Result summary](REPORT.md)
- [Claim–evidence map and manuscript review](EDITORIAL-NOTES.md)
- [Report verification](REPORT-VERIFICATION.json) and [file hashes](SHA256SUMS.json)

The main experiment is the simultaneous 1× replay of all 14 GRACO sequences with geometry maturation. It forms one component and reaches 0.933 m final joint ATE with 546.77 MB of inter-robot application payload. The report includes the relevant immediate-freezing ablation, explicitly scoped native-baseline communication context, and computation measurements as the online graph grows.

The manuscript describes one method. It does not combine results from other pipeline configurations into the reported method. Evidence and configs are retained separately for reproducibility; the report is not a development history.

## Figures and tables

- [Joint map](figures/online-global-map.png)
- [CBS computation over experiment progress](output/pdf/cbs-update-progress.pdf)
- [CBS computation versus local graph size](output/pdf/cbs-local-graph-scaling.pdf)
- [Per-robot timing](output/pdf/cbs-per-robot-progress.pdf)
- [Generated LaTeX tables](latex/generated/)

The source bundle contains the figures, tables, compact numerical evidence, configurations and hashes needed to build the report without the dataset or remote access. Bulk sensor captures and Rerun recordings remain on workstation 148.
