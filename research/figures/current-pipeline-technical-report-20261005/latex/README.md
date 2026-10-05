# Rebuilding the technical report

This is a multi-file LaTeX project with local figure assets. It needs Python 3, pdfLaTeX, BibTeX and standard TeX packages. No dataset, SLAM installation, GPU, remote access or plotting environment is required for compilation.

From the package root:

```sh
python3 latex/build_tables.py
python3 latex/compile.py
```

The final document is `output/pdf/technical-report.pdf`. Auxiliary build files stay in `tmp/latex-portable/`. `main.tex` includes six current-method sections and six generated tables. Tables read the retained evidence and check byte sums, per-robot/joint ATE consistency, occupancy coverage and comparator completeness.

Figure PDF assets are prebuilt. Timing figure regeneration code and its captured input are retained under `evidence/cbs-timing/` for provenance; regeneration is not part of compilation. The numerical tables never refit trajectories or rerun the experiment.

The included configurations are the recorded run inputs. Per-robot CBS YAML and calibrated native frontend files identify effective execution settings; an inherited field in a general profile is not evidence that an unused mode ran.
