#!/usr/bin/env python3
"""Compile the portable, frozen LaTeX source without the experiment workspace."""
import os
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parent
stem = 'technical-report'
out = root.parent / 'tmp' / 'latex-portable'
out.mkdir(parents=True, exist_ok=True)
pdf = root.parent / 'output' / 'pdf' / (stem + '.pdf')
env = dict(os.environ, BIBINPUTS=str(root) + os.pathsep)
latex = ['pdflatex', '-interaction=nonstopmode', '-halt-on-error',
         f'-jobname={stem}', f'-output-directory={out}', 'main.tex']
commands = (latex, ['bibtex', stem], latex, latex)
for command in commands:
    result = subprocess.run(command, cwd=out if command[0] == 'bibtex' else root,
                            env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    if result.returncode:
        raise SystemExit(result.stdout[-8000:])
log = (out / (stem + '.log')).read_text()
warnings = [line for line in log.splitlines()
            if any(word in line for word in ('Warning', 'Overfull', 'Underfull'))]
print('\n'.join(warnings) or 'No LaTeX warnings.')
pdf.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(out / (stem + '.pdf'), pdf)
print(pdf)
