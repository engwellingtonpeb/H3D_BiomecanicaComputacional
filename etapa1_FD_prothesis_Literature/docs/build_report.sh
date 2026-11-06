#!/bin/sh
# Compila o relatorio: copia o codigo-fonte atual (o MiKTeX nao le ../) e roda pdflatex 2x
cd "$(dirname "$0")"
mkdir -p codigo && cp ../src/*.m ../src/*.ps1 codigo/
cp ../results/train_log.csv ../results/agent_comparison.csv figs/ 2>/dev/null
for i in 1 2; do pdflatex -interaction=nonstopmode -halt-on-error relatorio_tecnico.tex > build_stdout.txt 2>&1 || { grep -a -E "^!|^l\.[0-9]+" relatorio_tecnico.log; exit 1; }; done
grep -a "Output written" relatorio_tecnico.log
