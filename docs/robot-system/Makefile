# Build the robot system document. Requires pdflatex and bibtex.
#   make        -> main.pdf (also copied to bbai-robot-system.pdf)
#   make clean  -> remove build artifacts

MAIN = main

all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex $(wildcard chapters/*.tex) refs.bib
	pdflatex -interaction=nonstopmode -halt-on-error $(MAIN).tex
	bibtex $(MAIN)
	pdflatex -interaction=nonstopmode -halt-on-error $(MAIN).tex
	pdflatex -interaction=nonstopmode -halt-on-error $(MAIN).tex
	cp $(MAIN).pdf bbai-robot-system.pdf

clean:
	rm -f *.aux *.bbl *.blg *.log *.out *.toc *.lof *.lot *.fls *.fdb_latexmk
	rm -f chapters/*.aux

distclean: clean
	rm -f $(MAIN).pdf

.PHONY: all clean distclean
