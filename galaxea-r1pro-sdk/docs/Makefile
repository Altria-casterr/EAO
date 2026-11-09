.PHONY: check test

check:
	python -m compileall -q src/galaxea_r1pro example

test: check
	PYTHONPATH=src python -m unittest discover -s example -v
