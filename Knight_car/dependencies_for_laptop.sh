#!/bin/bash
set -e
set -x

./dependencies_common.sh


sudo apt install -y \
	bibtex2html \
	pdftk
