# Getting Started

Sphinx is a *documentation generator* or a tool that translates a set of plain
text source files into various output formats, automatically producing
cross-references, indices, etc.  That is, if you have a directory containing a
bunch of :doc:`/usage/restructuredtext/index` or :doc:`/usage/markdown`
documents, Sphinx can generate a series of HTML files, a PDF file (via LaTeX),
man pages and much more.

## Setting up the documentation sources

The root directory of a Sphinx collection of plain-text document sources is
called the :term:`source directory`.  This directory also contains the Sphinx
configuration file :file:`conf.py`, where you can configure all aspects of how
Sphinx reads your sources and builds your documentation.  

Sphinx comes with a script called :program:`sphinx-quickstart` that sets up a
source directory and creates a default :file:`conf.py` with the most useful
configuration values from a few questions it asks you. To use this, run:

```

    sphinx-quickstart

```

