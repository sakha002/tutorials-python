===============
Getting Started
===============

Sphinx is a *documentation generator* or a tool that translates a set of plain
text source files into various output formats, automatically producing
cross-references, indices, etc.  That is, if you have a directory containing a
bunch of :doc:`/usage/restructuredtext/index` or :doc:`/usage/markdown`
documents, Sphinx can generate a series of HTML files, a PDF file (via LaTeX),
man pages and much more.

Sphinx focuses on documentation, in particular handwritten documentation,
however, Sphinx can also be used to generate blogs, homepages and even books.
Much of Sphinx's power comes from the richness of its default plain-text markup
format, :doc:`reStructuredText </usage/restructuredtext/index>`, along with
it's :doc:`significant extensibility capabilities </development/index>`.

The goal of this document is to give you a quick taste of what Sphinx is and
how you might use it. When you're done here, you can check out the
:doc:`installation guide </usage/installation>` followed by the intro to the
default markup format used by Sphinx, :doc:`reStucturedText
</usage/restructuredtext/index>`.

For a great "introduction" to writing docs in general -- the whys and hows, see
also `Write the docs`__, written by Eric Holscher.

.. __: http://www.writethedocs.org/guide/writing/beginners-guide-to-docs/


Setting up the documentation sources
------------------------------------

The root directory of a Sphinx collection of plain-text document sources is
called the :term:`source directory`.  This directory also contains the Sphinx
configuration file :file:`conf.py`, where you can configure all aspects of how
Sphinx reads your sources and builds your documentation.  [#]_

Sphinx comes with a script called :program:`sphinx-quickstart` that sets up a
source directory and creates a default :file:`conf.py` with the most useful
configuration values from a few questions it asks you. To use this, run:

.. code-block:: shell

   $ sphinx-quickstart


