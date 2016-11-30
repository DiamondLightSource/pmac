# Documentation

Building the documentation has the following prerequisites:

doxygen (www.doxygen.org)
sphinx (www.sphinx-doc.org)
breathe (https://github.com/michaeljones/breathe)

Doxygen should be installed and found within the PATH environment variable.
Sphinx should be installed and found within the PATH environment variable.
Breathe is a python module and is expected to be found on the PYTHONPATH environment variable.

To build the documentation into a set of html files:

make html

The documentation index page will be located at:

./build/html/index.html


