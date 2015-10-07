.. _developer:

Developer's guide
=================

Hi there VIKI-Developer,

This guide consists of a few parts.

#. :ref:`Framework description` - short description of the framework
#. :ref:`system_overview` - in depth, technical view of the system
#. :ref:`apidoc` - API for all functions

Updating documentation
----------------------
Once you have made changes please update the documentation by editing the *.rst* files in *VIKI/doc/* and re-build the documentation by running the following the the VIKI root directory

.. code-block:: bash

      sphinx-apidoc -f -o docs core; cd docs; make html; cd ..