.. AeroWorks documentation master file, created by
   sphinx-quickstart on Mon Mar 30 16:58:54 2015.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. _home:

AeroWorks documentation home
============================

Welcome to AeroWorks's documentation!

This documentation consists of three parts:

#. :ref:`Framework description` (what is the framework, what does it do and what for)
#. A :ref:`userguide`
#. An :ref:`apidoc`

If you've been here before perhaps you are looking for the following links:

#. :ref:`howtomod`
#. :ref:`howtoconfig`

The page you are seeing now is auto-generated using Sphinx. If you have sphinx you can update the documentation by running the following line in the root of the project.

.. code-block:: bash

      sphinx-apidoc -f -o docs core; cd docs; make html; cd ..

The developers will also update the documentation in the repository when important changes are done. In that case just updating your repository to the latest version is sufficient.

Contents
--------

.. toctree::
   :maxdepth: 2

   framework_description
   api
   userguide
   developers
   howtoconfig
   howtomod
   modules
   virtualbox

Automatically generated pages
-----------------------------
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`