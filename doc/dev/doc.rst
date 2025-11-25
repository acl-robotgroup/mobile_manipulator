.. _dev_doc:

==========================
Building the Documentation
==========================

The documentation is built using `Sphinx
<https://www.sphinx-doc.org/en/master>`_ using `Read the Docs
<https://about.readthedocs.com/>`_ theme.

Docker Build (recommended)
==========================

Use Docker for an isolated build and nginx hosting:

.. code-block:: bash

   # in <repo_root>/doc
   docker compose up -d

Open http://localhost:8000 to view the site. Rebuild after changes with:

.. code-block:: bash

   docker compose up --force-recreate -d

Stop the container with:

.. code-block:: bash

   docker compose down

Local Build
===========

After installing ``docs/requirements.txt``, you can build locally:

.. code-block:: bash

   # in <repo_root>/doc
   make html

Open ``build/html/index.html`` in a browser to preview the output.
