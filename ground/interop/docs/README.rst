Some helpful hints for using Sphinx:

To build the documentation::

    $ source tools/venv/bin/activate
    $ cd docs
    $ make html

The homepage is at ``_build/html/index.html``. You should be able to navigate
the entire site from that page.

References:

* `reStructuredText syntax overview <http://sphinx-doc.org/latest/rest.html>`__
* `All Sphinx docs <http://sphinx-doc.org/latest/contents.html>`__

Some special syntax useful in Sphinx::

    :doc:`otherpage`
    :ref:`coolsection`

`doc`_ allows you to link to another page. The name of the page will be used
for the link.

`ref`_ allows you to link to any label, like a specific section. See the Sphinx
docs for more details.

.. _doc: http://sphinx-doc.org/latest/markup/inline.html#role-doc

.. _ref: http://sphinx-doc.org/latest/markup/inline.html#cross-referencing-arbitrary-locations
