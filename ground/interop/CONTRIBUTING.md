Contributing
============

We follow the standard Python [PEP8](https://www.python.org/dev/peps/pep-0008/)
style guide.

Additionally, we give the [YAPF](https://github.com/google/yapf) formatter the
final say in formatting code.

* Please run `yapf` on all files in your changes before sending a Pull Request.
* The `tools/format.sh` script will automatically determine which files have
  changed, and run `yapf` on them.
* See YAPF's
  [FAQ](https://github.com/google/yapf#why-does-yapf-destroy-my-awesome-formatting)
  for details on disabling the formatter on select portions of the source.
