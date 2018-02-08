def _impl(rctxt):
  path = rtcxt.attr.path
  # This path is a bit ugly to get the actual path if it is relative.
  if path[0] != "/":
    # rctxt.path(Label("//:BUILD")) will returns a path to the BUILD file
    # in the current workspace, so getting the dirname get the path
    # relative to the workspace.
    path = rctxt.path(Label("//:BUILD")).dirname + "/" + path
  # Copy the repository
  result = rctxt.execute(["cp", "-fr", path + "/*", rctxt.path()])
  if result.return_code != 0:
    fail("Failed to copy %s (%s)" % (rctxt.attr.path, result.return_code))
  # Now patch the repository
  patch_file = str(rctxt.path(rctxt.attr.patch).realpath)
  result = rctxt.execute(["bash", "-c", "patch -p0 < " + patch_file])
  if result.return_code != 0:
    fail("Failed to patch (%s): %s" % (result.return_code, result.stderr))

local_patched_repository = repository_rule(
  implementation=_impl,
  attrs={
    "path": attr.string(mandatory=True),
    "patch": attr.label(mandatory=True)
  },
  local = True
)
