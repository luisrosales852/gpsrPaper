"""Visual MuJoCo simulation harness for the GPSR merger.

The simulator is deliberately a *consumer* of ``gpsr_merger``: it supplies a
``locator``, an ``origin`` and a list of subtask handlers, then runs the real
``merge()`` and the real ``build_tree()``. Nothing in ``gpsr_merger/`` is
imported-and-patched or re-implemented here, so the rendered video is evidence
about the shipped planner rather than about a stand-in.
"""
