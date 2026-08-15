import os


def find_project_root(*extra_starts: str) -> str:
    """Locate dial-mpc root by searching upward for pm_links.py."""
    starts: list[str] = []
    for path in extra_starts:
        if path:
            starts.append(os.path.abspath(path))

    try:
        if __file__:
            starts.append(os.path.dirname(os.path.abspath(__file__)))
    except NameError:
        pass

    seen: set[str] = set()
    for start in starts:
        cur = os.path.abspath(start)
        if cur in seen:
            continue
        seen.add(cur)

        for _ in range(8):
            if os.path.isfile(os.path.join(cur, "pm_links.py")):
                return cur
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    raise FileNotFoundError(
        "Cannot find dial-mpc project root (pm_links.py). "
        "Run /home/ubuntu/dial-mpc/import_blender_pm.py from the Scripting tab."
    )
