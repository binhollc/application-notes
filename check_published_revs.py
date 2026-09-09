#!/usr/bin/env python3
"""Check that the CDN serves every published note, at its current revision.

Published blog posts link to fixed, unversioned CDN URLs, so there are two
separate ways a reader can be let down and they fail differently:

  resolves    the URL returns 200 rather than 404, so the link is not broken
  current     the object behind it was built from the revision in this repo

A note whose rev is bumped here without a republish keeps answering 200 while
handing readers the previous document. That is the failure this catches, and it
is invisible from the link alone.

The published revision is read out of the assets archive, not the PDF. The
archive carries the note's own Markdown, so its `rev:` is the revision that was
actually built, stated by the artifact itself. Scraping the PDF looks like the
obvious approach but its fonts are subset and the glyph mapping is lossy, so
text recovered from it cannot be trusted for something this specific. Comparing
md5 against a local build does not work either: the PDF embeds a build
timestamp, so two builds of identical source never match.

A note at revision 0.x is a draft. Nothing is published for it, so it is
reported as skipped rather than missing.

Run from the repository root:  python3 check_published_revs.py [AN0004 ...]
Exits non-zero if any published artifact is missing or behind its source.
"""
import io
import pathlib
import re
import sys
import time
import urllib.error
import urllib.request
import zipfile

BASE = "https://cdn.binho.io/application-notes"
TIMEOUT = 60


def rev_of(text):
    m = re.search(r"^rev:\s*(\S+)", text, re.M)
    return m.group(1) if m else None


def fetch(url):
    """Fetch with a cache buster, so this reads the origin and not an edge copy.

    A stale edge answers 200 with the previous object and would otherwise be
    reported as a stale publish, which is a different problem with a different
    fix: an edge copy corrects itself when its TTL expires, a stale origin
    never does.
    """
    request = urllib.request.Request(
        f"{url}?cb={int(time.time())}",
        headers={"Cache-Control": "no-cache", "Pragma": "no-cache"})
    with urllib.request.urlopen(request, timeout=TIMEOUT) as response:
        return response.status, response.read()


def published_rev(note):
    """The rev recorded in the Markdown inside the published assets archive."""
    status, body = fetch(f"{BASE}/{note}/{note}-assets.zip")
    if status != 200:
        return None, f"assets archive returned HTTP {status}"
    try:
        with zipfile.ZipFile(io.BytesIO(body)) as archive:
            for name in archive.namelist():
                if name.endswith(f"{note}.md"):
                    text = archive.read(name).decode("utf-8", "replace")
                    rev = rev_of(text)
                    if rev is None:
                        return None, f"no 'rev:' in {name} inside the archive"
                    return rev, None
    except zipfile.BadZipFile:
        return None, f"assets archive is not a readable zip ({len(body):,} bytes)"
    return None, f"archive holds no {note}.md"


def main(argv):
    root = pathlib.Path(__file__).resolve().parent
    wanted = set(argv[1:])
    folders = sorted(p for p in root.glob("AN[0-9][0-9][0-9][0-9]-*") if p.is_dir())
    if wanted:
        folders = [f for f in folders if f.name.split("-")[0] in wanted]
    if not folders:
        print("no notes matched")
        return 2

    problems = []
    drafts = []
    print(f"  {'note':<8}{'source':>8}{'published':>11}   verdict")

    for folder in folders:
        note = folder.name.split("-")[0]
        md = folder / f"{note}.md"
        if not md.exists():
            problems.append(f"{note}: no {note}.md in {folder.name}")
            continue

        source = rev_of(md.read_bytes().decode("utf-8"))
        if source is None:
            problems.append(f"{note}: no 'rev:' in {md.name}")
            continue
        if source.startswith("0."):
            drafts.append(f"{note} (rev {source})")
            continue

        # The PDF is what a blog post usually links to, so a broken link there
        # matters even when the archive is fine. Check it separately.
        pdf_note = ""
        try:
            status, body = fetch(f"{BASE}/{note}/{note}.pdf")
            if status != 200:
                problems.append(f"{note}: {note}.pdf returned HTTP {status}")
                pdf_note = f", PDF HTTP {status}"
            elif not body.startswith(b"%PDF"):
                problems.append(f"{note}: {note}.pdf is not a PDF "
                                f"({len(body):,} bytes)")
                pdf_note = ", PDF unreadable"
        except (urllib.error.URLError, OSError) as exc:
            problems.append(f"{note}: {note}.pdf did not fetch ({exc})")
            pdf_note = ", PDF unreachable"

        try:
            got, why = published_rev(note)
        except (urllib.error.URLError, OSError) as exc:
            problems.append(f"{note}: assets archive did not fetch ({exc})")
            print(f"  {note:<8}{source:>8}{'?':>11}   NOT REACHABLE")
            continue

        if got is None:
            problems.append(f"{note}: {why}")
            verdict = "NOT PUBLISHED"
        elif got != source:
            problems.append(f"{note}: source is rev {source} but the CDN "
                            f"serves rev {got}")
            verdict = "PUBLISHED IS BEHIND THE SOURCE"
        else:
            verdict = "current"
        print(f"  {note:<8}{source:>8}{got or '?':>11}   {verdict}{pdf_note}")

    print()
    if drafts:
        print(f"  drafts, nothing published: {', '.join(drafts)}")
    if problems:
        for p in problems:
            print(f"  FAIL {p}")
        print(f"{len(problems)} failure(s)")
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
