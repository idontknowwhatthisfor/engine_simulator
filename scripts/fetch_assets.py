#!/usr/bin/env python3
"""Download the large assets listed in assets.json from a GitHub release (stdlib only).

    python3 scripts/fetch_assets.py          # fetch missing/corrupt files
    python3 scripts/fetch_assets.py --force  # re-download everything

Public repo: works with no setup. Private repo: set GITHUB_TOKEN (or GH_TOKEN) to a token
that can read the repo's contents, e.g. `export GITHUB_TOKEN=$(gh auth token)`.
"""
import hashlib, json, os, sys, urllib.error, urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
API = "https://api.github.com"
TOKEN = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")


class DropAuthOnRedirect(urllib.request.HTTPRedirectHandler):
    """Release downloads redirect to a signed storage URL, which rejects our token."""
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        new = super().redirect_request(req, fp, code, msg, headers, newurl)
        if new is not None:
            new.remove_header("Authorization")
        return new


opener = urllib.request.build_opener(DropAuthOnRedirect)


def request(url, accept):
    headers = {"Accept": accept, "User-Agent": "fetch-assets"}
    if TOKEN:
        headers["Authorization"] = f"Bearer {TOKEN}"
    return opener.open(urllib.request.Request(url, headers=headers), timeout=60)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(repo, tag, asset, tmp):
    if TOKEN:  # works for private repos
        with request(f"{API}/repos/{repo}/releases/tags/{tag}", "application/vnd.github+json") as r:
            assets = json.load(r)["assets"]
        match = next((a for a in assets if a["name"] == asset), None)
        if match is None:
            raise OSError(f"asset '{asset}' not found on release '{tag}'")
        url, accept = match["url"], "application/octet-stream"
    else:
        url, accept = f"https://github.com/{repo}/releases/download/{tag}/{asset}", "*/*"
    with request(url, accept) as r, open(tmp, "wb") as out:
        while chunk := r.read(1 << 20):
            out.write(chunk)


def main():
    force = "--force" in sys.argv
    with open(os.path.join(ROOT, "assets.json")) as f:
        manifest = json.load(f)
    repo, tag = manifest["repo"], manifest["tag"]
    failed = False
    for a in manifest["files"]:
        dest = os.path.join(ROOT, a["path"])
        if not force and os.path.exists(dest) and sha256(dest) == a["sha256"]:
            print(f"ok       {a['path']}")
            continue
        print(f"fetching {a['path']} from {repo}@{tag} ...")
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        tmp = dest + ".part"
        try:
            download(repo, tag, a["asset"], tmp)
        except urllib.error.HTTPError as e:
            hint = "" if TOKEN else " (private repo? set GITHUB_TOKEN)" if e.code == 404 else ""
            print(f"error    {a['path']}: HTTP {e.code}{hint}")
            failed = True
            continue
        except (urllib.error.URLError, OSError) as e:
            print(f"error    {a['path']}: {e}")
            failed = True
            continue
        if sha256(tmp) != a["sha256"]:
            os.remove(tmp)
            print(f"error    {a['path']}: checksum mismatch")
            failed = True
            continue
        os.replace(tmp, dest)
        print(f"done     {a['path']}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
