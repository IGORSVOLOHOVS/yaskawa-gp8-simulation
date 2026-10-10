#!/usr/bin/env python3
"""Fetch the external assets this repository deliberately does not carry in git.

The repository keeps no large vendor binaries under version control. Everything
listed in ``assets/external_assets_manifest.json`` is fetched from its own
publisher at ``./robot init`` time into ``assets/downloads/`` (git-ignored) and
checked against a recorded SHA-256 digest.

Standard library only: urllib, hashlib, json, argparse.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import urllib.error
import urllib.request

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MANIFEST = os.path.join(REPO_ROOT, "assets", "external_assets_manifest.json")
USER_AGENT = "waam-manipulator-asset-fetcher/1.0 (urllib; +./robot init)"
CHUNK = 256 * 1024

STATUS_CURRENT = "current"
STATUS_FETCHED = "fetched"
STATUS_SKIPPED = "skipped"
STATUS_FAILED = "failed"


# --------------------------------------------------------------------------- #
# Manifest handling
# --------------------------------------------------------------------------- #
def load_manifest(path):
    """Read the manifest and return (raw document, flattened asset list)."""
    with open(path, "r", encoding="utf-8") as handle:
        document = json.load(handle)

    assets = []
    for group in document.get("groups", []):
        for asset in group.get("assets", []):
            asset = dict(asset)
            asset["group"] = group.get("name", "ungrouped")
            assets.append(asset)

    seen = set()
    for asset in assets:
        if asset["id"] in seen:
            raise ValueError("duplicate asset id in manifest: %s" % asset["id"])
        seen.add(asset["id"])
    return document, assets


def download_root(document, manifest_path):
    """Absolute directory the manifest's relative paths are resolved against."""
    relative = document.get("download_dir", "assets/downloads")
    base = os.path.dirname(os.path.dirname(os.path.abspath(manifest_path)))
    return os.path.join(base, *relative.split("/"))


def local_path(root, asset):
    return os.path.join(root, *asset["path"].split("/"))


def human_bytes(count):
    if count is None:
        return "?"
    value = float(count)
    for unit in ("B", "KiB", "MiB", "GiB"):
        if value < 1024.0 or unit == "GiB":
            if unit == "B":
                return "%d B" % int(value)
            return "%.1f %s" % (value, unit)
        value /= 1024.0
    return "%.1f GiB" % value


def sha256_of_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(CHUNK), b""):
            digest.update(block)
    return digest.hexdigest()


# --------------------------------------------------------------------------- #
# Network
# --------------------------------------------------------------------------- #
class NetworkFailure(Exception):
    """The URL could not be read at all (DNS, TLS, timeout, HTTP status)."""


class DigestMismatch(Exception):
    """The bytes arrived but are not the bytes the manifest describes."""


def fetch_to_file(url, destination, timeout):
    """Stream `url` into `destination` via a temporary file. Returns (sha256, bytes).

    The temporary file is renamed over the destination only after the whole body
    has been read, so an interrupted run never leaves a half-written asset.
    """
    os.makedirs(os.path.dirname(destination), exist_ok=True)
    partial = destination + ".part"
    digest = hashlib.sha256()
    total = 0
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            with open(partial, "wb") as handle:
                while True:
                    block = response.read(CHUNK)
                    if not block:
                        break
                    handle.write(block)
                    digest.update(block)
                    total += len(block)
    except (urllib.error.URLError, urllib.error.HTTPError, OSError) as error:
        if os.path.exists(partial):
            os.remove(partial)
        raise NetworkFailure(str(error)) from error

    os.replace(partial, destination)
    return digest.hexdigest(), total


# --------------------------------------------------------------------------- #
# Per-asset actions
# --------------------------------------------------------------------------- #
def verify_on_disk(root, asset):
    """Return (ok, detail) for the copy already on disk."""
    path = local_path(root, asset)
    if not os.path.exists(path):
        return False, "not downloaded"
    actual_bytes = os.path.getsize(path)
    expected = asset.get("sha256") or ""
    if not expected:
        return False, "no sha256 in manifest (run --update-digests)"
    actual = sha256_of_file(path)
    if actual != expected:
        return False, "sha256 mismatch: expected %s, found %s" % (expected, actual)
    if asset.get("bytes") and actual_bytes != asset["bytes"]:
        return False, "size mismatch: expected %d, found %d" % (asset["bytes"], actual_bytes)
    return True, "%s on disk" % human_bytes(actual_bytes)


def fetch_asset(root, asset, timeout, force):
    """Fetch one asset. Returns (status, detail). Raises nothing but bookkeeping."""
    path = local_path(root, asset)
    if not force:
        ok, detail = verify_on_disk(root, asset)
        if ok:
            return STATUS_CURRENT, detail

    actual_sha, actual_bytes = fetch_to_file(asset["url"], path, timeout)
    expected = asset.get("sha256") or ""
    if expected and actual_sha != expected:
        os.remove(path)
        raise DigestMismatch(
            "expected sha256 %s but the server served %s (%d bytes); "
            "the bad file was deleted" % (expected, actual_sha, actual_bytes)
        )
    return STATUS_FETCHED, human_bytes(actual_bytes)


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #
def command_list(document, assets, root):
    id_width = max([len(a["id"]) for a in assets] + [8])
    total = 0
    print("Manifest: %s" % document.get("description", "external assets"))
    print("Download directory: %s" % document.get("download_dir", "assets/downloads"))
    print("")
    current_group = None
    for asset in assets:
        if asset["group"] != current_group:
            current_group = asset["group"]
            print("[%s]" % current_group)
        present = "yes" if os.path.exists(local_path(root, asset)) else "no"
        total += asset.get("bytes") or 0
        print(
            "  %-*s  %-8s  %10s  on disk: %-3s  %s"
            % (
                id_width,
                asset["id"],
                asset.get("kind", "?"),
                human_bytes(asset.get("bytes")),
                present,
                asset["path"],
            )
        )
    print("")
    print("%d assets, %s to download in total." % (len(assets), human_bytes(total)))
    return 0


def command_check(assets, root):
    bad = 0
    on_disk = 0
    id_width = max(len(a["id"]) for a in assets) + 1
    for asset in assets:
        ok, detail = verify_on_disk(root, asset)
        path = local_path(root, asset)
        if os.path.exists(path):
            on_disk += os.path.getsize(path)
        if ok:
            print("  ok      %-*s %s" % (id_width, asset["id"], detail))
        else:
            bad += 1
            print("  FAILED  %-*s %s" % (id_width, asset["id"], detail))
    print("")
    print(
        "Summary: %d current, 0 fetched, 0 skipped, %d failed, %s on disk."
        % (len(assets) - bad, bad, human_bytes(on_disk))
    )
    if bad:
        print("Run 'python tools/fetch_external_assets.py' to fetch the missing assets.")
        return 1
    print("Every external asset is current.")
    return 0


def command_fetch(assets, root, timeout, offline_ok, force):
    counts = {STATUS_CURRENT: 0, STATUS_FETCHED: 0, STATUS_SKIPPED: 0, STATUS_FAILED: 0}
    network_failures = 0
    digest_failures = 0
    width = len(str(len(assets)))
    id_width = max(len(a["id"]) for a in assets) + 1

    for index, asset in enumerate(assets, start=1):
        prefix = "[%*d/%d]" % (width, index, len(assets))
        try:
            status, detail = fetch_asset(root, asset, timeout, force)
        except NetworkFailure as error:
            counts[STATUS_FAILED] += 1
            network_failures += 1
            level = "WARNING" if offline_ok else "ERROR"
            print("%s %-*s %s network: %s" % (prefix, id_width, asset["id"], level, error))
            continue
        except DigestMismatch as error:
            counts[STATUS_FAILED] += 1
            digest_failures += 1
            level = "WARNING" if offline_ok else "ERROR"
            print("%s %-*s %s digest: %s" % (prefix, id_width, asset["id"], level, error))
            continue
        counts[status] += 1
        label = "up to date" if status == STATUS_CURRENT else status
        print("%s %-*s %-11s %s" % (prefix, id_width, asset["id"], label, detail))

    on_disk = sum(
        os.path.getsize(local_path(root, a))
        for a in assets
        if os.path.exists(local_path(root, a))
    )
    print("")
    print(
        "Summary: %d current, %d fetched, %d skipped, %d failed, %s on disk."
        % (
            counts[STATUS_CURRENT],
            counts[STATUS_FETCHED],
            counts[STATUS_SKIPPED],
            counts[STATUS_FAILED],
            human_bytes(on_disk),
        )
    )
    if counts[STATUS_FAILED] == 0:
        if counts[STATUS_FETCHED] == 0:
            print("Everything was already present: up to date, nothing downloaded.")
        return 0
    if digest_failures:
        print(
            "%d asset(s) did not match the manifest digest. If the publisher "
            "republished the file, refresh the manifest with --update-digests "
            "and review the change." % digest_failures
        )
    if network_failures:
        print(
            "%d asset(s) could not be reached. The build does not need them; "
            "re-run 'python tools/fetch_external_assets.py' when a network is "
            "available." % network_failures
        )
    if offline_ok:
        print("--offline-ok given: reporting the above as warnings, exit code 0.")
        return 0
    return 1


def command_update_digests(document, assets, root, manifest_path, timeout):
    """Re-download every selected asset and write the served digest/size back."""
    measured = {}
    failed = 0
    width = len(str(len(assets)))
    id_width = max(len(a["id"]) for a in assets) + 1
    for index, asset in enumerate(assets, start=1):
        prefix = "[%*d/%d]" % (width, index, len(assets))
        try:
            sha, size = fetch_to_file(asset["url"], local_path(root, asset), timeout)
        except NetworkFailure as error:
            failed += 1
            print("%s %-*s ERROR network: %s" % (prefix, id_width, asset["id"], error))
            continue
        measured[asset["id"]] = (sha, size)
        changed = ""
        if asset.get("sha256") and asset["sha256"] != sha:
            changed = "  (digest CHANGED)"
        print(
            "%s %-*s %s %10s%s"
            % (prefix, id_width, asset["id"], sha, human_bytes(size), changed)
        )

    for group in document.get("groups", []):
        for asset in group.get("assets", []):
            if asset["id"] in measured:
                asset["sha256"], asset["bytes"] = measured[asset["id"]]

    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, ensure_ascii=False)
        handle.write("\n")

    print("")
    print(
        "Summary: 0 current, %d fetched, 0 skipped, %d failed, %s on disk."
        % (
            len(measured),
            failed,
            human_bytes(
                sum(
                    os.path.getsize(local_path(root, a))
                    for a in assets
                    if os.path.exists(local_path(root, a))
                )
            ),
        )
    )
    print("Wrote %d digest(s) to %s" % (len(measured), manifest_path))
    return 1 if failed else 0


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def build_parser():
    parser = argparse.ArgumentParser(
        prog="fetch_external_assets.py",
        description=(
            "Fetch the Yaskawa documents and the upstream ROS-Industrial robot "
            "model listed in assets/external_assets_manifest.json. Nothing it "
            "downloads is committed to git."
        ),
    )
    parser.add_argument(
        "--manifest",
        default=DEFAULT_MANIFEST,
        help="manifest to read (default: assets/external_assets_manifest.json)",
    )
    parser.add_argument("--list", action="store_true", help="print the manifest and exit")
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify what is on disk, download nothing, exit 1 if anything is missing",
    )
    parser.add_argument("--only", metavar="ID", help="act on a single asset id")
    parser.add_argument(
        "--offline-ok",
        action="store_true",
        help="report fetch failures as warnings and still exit 0 (used by ./robot init)",
    )
    parser.add_argument(
        "--update-digests",
        action="store_true",
        help="re-download every asset and write the served sha256/bytes into the manifest",
    )
    parser.add_argument(
        "--timeout", type=float, default=60.0, help="per-request timeout in seconds (default: 60)"
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    try:
        document, assets = load_manifest(args.manifest)
    except (OSError, ValueError) as error:
        print("ERROR: cannot read manifest %s: %s" % (args.manifest, error))
        return 1

    if args.only:
        selected = [a for a in assets if a["id"] == args.only]
        if not selected:
            print("ERROR: no asset with id '%s'. Use --list to see the ids." % args.only)
            return 1
        assets = selected

    root = download_root(document, args.manifest)

    if args.list:
        return command_list(document, assets, root)

    print(
        "--> External assets: %d selected, destination %s"
        % (len(assets), root)
    )

    if args.check:
        return command_check(assets, root)
    if args.update_digests:
        return command_update_digests(document, assets, root, args.manifest, args.timeout)
    return command_fetch(assets, root, args.timeout, args.offline_ok, force=False)


if __name__ == "__main__":
    sys.exit(main())
