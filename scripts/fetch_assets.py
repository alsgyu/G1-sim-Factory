"""Download the matched official G1 assets, verifying immutable Git blobs.

The adjacent lock file is derived from the upstream Git tree at REV. No Git
history, training dependencies, unrelated robot models or GitHub token is needed.
Existing files are reused only after checksum verification; modified assets are
never silently overwritten. The generated manifest additionally records SHA256.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REV = "276801e46c5d433564f24658bac64f254b7d2d4b"
DEST = ROOT / "external/unitree_rl_gym"
LOCK = Path(__file__).with_name("unitree_assets.lock.json")
RAW_ROOT = f"https://raw.githubusercontent.com/unitreerobotics/unitree_rl_gym/{REV}"


def fingerprints(content: bytes) -> dict:
    header = f"blob {len(content)}\0".encode("ascii")
    return {"size": len(content),
            "git_blob_sha1": hashlib.sha1(header + content).hexdigest(),
            "sha256": hashlib.sha256(content).hexdigest()}


def verify(content: bytes, expected: dict, name: str) -> dict:
    actual = fingerprints(content)
    if any(actual[key] != expected[key] for key in ("size", "git_blob_sha1")):
        raise ValueError(f"Checksum mismatch: {name}; expected pinned official asset.")
    return actual


def fetch_one(name: str, expected: dict, destination: Path, timeout: float,
              retries: int, verify_only: bool) -> tuple[str, dict, str]:
    target = destination / name
    if target.exists():
        # A local change is a review decision, never an automatic overwrite.
        actual = verify(target.read_bytes(), expected, name)
        return name, actual, "verified"
    if verify_only:
        raise FileNotFoundError(f"Missing asset: {name}")
    request = urllib.request.Request(f"{RAW_ROOT}/{name}",
                                     headers={"User-Agent": "G1-sim-test-asset-fetcher"})
    for attempt in range(retries + 1):
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                content = response.read()
            actual = verify(content, expected, name)
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix=".download-",
                                             delete=False) as temporary:
                temporary_path = Path(temporary.name)
                temporary.write(content)
            try:
                if target.exists():
                    actual = verify(target.read_bytes(), expected, name)
                else:
                    os.replace(temporary_path, target)
            finally:
                temporary_path.unlink(missing_ok=True)
            return name, actual, "downloaded"
        except (OSError, urllib.error.URLError, ValueError) as error:
            if attempt == retries:
                raise RuntimeError(f"Asset failed after {attempt + 1} attempts: {name}: {error}") from error
            time.sleep(min(0.5 * 2 ** attempt, 8.0))
    raise AssertionError("unreachable")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEST)
    parser.add_argument("--workers", type=int, default=6, choices=range(1, 7))
    parser.add_argument("--timeout", type=float, default=45.0, help="Socket timeout in seconds")
    parser.add_argument("--retries", type=int, default=4)
    parser.add_argument("--verify-only", action="store_true", help="Verify installed assets without downloading")
    args = parser.parse_args()
    if args.timeout <= 0 or args.retries < 0:
        parser.error("timeout must be positive and retries nonnegative")
    lock = json.loads(LOCK.read_text())
    if lock["commit"] != REV:
        raise SystemExit("Asset lock revision differs from downloader revision")
    expected = lock["files"]
    destination = args.destination.resolve()
    records, errors = {}, []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        pending = [pool.submit(fetch_one, name, digest, destination, args.timeout,
                               args.retries, args.verify_only)
                   for name, digest in expected.items()]
        for future in as_completed(pending):
            try:
                name, actual, status = future.result()
                records[name] = {**actual, "url": f"{RAW_ROOT}/{name}"}
                print(f"[{len(records)}/{len(expected)}] {status}: {name}", flush=True)
            except Exception as error:
                errors.append(str(error))
    if errors:
        raise SystemExit("\n".join(errors) + "\nRerun to reuse verified downloads.")
    policy_name = "deploy/pre_train/g1/motion.pt"
    manifest = {"source": lock["source"], "commit": REV,
                "policy_sha256": records[policy_name]["sha256"],
                "files": dict(sorted(records.items()))}
    manifest_path = destination.parent / "assets_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Verified {len(records)} assets at {REV}; manifest: {manifest_path}")


if __name__ == "__main__":
    main()
