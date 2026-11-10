#!/usr/bin/env python3
"""Download a CC0 texture pool for randomized simulation scenes.

Textures come from Poly Haven (all CC0).  Each image is centre-cropped to a
square, resized to 512x512 and rejected if it is degenerate for visual odometry:

* almost uniform (very low entropy / contrast): no features to track;
* near white noise (very high entropy): no repeatable structure.

The filter keeps natural indoor materials (wood, plaster, fabric, tile, ...)
which is what the real rooms look like.  ``assets/`` is gitignored; the
manifest and license notes are written under ``configs/sim/`` and ``assets/``
so the pool can be rebuilt deterministically.
"""

from __future__ import annotations

import argparse
import io
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
API_URL = "https://api.polyhaven.com/assets?t=textures"
DOWNLOAD_URL = "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/{asset}/{asset}_diff_1k.jpg"
USER_AGENT = "flyrat-texture-pool/1.0 (+research dataset generation)"

# Outdoor / sky / ground textures do not look like an indoor flight arena.
SKIP_CATEGORIES = {
    "aerial",
    "grass",
    "sky",
    "terrain",
    "water",
    "snow",
    "rock",
    "sand",
    "ground",
    "road",
}
PREFERRED_CATEGORIES = {
    "wood",
    "fabric",
    "plaster",
    "concrete",
    "tiles",
    "bricks",
    "marble",
    "metal",
    "wallpaper",
    "carpet",
    "paint",
    "ceiling",
    "floor",
    "indoor",
}


def _fetch(url: str, timeout: float = 30.0) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def _entropy(gray: np.ndarray) -> float:
    histogram = np.bincount(gray.reshape(-1), minlength=256).astype(np.float64)
    probability = histogram / max(1.0, histogram.sum())
    nonzero = probability[probability > 0.0]
    return float(-(nonzero * np.log2(nonzero)).sum())


def _gradient_energy(gray: np.ndarray) -> float:
    dx = np.diff(gray.astype(np.float32), axis=1)
    dy = np.diff(gray.astype(np.float32), axis=0)
    return float(0.5 * (np.abs(dx).mean() + np.abs(dy).mean()))


def _prepare(raw: bytes, size: int) -> tuple[np.ndarray, dict[str, float]] | None:
    image = Image.open(io.BytesIO(raw)).convert("RGB")
    width, height = image.size
    side = min(width, height)
    left = (width - side) // 2
    top = (height - side) // 2
    image = image.crop((left, top, left + side, top + side)).resize(
        (size, size), Image.LANCZOS
    )
    array = np.asarray(image, dtype=np.uint8)
    gray = np.asarray(image.convert("L"), dtype=np.uint8)
    entropy = _entropy(gray)
    gradient = _gradient_energy(gray)
    contrast = float(gray.std())
    if entropy < 3.5 or entropy > 7.5 or gradient < 2.0 or contrast < 8.0:
        return None
    return array, {
        "entropy_bits": round(entropy, 3),
        "gradient_energy": round(gradient, 3),
        "contrast_std": round(contrast, 3),
    }


def _select_assets(assets: dict, count: int) -> list[tuple[str, dict]]:
    candidates: list[tuple[float, str, dict]] = []
    for asset_id, asset in assets.items():
        categories = {str(item).lower() for item in asset.get("categories", [])}
        if categories & SKIP_CATEGORIES:
            continue
        score = 1.0 + 2.0 * len(categories & PREFERRED_CATEGORIES)
        if "indoor" in categories:
            score += 2.0
        candidates.append((score, asset_id, asset))
    candidates.sort(key=lambda item: (-item[0], item[1]))

    # Round-robin over categories so the pool is not dominated by one material.
    buckets: dict[str, list[tuple[str, dict]]] = {}
    for _, asset_id, asset in candidates:
        key = sorted({str(c).lower() for c in asset.get("categories", [])})[0]
        buckets.setdefault(key, []).append((asset_id, asset))
    selected: list[tuple[str, dict]] = []
    keys = sorted(buckets, key=lambda name: -len(buckets[name]))
    while len(selected) < count and any(buckets[key] for key in keys):
        for key in keys:
            if buckets[key]:
                selected.append(buckets[key].pop(0))
                if len(selected) >= count:
                    break
    return selected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--size", type=int, default=512)
    parser.add_argument("--out", default=str(ROOT / "assets/textures"))
    parser.add_argument(
        "--manifest", default=str(ROOT / "configs/sim/textures_manifest.json")
    )
    parser.add_argument("--cache", default="/tmp/opencode/polyhaven_assets.json")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    cache = Path(args.cache)
    if cache.exists():
        assets = json.loads(cache.read_text(encoding="utf-8"))
        print(f"assets: {len(assets)} from cache {cache}")
    else:
        print(f"fetching {API_URL} ...")
        assets = json.loads(_fetch(API_URL))
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(assets), encoding="utf-8")
        print(f"assets: {len(assets)} (cached to {cache})")

    selected = _select_assets(assets, max(1, args.count))
    print(f"selected {len(selected)} candidate textures")

    entries: list[dict] = []
    started = time.time()
    for index, (asset_id, asset) in enumerate(selected, start=1):
        target = out_dir / f"{asset_id}.jpg"
        if target.exists():
            print(f"[{index}/{len(selected)}] {asset_id}: cached")
            entries.append({"id": asset_id, "file": target.name, "cached": True})
            continue
        url = DOWNLOAD_URL.format(asset=asset_id)
        try:
            raw = _fetch(url, timeout=60.0)
        except (urllib.error.URLError, TimeoutError) as error:
            print(f"[{index}/{len(selected)}] {asset_id}: download failed ({error})")
            continue
        prepared = _prepare(raw, args.size)
        if prepared is None:
            print(f"[{index}/{len(selected)}] {asset_id}: rejected by filter")
            continue
        array, metrics = prepared
        Image.fromarray(array, mode="RGB").save(target, format="JPEG", quality=92)
        entries.append(
            {
                "id": asset_id,
                "file": target.name,
                "name": asset.get("name", asset_id),
                "categories": sorted(str(c).lower() for c in asset.get("categories", [])),
                "license": "CC0",
                "source": f"https://polyhaven.com/a/{asset_id}",
                "url": url,
                **metrics,
            }
        )
        print(
            f"[{index}/{len(selected)}] {asset_id}: entropy {metrics['entropy_bits']:.2f} "
            f"gradient {metrics['gradient_energy']:.1f} contrast {metrics['contrast_std']:.1f}"
        )

    manifest = Path(args.manifest)
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(
        json.dumps(
            {
                "source": "Poly Haven (CC0)",
                "license": "CC0 1.0 Universal",
                "size_px": args.size,
                "count": len(entries),
                "textures": entries,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    elapsed = time.time() - started
    print(f"done: {len(entries)} textures in {elapsed:.1f}s -> {out_dir}")
    print(f"manifest: {manifest}")
    return 0 if entries else 1


if __name__ == "__main__":
    sys.exit(main())
