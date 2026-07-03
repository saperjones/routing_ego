import os
from parking_proj import osm


def _fake_tile(z, x, y):
    # a minimal valid 1x1 PNG (bytes); content doesn't matter for the manifest
    return (b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
            b"\x08\x06\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\nIDATx\x9cc\x00"
            b"\x01\x00\x00\x05\x00\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82"
            + bytes([x & 255, y & 255]))


def test_fetch_writes_tiles_and_manifest(tmp_path):
    box = (117.135, 31.828, 117.150, 31.840)
    man = osm.fetch_basemap(*box, out_dir=str(tmp_path), max_tiles=25, tile_getter=_fake_tile)
    assert man is not None
    assert man["nx"] * man["ny"] == len(man["tiles"]) <= 25
    for t in man["tiles"]:
        assert os.path.exists(os.path.join(str(tmp_path), t["file"]))
    assert man["tile"] == 256


def test_fetch_returns_none_on_failure(tmp_path):
    box = (117.135, 31.828, 117.150, 31.840)
    man = osm.fetch_basemap(*box, out_dir=str(tmp_path), tile_getter=lambda z, x, y: None)
    assert man is None


def test_fetch_returns_none_when_all_tiles_identical(tmp_path):
    box = (117.135, 31.828, 117.150, 31.840)
    blocked = b"\x89PNG\r\n\x1a\nIDENTICAL-BLOCKED-TILE"
    man = osm.fetch_basemap(*box, out_dir=str(tmp_path), tile_getter=lambda z, x, y: blocked)
    assert man is None
