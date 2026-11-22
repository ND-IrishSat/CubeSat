from cloversat.devices.hardware.vn100_driver import _checksum, _frame, _unframe


def test_checksum_matches_manual_example():
    # VN-100 user manual example: $VNRRG,8*4B
    assert _checksum("VNRRG,8") == "4B"


def test_frame_roundtrip():
    framed = _frame("VNRRG,8")
    assert framed == b"$VNRRG,8*4B\r\n"
    assert _unframe(framed) == "VNRRG,8"


def test_unframe_rejects_bad_checksum():
    assert _unframe(b"$VNRRG,8*00\r\n") is None


def test_unframe_rejects_partial_line():
    assert _unframe(b"RRG,8*4B") is None