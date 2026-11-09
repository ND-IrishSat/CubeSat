import json
import zlib

from cloversat.datatypes.command_types import Mode
from cloversat.datatypes.state_types import PersistedState
from cloversat.devices.interfaces.storage_interface import StorageInterface
from cloversat.executive.persistence import Persistence, decode, encode, majority_vote


class FakeStorage(StorageInterface):
    def __init__(self):
        self.slots = {}
        self.fail = set()

    def init(self):
        pass

    def self_test(self):
        raise NotImplementedError

    def write_copy(self, i, blob):
        if i in self.fail:
            return False
        self.slots[i] = blob
        return True

    def read_copy(self, i):
        return self.slots.get(i)


def state(**kw):
    return PersistedState(mode=kw.pop("mode", Mode.OPERATIONAL), **kw)


def test_round_trip():
    s = state(deployed=True, deploy_attempts=2, t_saved=12.5, reset_count=3,
              last_reset_cause="watchdog", disabled_subsystems=["gps"], seq=7)
    assert decode(encode(s)) == s


def test_decode_rejects_corruption():
    blob = bytearray(encode(state()))
    blob[-3] ^= 0x01  # single bit flip in the payload
    assert decode(bytes(blob)) is None
    assert decode(b"") is None and decode(b"abc") is None


def _reblob(payload: dict) -> bytes:
    raw = json.dumps(payload).encode()
    return zlib.crc32(raw).to_bytes(4, "big") + raw


def test_decode_rejects_valid_crc_with_bad_values():
    good = json.loads(encode(state())[4:])
    assert decode(_reblob(good)) is not None
    assert decode(_reblob({**good, "v": 99})) is None           # unknown version
    assert decode(_reblob({**good, "mode": "nope"})) is None
    assert decode(_reblob({**good, "deployed": 1})) is None     # not a bool
    assert decode(_reblob({**good, "reset_count": -1})) is None
    assert decode(_reblob({**good, "deploy_attempts": True})) is None
    extra = {**good, "surprise": 1}
    assert decode(_reblob(extra)) is None                       # unknown field


def test_vote_prefers_newest_not_majority():
    new, old = state(deployed=True, seq=5), state(deployed=False, seq=4)
    # Reset mid-save: one new copy, two old ones. Old must not win.
    assert majority_vote([new, old, old]).deployed is True


def test_vote_skips_invalid_and_handles_none():
    s = state(seq=3)
    assert majority_vote([None, s, None]) == s
    assert majority_vote([None, None, None]) is None


def test_vote_same_seq_disagreement_uses_agreement():
    a, b = state(reset_count=1, seq=2), state(reset_count=9, seq=2)
    assert majority_vote([a, b, b]) == b
    assert majority_vote([a, b, None]) == a  # tie -> earliest


def test_save_then_load():
    p = Persistence(FakeStorage())
    assert p.save(state(reset_count=4)) == 3
    assert p.load().reset_count == 4


def test_seq_increments_each_save():
    p = Persistence(FakeStorage())
    p.save(state())
    p.save(state())
    assert p.load().seq == 2


def test_load_survives_one_corrupted_copy():
    st = FakeStorage()
    p = Persistence(st)
    p.save(state(deployed=True))
    st.slots[1] = b"garbage"
    assert p.load().deployed is True


def test_load_none_when_empty_or_all_corrupt():
    st = FakeStorage()
    assert Persistence(st).load() is None
    p = Persistence(st)
    p.save(state())
    for i in range(3):
        st.slots[i] = b"\x00" * 20
    assert p.load() is None


def test_partial_save_picks_newer_copy():
    st = FakeStorage()
    p = Persistence(st)
    p.save(state(deployed=False))
    st.fail = {1, 2}  # reset happened after only copy 0 was updated
    assert p.save(state(deployed=True)) == 1
    st.fail = set()
    assert Persistence(st).load().deployed is True


def test_save_continues_past_a_failed_slot():
    st = FakeStorage()
    st.fail = {0}
    assert Persistence(st).save(state()) == 2
    assert 1 in st.slots and 2 in st.slots


def test_fresh_instance_does_not_reuse_seq():
    st = FakeStorage()
    Persistence(st).save(state())
    Persistence(st).save(state())  # new object, never called load()
    assert Persistence(st).load().seq == 2
