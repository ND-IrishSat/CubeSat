"""Persistence of ``PersistedState``.

Purpose:  Continuous triple-copy saves of ``PersistedState``; on load, a corrupt
          copy is detected by its checksum and the newest valid copy wins.
Inputs:   ``PersistedState``, ``StorageInterface``.
Outputs:  Saved copies; voted ``PersistedState`` on load.
Status:   [NEW]
Port from: none

Atomic replacement of each copy is the storage driver's job. This module only
serializes, checksums and chooses between copies.

A copy is ``CRC32 (4 bytes, big-endian) + JSON``. The version field in the JSON
lets the format change later; an unknown version reads as invalid.

Why "newest valid" and not a plain majority: the copies are written one after
another, so a reset mid-save can leave copy 0 new and copies 1 and 2 old. A
plain 2-of-3 vote would pick the old state, and if the new one was
``deployed=True`` that could fire the deployer a second time. The CRC already
catches corruption, so recency is the better tie-breaker. Agreement between
copies only matters when two valid copies share a ``seq`` but differ.
"""
from __future__ import annotations

import json
import zlib
from collections import Counter
from dataclasses import asdict, replace
from typing import TYPE_CHECKING, Optional

from cloversat.datatypes.state_types import PersistedState

if TYPE_CHECKING:
    from cloversat.devices.interfaces.storage_interface import StorageInterface

FORMAT_VERSION = 2
_CRC_BYTES = 4


def encode(state: PersistedState) -> bytes:
    """Serialize with an integrity check (CRC32 of the JSON payload)."""
    payload = json.dumps({"v": FORMAT_VERSION, **asdict(state)}, sort_keys=True, separators=(",", ":")).encode()
    return zlib.crc32(payload).to_bytes(_CRC_BYTES, "big") + payload


def decode(blob: bytes) -> Optional[PersistedState]:
    """Deserialize; None if the integrity check or any field check fails."""
    if len(blob) <= _CRC_BYTES:
        return None
    crc, payload = blob[:_CRC_BYTES], blob[_CRC_BYTES:]
    if zlib.crc32(payload).to_bytes(_CRC_BYTES, "big") != crc:
        return None
    try:
        d = json.loads(payload)
        if d.pop("v") != FORMAT_VERSION:
            return None
        state = PersistedState(**d)
    except (ValueError, KeyError, TypeError):
        return None
    if not _fields_ok(state):
        return None
    return state


def _fields_ok(s: PersistedState) -> bool:
    """A passing CRC doesn't make the values sane (e.g. a writer bug)."""
    # bool is a subclass of int, so it is excluded explicitly.
    counts = (s.deploy_attempts, s.seq)
    return (
        isinstance(s.deployed, bool)
        and all(isinstance(n, int) and not isinstance(n, bool) and n >= 0 for n in counts)
        and _is_number(s.t_saved)
        and isinstance(s.recent_reset_times, list)
        and all(_is_number(x) for x in s.recent_reset_times)
        and isinstance(s.disabled_subsystems, list)
        and all(isinstance(x, str) for x in s.disabled_subsystems)
    )


def _is_number(x: object) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def majority_vote(copies: list[Optional[PersistedState]]) -> Optional[PersistedState]:
    """Pick the state to trust from the copies that decoded (None = invalid).

    Takes the highest ``seq``. If valid copies at that ``seq`` disagree, which
    should not happen, the content most copies agree on wins; a tie goes to the
    earliest copy. Returns None if no copy is valid.
    """
    valid = [c for c in copies if c is not None]
    if not valid:
        return None
    newest = max(c.seq for c in valid)
    candidates = [c for c in valid if c.seq == newest]
    # Dataclasses with list fields aren't hashable, so count by encoded bytes.
    votes = Counter(encode(c) for c in candidates)
    best = max(votes.values())
    return next(c for c in candidates if votes[encode(c)] == best)


class Persistence:
    """Triple-copy save/load over a ``StorageInterface``."""

    N_COPIES = 3

    def __init__(self, storage: StorageInterface) -> None:
        self._storage = storage
        # Learned from disk on first use, so a save before any load can't
        # reuse a seq that is already on disk.
        self._seq: Optional[int] = None

    def save(self, state: PersistedState) -> int:
        """Write all copies; return how many were written successfully.

        A return below ``N_COPIES`` means redundancy is reduced. The caller
        decides whether to log it or raise a fault; 0 means nothing was saved.
        """
        if self._seq is None:
            loaded = self.load()
            self._seq = loaded.seq if loaded is not None else 0
        self._seq += 1
        blob = encode(replace(state, seq=self._seq))
        # Try every copy even if one fails, so one bad slot doesn't cost the rest.
        return sum(self._storage.write_copy(i, blob) for i in range(self.N_COPIES))

    def load(self) -> Optional[PersistedState]:
        """Read all copies and vote. None means no valid copy exists.

        None must be treated as "unknown", not "defaults": in particular
        ``deployed`` has to be re-read from the sensor before any burn.
        """
        decoded = []
        for i in range(self.N_COPIES):
            blob = self._storage.read_copy(i)
            decoded.append(decode(blob) if blob is not None else None)
        winner = majority_vote(decoded)
        if winner is not None:
            self._seq = winner.seq
        return winner
