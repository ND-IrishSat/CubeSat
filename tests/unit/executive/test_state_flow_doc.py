"""docs/state_flow.md must draw exactly the transition tables in code."""
import importlib
import re
from pathlib import Path

import pytest

DOC = Path(__file__).resolve().parents[3] / "docs" / "state_flow.md"
BLOCK = re.compile(r"<!-- table: ([\w.]+) -->\s*```mermaid\n(.*?)```", re.S)
EDGE = re.compile(r"^\s*(\[\*\]|\w+)\s*-->\s*(\[\*\]|\w+)", re.M)

TABLES = [
    "cloversat.executive.state_manager.MODE_TRANSITIONS",
    "cloversat.executive.state_manager.SUBMODE_TRANSITIONS",
    "cloversat.executive.boot_mode.BOOT_STEP_TRANSITIONS",
    "cloversat.executive.safe_mode.SAFE_PHASE_TRANSITIONS",
]


def doc_blocks():
    return dict(BLOCK.findall(DOC.read_text()))


def resolve(path):
    module, name = path.rsplit(".", 1)
    return getattr(importlib.import_module(module), name)


def drawn_edges(body):
    # [*] is the diagram's start/end marker, not a state in the table.
    return {(a, b) for a, b in EDGE.findall(body) if "[*]" not in (a, b)}


def coded_edges(table):
    return {(a.name, b.name) for a, nxt in table.items() for b in nxt}


def test_every_table_has_a_diagram():
    assert set(doc_blocks()) == set(TABLES)


@pytest.mark.parametrize("path", TABLES)
def test_diagram_matches_table(path):
    drawn, coded = drawn_edges(doc_blocks()[path]), coded_edges(resolve(path))
    assert drawn == coded, f"only in doc: {drawn - coded}; only in code: {coded - drawn}"
