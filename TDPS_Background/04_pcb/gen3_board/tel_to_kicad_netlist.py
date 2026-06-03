#!/usr/bin/env python3
"""Convert a Telesis-format netlist (.tel, as exported by 立创EDA / LCEDA) into
a KiCad eeschema netlist (.net, S-expression) so it can be reviewed by KiCad
tooling or the `kicad` analysis skill.

Telesis structure:
    $PACKAGES
      FOOTPRINT ! FOOTPRINT ! [VALUE] ; refdes1 refdes2 ...
      (entries continue onto the next line when a line ends with ',')
    $NETS
      'NETNAME' ; REF.PIN REF.PIN ...
    $END

Usage:
    python tel_to_kicad_netlist.py "Netlist_Schematic5.tel" [-o out.net]
If -o is omitted, the output is written next to the input with a .net suffix.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def read_logical_entries(lines: list[str]) -> list[str]:
    """Join physical lines into logical entries. A line whose stripped text ends
    with ',' continues onto the next line; otherwise it terminates the entry."""
    entries: list[str] = []
    buf = ""
    for raw in lines:
        s = raw.strip()
        if not s:
            continue
        cont = s.endswith(",")
        if cont:
            s = s[:-1].strip()  # drop the trailing continuation comma
        buf = f"{buf} {s}".strip() if buf else s
        if not cont:
            if buf:
                entries.append(buf)
            buf = ""
    if buf:
        entries.append(buf)
    return entries


def split_sections(text: str) -> dict[str, list[str]]:
    """Split the file into its $-delimited sections."""
    sections: dict[str, list[str]] = {}
    current = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("$"):
            current = stripped.split()[0]
            sections.setdefault(current, [])
            continue
        if current is not None:
            sections[current].append(line)
    return sections


def _strip_value(tok: str) -> str:
    tok = tok.strip().strip("'").strip()
    # '{Value}' is a placeholder meaning "inherit the schematic value"; not a real value.
    return "" if tok in ("{Value}", "") else tok


def parse_packages(entries: list[str]) -> dict[str, dict[str, str]]:
    """refdes -> {footprint, value}."""
    comps: dict[str, dict[str, str]] = {}
    for entry in entries:
        if ";" not in entry or "!" not in entry:
            continue
        left, refs_part = entry.split(";", 1)
        fields = left.split("!")
        footprint = fields[0].strip()
        value = _strip_value(fields[2]) if len(fields) >= 3 else ""
        for ref in refs_part.split():
            comps[ref] = {"footprint": footprint, "value": value}
    return comps


def parse_nets(entries: list[str]) -> list[tuple[str, list[tuple[str, str]]]]:
    """[(netname, [(ref, pin), ...]), ...]."""
    nets: list[tuple[str, list[tuple[str, str]]]] = []
    for entry in entries:
        if ";" not in entry:
            continue
        name_part, pins_part = entry.split(";", 1)
        name_part = name_part.strip()
        if name_part.startswith("'"):
            end = name_part.find("'", 1)
            netname = name_part[1:end] if end > 0 else name_part.strip("'")
        else:
            netname = name_part.split()[0] if name_part.split() else name_part
        nodes: list[tuple[str, str]] = []
        for tok in pins_part.split():
            ref, _, pin = tok.rpartition(".")
            if ref and pin:
                nodes.append((ref, pin))
        if nodes:
            nets.append((netname, nodes))
    return nets


def _esc(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def build_kicad_netlist(comps: dict[str, dict[str, str]],
                        nets: list[tuple[str, list[tuple[str, str]]]],
                        source: str) -> str:
    out: list[str] = []
    out.append('(export (version "E")')
    out.append("  (design")
    out.append(f'    (source "{_esc(source)}")')
    out.append(f'    (date "{time.strftime("%Y-%m-%d %H:%M:%S")}")')
    out.append('    (tool "tel_to_kicad_netlist.py (Telesis .tel -> KiCad)"))')

    # Components referenced anywhere (packages + any ref that appears only in nets).
    refs_in_nets = {ref for _, nodes in nets for ref, _ in nodes}
    all_refs = sorted(set(comps) | refs_in_nets)

    out.append("  (components")
    for ref in all_refs:
        info = comps.get(ref, {"footprint": "", "value": ""})
        value = info["value"] or "~"
        line = f'    (comp (ref "{_esc(ref)}") (value "{_esc(value)}")'
        if info["footprint"]:
            line += f' (footprint "{_esc(info["footprint"])}")'
        line += ")"
        out.append(line)
    out.append("  )")

    out.append("  (nets")
    for code, (netname, nodes) in enumerate(nets, start=1):
        out.append(f'    (net (code "{code}") (name "{_esc(netname)}")')
        for ref, pin in nodes:
            out.append(f'      (node (ref "{_esc(ref)}") (pin "{_esc(pin)}"))')
        out.append("    )")
    out.append("  )")
    out.append(")")
    return "\n".join(out) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description="Convert a Telesis .tel netlist to a KiCad .net netlist")
    ap.add_argument("tel", help="input .tel file")
    ap.add_argument("-o", "--out", default=None, help="output .net path (default: alongside input)")
    args = ap.parse_args()

    tel_path = Path(args.tel)
    if not tel_path.exists():
        print(f"[ERROR] not found: {tel_path}", file=sys.stderr)
        sys.exit(1)

    text = tel_path.read_text(encoding="utf-8", errors="replace")
    sections = split_sections(text)
    comps = parse_packages(read_logical_entries(sections.get("$PACKAGES", [])))
    nets = parse_nets(read_logical_entries(sections.get("$NETS", [])))

    netlist = build_kicad_netlist(comps, nets, tel_path.name)
    out_path = Path(args.out) if args.out else tel_path.with_suffix(".net")
    out_path.write_text(netlist, encoding="utf-8")

    total_nodes = sum(len(nodes) for _, nodes in nets)
    print(f"[OK] components: {len(set(comps) | {r for _, ns in nets for r, _ in ns})}")
    print(f"[OK] nets: {len(nets)}  (pin connections: {total_nodes})")
    print(f"[OK] written: {out_path}")


if __name__ == "__main__":
    main()
