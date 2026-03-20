#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
README_PATH = ROOT / "README.md"
ROOT_CMAKE_PATH = ROOT / "CMakeLists.txt"
SRC_CMAKE_PATH = ROOT / "src" / "CMakeLists.txt"
JSD_TYPES_PATH = ROOT / "src" / "jsd_types.h"
SRC_DIR = ROOT / "src"

README_GROUP_ALIASES = {
    "ATI FORCE-TORQUE SENSOR": "ATI_FORCE_TORQUE_SENSOR",
    "ELMO GOLD DRIVES": "ELMO_GOLD_DRIVES",
    "ELMO PLATINUM DRIVES": "ELMO_PLATINUM_DRIVES",
    "JED (JPL ETHERCAT DEVICE)": "JED",
}

GROUP_DISPLAY_NAMES = {
    "ATI_FORCE_TORQUE_SENSOR": "ATI Force-Torque Sensor",
    "ELMO_GOLD_DRIVES": "Elmo Gold Drives",
    "ELMO_PLATINUM_DRIVES": "Elmo Platinum Drives",
    "JED": "JED (JPL EtherCat Device)",
}

DRIVER_GROUP_OVERRIDES = {
    "ATI_FTS": "ATI_FORCE_TORQUE_SENSOR",
    "EGD": "ELMO_GOLD_DRIVES",
    "EPD_NOMINAL": "ELMO_PLATINUM_DRIVES",
    "EPD_SIL": "ELMO_PLATINUM_DRIVES",
    "EL3202": "EL3202-0010",
    "JED0101": "JED",
    "JED0200": "JED",
}

GROUP_NOTES = {
    "EL3202-0010": (
        "The EL3202 implementation is annotated as EL3202-0010 in "
        "src/jsd_el3202_types.h."
    ),
    "ELMO_PLATINUM_DRIVES": (
        "Platinum support is implemented by the EPD nominal and EPD SIL drivers."
    ),
    "JED": "JED support is split across the JED0101 and JED0200 drivers.",
}

NON_DRIVER_CMAKE_SOURCES = {
    "COMMON_DEVICE_TYPES",
    "ELMO_COMMON",
    "EPD_COMMON",
    "ERROR_CIRQ",
    "SDO",
    "TIMER",
}


@dataclass(frozen=True)
class ReadmeEntry:
    device: str
    tier: str
    version: str
    group: str


@dataclass(frozen=True)
class DriverEvidence:
    driver_id: str
    group: str
    source_path: Path
    in_src_cmake: bool
    in_driver_enum: bool
    in_config_union: bool
    in_state_union: bool
    has_product_code_check: bool

    @property
    def is_supported(self) -> bool:
        return (
            self.in_src_cmake
            and self.in_driver_enum
            and self.in_config_union
            and self.in_state_union
            and self.has_product_code_check
        )


def normalize_readme_group(device_name: str) -> str:
    stripped = device_name.strip()
    return README_GROUP_ALIASES.get(stripped.upper(), stripped.upper())


def display_group_name(group: str) -> str:
    return GROUP_DISPLAY_NAMES.get(group, group)


def driver_to_group(driver_id: str) -> str:
    return DRIVER_GROUP_OVERRIDES.get(driver_id, driver_id)


def parse_readme_entries(readme_path: Path) -> list[ReadmeEntry]:
    lines = readme_path.read_text(encoding="utf-8").splitlines()
    in_section = False
    in_table = False
    entries: list[ReadmeEntry] = []

    for line in lines:
        if line.strip() == "# Supported Devices":
            in_section = True
            continue
        if in_section and line.startswith("# "):
            break
        if not in_section:
            continue
        if line.startswith("|"):
            in_table = True
            columns = [column.strip() for column in line.strip().strip("|").split("|")]
            if len(columns) != 3:
                continue
            if columns[0] == "Device" or set("".join(columns)) == {"-"}:
                continue
            device, tier, version = columns
            entries.append(
                ReadmeEntry(
                    device=device,
                    tier=tier,
                    version=version,
                    group=normalize_readme_group(device),
                )
            )
            continue
        if in_table and line.strip() == "":
            break

    if not entries:
        raise RuntimeError("Failed to parse the Supported Devices table from README.md")

    return entries


def parse_root_cmake_has_src(root_cmake_path: Path) -> bool:
    text = root_cmake_path.read_text(encoding="utf-8")
    return "add_subdirectory(src)" in text


def parse_src_cmake_driver_ids(src_cmake_path: Path) -> set[str]:
    text = src_cmake_path.read_text(encoding="utf-8")
    ids = {match.upper() for match in re.findall(r"\bjsd_([a-z0-9_]+)\.c\b", text)}
    return {driver_id for driver_id in ids if driver_id not in NON_DRIVER_CMAKE_SOURCES}


def _extract_typedef_block(text: str, typedef_name: str) -> str:
    pattern = re.compile(
        rf"typedef\s+(?:enum|struct)\s*\{{(?P<body>.*?)\}}\s*{re.escape(typedef_name)}\s*;",
        re.S,
    )
    match = pattern.search(text)
    if not match:
        raise RuntimeError(f"Failed to locate typedef block for {typedef_name}")
    return match.group("body")


def parse_jsd_types(jsd_types_path: Path) -> tuple[set[str], set[str], set[str]]:
    text = jsd_types_path.read_text(encoding="utf-8")

    enum_body = _extract_typedef_block(text, "jsd_driver_type_t")
    enum_ids = set(re.findall(r"JSD_DRIVER_TYPE_([A-Z0-9_]+)", enum_body))

    config_body = _extract_typedef_block(text, "jsd_slave_config_t")
    config_ids = {match.upper() for match in re.findall(r"jsd_([a-z0-9_]+)_config_t", config_body)}

    state_body = _extract_typedef_block(text, "jsd_slave_state_t")
    state_ids = {
        match.upper()
        for match in re.findall(r"jsd_([a-z0-9_]+?)(?:_private)?_state_t", state_body)
    }

    return enum_ids, config_ids, state_ids


def has_product_code_check(source_path: Path, driver_id: str) -> bool:
    text = source_path.read_text(encoding="utf-8")
    function_name = f"jsd_{driver_id.lower()}_product_code_is_compatible"
    return function_name in text


def collect_driver_evidence() -> tuple[bool, list[DriverEvidence]]:
    root_has_src = parse_root_cmake_has_src(ROOT_CMAKE_PATH)
    src_cmake_ids = parse_src_cmake_driver_ids(SRC_CMAKE_PATH)
    enum_ids, config_ids, state_ids = parse_jsd_types(JSD_TYPES_PATH)
    candidate_ids = sorted(src_cmake_ids | enum_ids)

    evidence: list[DriverEvidence] = []
    for driver_id in candidate_ids:
        source_path = SRC_DIR / f"jsd_{driver_id.lower()}.c"
        evidence.append(
            DriverEvidence(
                driver_id=driver_id,
                group=driver_to_group(driver_id),
                source_path=source_path,
                in_src_cmake=driver_id in src_cmake_ids,
                in_driver_enum=driver_id in enum_ids,
                in_config_union=driver_id in config_ids,
                in_state_union=driver_id in state_ids,
                has_product_code_check=source_path.exists()
                and has_product_code_check(source_path, driver_id),
            )
        )

    return root_has_src, evidence


def grouped_supported_drivers(evidence: list[DriverEvidence]) -> dict[str, list[DriverEvidence]]:
    grouped: dict[str, list[DriverEvidence]] = {}
    for item in evidence:
        if not item.is_supported:
            continue
        grouped.setdefault(item.group, []).append(item)
    return grouped


def format_driver_list(items: list[DriverEvidence]) -> str:
    driver_ids = ", ".join(sorted(item.driver_id for item in items))
    return driver_ids


def print_report(strict: bool) -> int:
    readme_entries = parse_readme_entries(README_PATH)
    readme_groups = {entry.group for entry in readme_entries}

    root_has_src, evidence = collect_driver_evidence()
    supported_groups = grouped_supported_drivers(evidence)
    supported_group_names = set(supported_groups)

    stale_readme_groups = sorted(readme_groups - supported_group_names)
    missing_readme_groups = sorted(supported_group_names - readme_groups)
    incomplete_drivers = [item for item in evidence if not item.is_supported]

    print("Supported device analysis")
    print(f"  root CMake wires src/: {'yes' if root_has_src else 'no'}")
    print(f"  README rows: {len(readme_entries)}")
    print(f"  code-supported groups: {len(supported_groups)}")
    print("")

    print("Code-supported devices")
    for group in sorted(supported_groups):
        print(
            f"  {display_group_name(group)}"
            f" [{format_driver_list(supported_groups[group])}]"
        )
    print("")

    print("README rows backed by code")
    for entry in readme_entries:
        status = "yes" if entry.group in supported_groups else "no"
        print(f"  {entry.device}: {status}")
    print("")

    print("README rows without code support")
    if stale_readme_groups:
        for group in stale_readme_groups:
            print(f"  {display_group_name(group)}")
    else:
        print("  none")
    print("")

    print("Code-supported devices missing from README")
    if missing_readme_groups:
        for group in missing_readme_groups:
            print(
                f"  {display_group_name(group)}"
                f" [{format_driver_list(supported_groups[group])}]"
            )
    else:
        print("  none")
    print("")

    print("Notes")
    noted_groups = sorted((supported_group_names | readme_groups) & GROUP_NOTES.keys())
    if noted_groups:
        for group in noted_groups:
            print(f"  {display_group_name(group)}: {GROUP_NOTES[group]}")
    else:
        print("  none")
    print("")

    if incomplete_drivers:
        print("Incomplete driver registrations")
        for item in incomplete_drivers:
            missing = []
            if not item.in_src_cmake:
                missing.append("src/CMakeLists.txt")
            if not item.in_driver_enum:
                missing.append("jsd_driver_type_t")
            if not item.in_config_union:
                missing.append("jsd_slave_config_t")
            if not item.in_state_union:
                missing.append("jsd_slave_state_t")
            if not item.has_product_code_check:
                missing.append("product_code_is_compatible")
            print(f"  {item.driver_id}: missing {', '.join(missing)}")
        print("")

    if strict and (stale_readme_groups or missing_readme_groups or incomplete_drivers):
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Cross-check the README Supported Devices table against build and type "
            "registration in the source tree."
        )
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return a non-zero exit code when the README is out of sync.",
    )
    args = parser.parse_args()
    return print_report(strict=args.strict)


if __name__ == "__main__":
    sys.exit(main())
