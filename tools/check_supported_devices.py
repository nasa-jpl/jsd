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
COMMON_DEVICE_TYPES_PATH = ROOT / "src" / "jsd_common_device_types.h"
EPD_COMMON_TYPES_PATH = ROOT / "src" / "jsd_epd_common_types.h"
SRC_DIR = ROOT / "src"
DOCS_DIR = ROOT / "docs"

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

DEVICE_CATALOG_METADATA = {
    "ATI_FTS": {
        "full_name": "ATI Force-Torque Sensor",
        "manufacturer": "ATI Industrial Automation",
        "vendor_macro": "JSD_ATI_VENDOR_ID",
        "product_macros": ["JSD_ATI_FTS_PRODUCT_CODE"],
        "channel_summary": "6-axis wrench (Fx, Fy, Fz, Tx, Ty, Tz)",
        "io_kind": "force-torque sensing",
        "direction": "input",
        "details": "Calibration-selectable force/torque sensor with integrated signal conditioning, status code, and sample counter.",
        "accepted_voltage": "20-48 V DC, or IEEE 802.3af Mode A PoE",
        "source_url": "https://www.ati-ia.com/app_content/documents/9620-05-EtherCAT.pdf",
    },
    "EGD": {
        "full_name": "Elmo Gold Drive",
        "manufacturer": "Elmo Motion Control",
        "vendor_macro": "JSD_ELMO_VENDOR_ID",
        "product_macros": ["JSD_EGD_PRODUCT_CODE"],
        "channel_macros": ["JSD_EGD_NUM_DIGITAL_INPUTS", "JSD_EGD_NUM_DIGITAL_OUTPUTS"],
        "io_kind": "servo drive and discrete I/O",
        "direction": "bidirectional",
        "details": "Gold Line EtherCAT servo-drive family with DS-402 motion modes, analog input, and digital I/O.",
        "accepted_voltage": "Varies by Gold model; official Gold pages show 12-95 VDC low-voltage variants and 23-195 VDC higher-voltage variants.",
        "source_url": "https://www.elmomc.com/product/gold-cello/",
    },
    "EL1008": {
        "full_name": "Beckhoff EL1008",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL1008_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL1008_NUM_CHANNELS"],
        "io_kind": "digital input",
        "direction": "input",
        "details": "8-channel digital input terminal.",
        "accepted_voltage": "24 V DC nominal (-15%/+20%); logic 0: -3 to +5 V, logic 1: 11 to 30 V.",
        "source_url": "https://www.beckhoff.com/de-de/produkte/i-o/ethercat-klemmen/el-ed1xxx-digital-eingang/el1008.html",
    },
    "EL2124": {
        "full_name": "Beckhoff EL2124",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL2124_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL2124_NUM_CHANNELS"],
        "io_kind": "digital output",
        "direction": "output",
        "details": "4-channel CMOS push-pull digital output terminal.",
        "accepted_voltage": "5 V DC nominal output.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed2xxx-digital-output/el2124.html",
    },
    "EL2798": {
        "full_name": "Beckhoff EL2798",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL2798_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL2798_NUM_CHANNELS"],
        "io_kind": "digital output",
        "direction": "output",
        "details": "8-channel solid-state relay output with potential-free make contacts.",
        "accepted_voltage": "0-30 V AC/DC nominal; up to 48 V DC for ohmic loads.",
        "source_url": "https://www.beckhoff.com/pl-pl/products/i-o/ethercat-terminals/el-ed2xxx-digital-output/el2798.html",
    },
    "EL2809": {
        "full_name": "Beckhoff EL2809",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL2809_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL2809_NUM_CHANNELS"],
        "io_kind": "digital output",
        "direction": "output",
        "details": "16-channel positive-switching digital output terminal.",
        "accepted_voltage": "24 V DC nominal (-15%/+20%).",
        "source_url": "https://www.beckhoff.com/ms-my/products/i-o/ethercat-terminals/el2xxx-digital-output/el2809.html",
    },
    "EL2828": {
        "full_name": "Beckhoff EL2828",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL2828_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL2828_NUM_CHANNELS"],
        "io_kind": "digital output",
        "direction": "output",
        "details": "8-channel digital output terminal for resistive, inductive, and capacitive loads.",
        "accepted_voltage": "24 V DC nominal; 20.4-28.8 V DC technical range.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed2xxx-digital-output/ed2828.html",
    },
    "EL3104": {
        "full_name": "Beckhoff EL3104",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3104_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3104_NUM_CHANNELS"],
        "io_kind": "analog voltage input",
        "direction": "input",
        "details": "4-channel differential analog voltage input with filter, limit, and sync-error reporting.",
        "accepted_voltage": "-10 to +10 V signal range.",
        "source_url": "https://www.beckhoff.com/de-de/produkte/i-o/ethercat-klemmen/el-ed3xxx-analog-eingang/el3104.html",
    },
    "EL3162": {
        "full_name": "Beckhoff EL3162",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3162_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3162_NUM_CHANNELS"],
        "io_kind": "analog voltage input",
        "direction": "input",
        "details": "2-channel single-ended analog voltage input with underrange, overrange, and error flags.",
        "accepted_voltage": "0 to 10 V signal range; dielectric strength max 30 V.",
        "source_url": "https://www.beckhoff.com/de-de/produkte/i-o/ethercat-klemmen/el-ed3xxx-analog-eingang/el3162.html",
    },
    "EL3202": {
        "full_name": "Beckhoff EL3202-0010",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3202_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3202_NUM_CHANNELS"],
        "io_kind": "RTD / resistance input",
        "direction": "input",
        "details": "2-channel RTD/resistance input with configurable sensor type and connection technology.",
        "accepted_voltage": "Not a voltage-input terminal; supports RTD/resistance sensors with measuring current < 0.5 mA.",
        "source_url": "https://www.beckhoff.com/en-en/products/i-o/ethercat-terminals/el-ed3xxx-analog-input/el3202-0010.html",
    },
    "EL3208": {
        "full_name": "Beckhoff EL3208",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3208_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3208_NUM_CHANNELS"],
        "io_kind": "RTD / resistance input",
        "direction": "input",
        "details": "8-channel RTD/resistance input with configurable sensor type, wiring, filters, and presentation mode.",
        "accepted_voltage": "Not a voltage-input terminal; supports RTD/resistance sensors with measuring current < 0.5 mA.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed3xxx-analog-input/el3208.html",
    },
    "EL3314": {
        "full_name": "Beckhoff EL3314",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3314_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3314_NUM_CHANNELS"],
        "io_kind": "thermocouple input",
        "direction": "input",
        "details": "4-channel thermocouple input with multiple TC types, wire-break detection, and cold-junction handling.",
        "accepted_voltage": "Thermocouple / voltage measurement ranges of +/-30, +/-60, or +/-75 mV.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed3xxx-analog-input/el3314.html",
    },
    "EL3318": {
        "full_name": "Beckhoff EL3318",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3318_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3318_NUM_CHANNELS"],
        "io_kind": "thermocouple input",
        "direction": "input",
        "details": "8-channel thermocouple input with multiple TC types, wire-break detection, and cold-junction handling.",
        "accepted_voltage": "Thermocouple / voltage measurement ranges of +/-30, +/-60, or +/-75 mV.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed3xxx-analog-input/el3318.html",
    },
    "EL3356": {
        "full_name": "Beckhoff EL3356",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3356_PRODUCT_CODE"],
        "channel_summary": "1 weighing / strain-gauge value",
        "io_kind": "load cell / strain gauge input",
        "direction": "input",
        "details": "Load-cell / full-bridge input with tare support, scaling, auto-calibration, and steady-state status.",
        "accepted_voltage": "Bridge reference nominal +/-12 V; technically usable to about +/-13.8 V. Beckhoff recommends 10 V or 12 V depending on sensor.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el3xxx-analog-input/el3356.html",
    },
    "EL3602": {
        "full_name": "Beckhoff EL3602",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL3602_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL3602_NUM_CHANNELS"],
        "io_kind": "high-resolution analog voltage input",
        "direction": "input",
        "details": "2-channel differential high-resolution analog voltage input with filters and limit monitoring.",
        "accepted_voltage": "Main EL3602 supports +/-10, +/-5, +/-2.5, and +/-1.25 V; related variants support +/-75 mV and +/-200 mV.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed3xxx-analog-input/el3602.html ; https://www.beckhoff.com/de-ch/produkte/i-o/ethercat-klemmen/el3xxx-analog-eingang/el3602-0010.html ; https://www.beckhoff.com/en-en/products/i-o/ethercat-terminals/el-ed3xxx-analog-input/el3602-0002.html",
    },
    "EL4102": {
        "full_name": "Beckhoff EL4102",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL4102_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL4102_NUM_CHANNELS"],
        "io_kind": "analog voltage output",
        "direction": "output",
        "details": "2-channel analog voltage output terminal.",
        "accepted_voltage": "0 to 10 V output range.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed4xxx-analog-output/el4102.html",
    },
    "EL5042": {
        "full_name": "Beckhoff EL5042",
        "manufacturer": "Beckhoff",
        "vendor_macro": "JSD_BECKHOFF_VENDOR_ID",
        "product_macros": ["JSD_EL5042_PRODUCT_CODE"],
        "channel_macros": ["JSD_EL5042_NUM_CHANNELS"],
        "io_kind": "BiSS-C / SSI position input",
        "direction": "input",
        "details": "2-channel BiSS-C / SSI encoder interface with configurable clocking, coding, and encoder bit layout.",
        "accepted_voltage": "Encoder supply selectable between 5 V DC and 9 V DC; electronics supplied from 24 V DC power contacts.",
        "source_url": "https://www.beckhoff.com/en-us/products/i-o/ethercat-terminals/el-ed5xxx-position-measurement/el5042.html",
    },
    "EPD_NOMINAL": {
        "full_name": "Elmo Platinum Drive (Nominal Mode)",
        "manufacturer": "Elmo Motion Control",
        "vendor_macro": "JSD_ELMO_VENDOR_ID",
        "product_macros": [
            "JSD_EPD_PRODUCT_CODE_STD_FW",
            "JSD_EPD_PRODUCT_CODE_SAFETY_FW",
        ],
        "channel_macros": [
            "JSD_EPD_NOMINAL_NUM_DIGITAL_INPUTS",
            "JSD_EPD_NOMINAL_NUM_DIGITAL_OUTPUTS",
        ],
        "io_kind": "servo drive and mixed I/O",
        "direction": "bidirectional",
        "details": "Platinum EtherCAT servo-drive family in nominal mode, with DS-402 motion modes, 2 analog inputs, and discrete I/O.",
        "accepted_voltage": "Varies by Platinum model; official Platinum pages show 10-95 VDC low-voltage variants, while high-voltage models use roughly 50-780 VDC bus supplies and 22-52 VDC control supplies.",
        "source_url": "https://www.elmomc.com/product/platinum-quartet/ ; https://www.elmomc.com/product/platinum-jori/",
    },
    "EPD_SIL": {
        "full_name": "Elmo Platinum Drive (SIL Mode)",
        "manufacturer": "Elmo Motion Control",
        "vendor_macro": "JSD_ELMO_VENDOR_ID",
        "product_macros": [
            "JSD_EPD_PRODUCT_CODE_STD_FW",
            "JSD_EPD_PRODUCT_CODE_SAFETY_FW",
        ],
        "channel_macros": ["JSD_EPD_SIL_R1_MAX_NUM", "JSD_EPD_SIL_R2_MAX_NUM"],
        "io_kind": "servo drive with SIL user mappings",
        "direction": "bidirectional",
        "details": "Platinum EtherCAT servo-drive family in SIL mode, with configurable R1/R2 mapped variables and user PO2SO callback support.",
        "accepted_voltage": "Varies by Platinum model; official Platinum pages show 10-95 VDC low-voltage variants, while high-voltage models use roughly 50-780 VDC bus supplies and 22-52 VDC control supplies.",
        "source_url": "https://www.elmomc.com/product/platinum-quartet/ ; https://www.elmomc.com/product/platinum-jori/",
    },
    "ILD1900": {
        "full_name": "Micro-Epsilon optoNCDT ILD1900",
        "manufacturer": "Micro-Epsilon",
        "vendor_macro": "JSD_MICROEPSILON_VENDOR_ID",
        "product_macros": ["JSD_ILD1900_PRODUCT_CODE"],
        "channel_summary": "1 displacement measurement",
        "io_kind": "laser displacement input",
        "direction": "input",
        "details": "Laser displacement sensor with configurable model, exposure mode, peak selection, averaging, and measurement rate.",
        "accepted_voltage": "24 V DC nominal, 11-30 V DC operating range; PoE is also supported on EtherCAT variants.",
        "source_url": "https://www.micro-epsilon.com/fileadmin/download/manuals/man--optoNCDT-1900-IE-EtherCAT--en.pdf",
    },
    "JED0101": {
        "full_name": "JPL EtherCAT Device 0101",
        "manufacturer": "NASA JPL",
        "vendor_macro": "JSD_JPL_VENDOR_ID",
        "product_macros": ["JSD_JED0101_PRODUCT_CODE"],
        "channel_summary": "quaternion telemetry + command word",
        "io_kind": "custom telemetry / command interface",
        "direction": "bidirectional",
        "details": "Custom JPL EtherCAT device exposing IMU quaternion data and a command register.",
        "accepted_voltage": "No public online documentation found.",
        "source_url": "",
    },
    "JED0200": {
        "full_name": "JPL EtherCAT Device 0200",
        "manufacturer": "NASA JPL",
        "vendor_macro": "JSD_JPL_VENDOR_ID",
        "product_macros": ["JSD_JED0200_PRODUCT_CODE"],
        "channel_summary": "power, environment, brake telemetry + command word",
        "io_kind": "custom telemetry / command interface",
        "direction": "bidirectional",
        "details": "Custom JPL EtherCAT device exposing voltages, temperatures, humidity, pressure, and brake telemetry.",
        "accepted_voltage": "No public online documentation found.",
        "source_url": "",
    },
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
    source_exists: bool
    private_header_path: Path
    private_header_exists: bool
    public_header_path: Path
    public_header_exists: bool
    types_header_path: Path
    types_header_exists: bool
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


def parse_src_driver_source_ids(src_dir: Path) -> set[str]:
    ids = {path.stem.removeprefix("jsd_").upper() for path in src_dir.glob("jsd_*.c")}
    return {driver_id for driver_id in ids if driver_id not in NON_DRIVER_CMAKE_SOURCES}


def parse_c_defines(path: Path) -> dict[str, str]:
    text = path.read_text(encoding="utf-8")
    logical_lines: list[str] = []
    current = ""

    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        if current:
            current += line.lstrip()
        else:
            current = line

        if current.endswith("\\"):
            current = current[:-1] + " "
            continue

        logical_lines.append(current)
        current = ""

    defines: dict[str, str] = {}
    for line in logical_lines:
        match = re.match(r"#define\s+([A-Z0-9_]+)\s+(.*)", line)
        if match:
            defines[match.group(1)] = match.group(2).strip()

    return defines


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
    src_source_ids = parse_src_driver_source_ids(SRC_DIR)
    src_cmake_ids = parse_src_cmake_driver_ids(SRC_CMAKE_PATH)
    enum_ids, config_ids, state_ids = parse_jsd_types(JSD_TYPES_PATH)
    candidate_ids = sorted(src_source_ids | src_cmake_ids | enum_ids | config_ids | state_ids)

    evidence: list[DriverEvidence] = []
    for driver_id in candidate_ids:
        source_path = SRC_DIR / f"jsd_{driver_id.lower()}.c"
        private_header_path = SRC_DIR / f"jsd_{driver_id.lower()}.h"
        public_header_path = SRC_DIR / f"jsd_{driver_id.lower()}_pub.h"
        types_header_path = SRC_DIR / f"jsd_{driver_id.lower()}_types.h"
        evidence.append(
            DriverEvidence(
                driver_id=driver_id,
                group=driver_to_group(driver_id),
                source_path=source_path,
                source_exists=source_path.exists(),
                private_header_path=private_header_path,
                private_header_exists=private_header_path.exists(),
                public_header_path=public_header_path,
                public_header_exists=public_header_path.exists(),
                types_header_path=types_header_path,
                types_header_exists=types_header_path.exists(),
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


def bool_cell(value: bool) -> str:
    return "yes" if value else "no"


def markdown_escape(text: str) -> str:
    return text.replace("|", "\\|")


def extract_hex_literal(raw_value: str) -> str:
    match = re.search(r"0x[0-9A-Fa-f]+", raw_value)
    return match.group(0) if match else raw_value


def extract_int_literal(raw_value: str) -> str:
    match = re.search(r"\b\d+\b", raw_value)
    return match.group(0) if match else raw_value


def load_global_defines() -> dict[str, str]:
    defines = {}
    defines.update(parse_c_defines(COMMON_DEVICE_TYPES_PATH))
    defines.update(parse_c_defines(EPD_COMMON_TYPES_PATH))

    for path in SRC_DIR.glob("jsd_*_types.h"):
        defines.update(parse_c_defines(path))

    return defines


def format_product_codes(product_macros: list[str], defines: dict[str, str]) -> str:
    values = []
    for macro_name in product_macros:
        raw_value = defines.get(macro_name)
        if raw_value:
            values.append(f"{macro_name}={extract_hex_literal(raw_value)}")
        else:
            values.append(f"{macro_name}=missing")
    return ", ".join(values)


def format_capacity(driver_id: str, metadata: dict[str, object], defines: dict[str, str]) -> str:
    channel_macros = metadata.get("channel_macros")
    if channel_macros:
        channel_macros = list(channel_macros)
        if driver_id == "EGD" and len(channel_macros) == 2:
            di = extract_int_literal(defines.get(channel_macros[0], "missing"))
            do = extract_int_literal(defines.get(channel_macros[1], "missing"))
            return f"{di} digital in / {do} digital out"
        if driver_id == "EPD_NOMINAL" and len(channel_macros) == 2:
            di = extract_int_literal(defines.get(channel_macros[0], "missing"))
            do = extract_int_literal(defines.get(channel_macros[1], "missing"))
            return f"{di} digital in / {do} digital out (+ 2 analog inputs)"
        if driver_id == "EPD_SIL" and len(channel_macros) == 2:
            r1 = extract_int_literal(defines.get(channel_macros[0], "missing"))
            r2 = extract_int_literal(defines.get(channel_macros[1], "missing"))
            return f"up to {r1} R1 vars / {r2} R2 vars"
        if len(channel_macros) == 1:
            return extract_int_literal(defines.get(channel_macros[0], "missing"))
        return ", ".join(
            f"{macro_name}={extract_int_literal(defines.get(macro_name, 'missing'))}"
            for macro_name in channel_macros
        )

    channel_summary = metadata.get("channel_summary")
    return str(channel_summary) if channel_summary else ""


def build_device_catalog_rows(evidence: list[DriverEvidence]) -> list[dict[str, str]]:
    defines = load_global_defines()
    rows: list[dict[str, str]] = []

    for item in evidence:
        metadata = DEVICE_CATALOG_METADATA.get(item.driver_id)
        if not metadata:
            continue

        vendor_macro = str(metadata["vendor_macro"])
        vendor_value = defines.get(vendor_macro, "missing")

        rows.append(
            {
                "driver": item.driver_id,
                "full_name": str(metadata["full_name"]),
                "manufacturer": str(metadata["manufacturer"]),
                "vendor_id": f"{vendor_macro}={extract_hex_literal(vendor_value)}",
                "product_codes": format_product_codes(
                    list(metadata["product_macros"]),
                    defines,
                ),
                "capacity": format_capacity(item.driver_id, metadata, defines),
                "io_kind": str(metadata["io_kind"]),
                "direction": str(metadata["direction"]),
                "accepted_voltage": str(metadata["accepted_voltage"]),
                "details": str(metadata["details"]),
                "source_url": str(metadata["source_url"]),
            }
        )

    return rows


def render_markdown_matrix(root_has_src: bool, evidence: list[DriverEvidence]) -> str:
    readme_entries = parse_readme_entries(README_PATH)
    readme_by_group = {entry.group: entry for entry in readme_entries}

    lines = [
        "# Device Support Matrix",
        "",
        "Generated by `tools/check_supported_devices.py` from code-derived signals.",
        "",
        f"- Root [CMakeLists.txt](../CMakeLists.txt) wires `src/`: `{bool_cell(root_has_src)}`",
        "- `Supported` means all of these are true: built in `src/CMakeLists.txt`, present in `JSD_DRIVER_TYPE_*`, present in the config union, present in the state union, and implemented with `*_product_code_is_compatible(...)`.",
        "- `README row` is grouped where the README uses a family label such as `Elmo Platinum Drives` or `JED (JPL EtherCat Device)`.",
        "",
        "| Driver | README row | Source | Private hdr | Public hdr | Types hdr | Built in src/CMakeLists.txt | In JSD_DRIVER_TYPE_* | In config union | In state union | Has product_code_is_compatible | Supported |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]

    for item in evidence:
        readme_label = readme_by_group[item.group].device if item.group in readme_by_group else ""
        source_label = item.source_path.relative_to(ROOT).as_posix()
        private_header_label = item.private_header_path.relative_to(ROOT).as_posix()
        public_header_label = item.public_header_path.relative_to(ROOT).as_posix()
        types_header_label = item.types_header_path.relative_to(ROOT).as_posix()

        lines.append(
            "| "
            + " | ".join(
                [
                    markdown_escape(item.driver_id),
                    markdown_escape(readme_label),
                    markdown_escape(
                        f"`{source_label}`" if item.source_exists else f"`{source_label}` (missing)"
                    ),
                    markdown_escape(
                        f"`{private_header_label}`"
                        if item.private_header_exists
                        else f"`{private_header_label}` (missing)"
                    ),
                    markdown_escape(
                        f"`{public_header_label}`"
                        if item.public_header_exists
                        else f"`{public_header_label}` (missing)"
                    ),
                    markdown_escape(
                        f"`{types_header_label}`"
                        if item.types_header_exists
                        else f"`{types_header_label}` (missing)"
                    ),
                    bool_cell(item.in_src_cmake),
                    bool_cell(item.in_driver_enum),
                    bool_cell(item.in_config_union),
                    bool_cell(item.in_state_union),
                    bool_cell(item.has_product_code_check),
                    bool_cell(item.is_supported),
                ]
            )
            + " |"
        )

    lines.extend(["", "## Notes", ""])

    supported_groups = grouped_supported_drivers(evidence)
    noted_groups = sorted((set(supported_groups) | {entry.group for entry in readme_entries}) & GROUP_NOTES.keys())
    if noted_groups:
        for group in noted_groups:
            lines.append(f"- `{display_group_name(group)}`: {GROUP_NOTES[group]}")
    else:
        lines.append("- None")

    lines.append("")
    return "\n".join(lines)


def write_markdown_matrix(output_path: Path) -> None:
    root_has_src, evidence = collect_driver_evidence()
    output_path.write_text(render_markdown_matrix(root_has_src, evidence), encoding="utf-8")


def render_device_catalog_markdown(evidence: list[DriverEvidence]) -> str:
    rows = build_device_catalog_rows(evidence)

    lines = [
        "# Device Catalog",
        "",
        "Generated by `tools/check_supported_devices.py` from device type headers, shared driver metadata, and online manufacturer references.",
        "",
        "| Driver | Full name | Manufacturer | Vendor ID | Product code(s) | Channels / capacity | I/O kind | Direction | Accepted voltage | Details | Official source |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]

    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    markdown_escape(row["driver"]),
                    markdown_escape(row["full_name"]),
                    markdown_escape(row["manufacturer"]),
                    markdown_escape(f"`{row['vendor_id']}`"),
                    markdown_escape(f"`{row['product_codes']}`"),
                    markdown_escape(row["capacity"]),
                    markdown_escape(row["io_kind"]),
                    markdown_escape(row["direction"]),
                    markdown_escape(row["accepted_voltage"]),
                    markdown_escape(row["details"]),
                    markdown_escape(row["source_url"] if row["source_url"] else "No public source found"),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Scope Notes",
            "",
            "- `Product code(s)` and vendor IDs are parsed from the type headers and shared device-type headers.",
            "- `Channels / capacity` uses `JSD_*_NUM_CHANNELS` where available. For drives and custom devices, it uses the most relevant capacity macros or a driver-specific summary.",
            "- `Accepted voltage` and `Official source` are based on vendor product pages, manuals, or technical reference pages checked online.",
            "- `Details` are intentionally concise and combine the code-facing view of the device with the vendor descriptions.",
            "",
        ]
    )

    return "\n".join(lines)


def write_device_catalog(output_path: Path) -> None:
    _, evidence = collect_driver_evidence()
    output_path.write_text(render_device_catalog_markdown(evidence), encoding="utf-8")


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
    parser.add_argument(
        "--write-markdown",
        type=Path,
        help="Write a detailed device support matrix as Markdown to the given path.",
    )
    parser.add_argument(
        "--write-device-catalog",
        type=Path,
        help="Write a generated device metadata catalog as Markdown to the given path.",
    )
    args = parser.parse_args()
    if args.write_markdown:
        output_path = args.write_markdown
        if not output_path.is_absolute():
            output_path = ROOT / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_markdown_matrix(output_path)
    if args.write_device_catalog:
        output_path = args.write_device_catalog
        if not output_path.is_absolute():
            output_path = ROOT / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_device_catalog(output_path)
    return print_report(strict=args.strict)


if __name__ == "__main__":
    sys.exit(main())
