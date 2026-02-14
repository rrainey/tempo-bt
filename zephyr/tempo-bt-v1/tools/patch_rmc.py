#!/usr/bin/env python3
"""
patch_rmc.py - Insert a synthesized $GNRMC sentence into a Tempo-BT flight log.

Logs created by firmware versions prior to the RMC fix lack $GNRMC sentences,
which carry the UTC date. This script synthesizes an RMC sentence from the
first GGA and VTG sentences in the log and inserts it immediately after the
$PTH line that follows that first VTG/GGA pair.

Usage:
    python patch_rmc.py <YYYY-MM-DD> <flight.txt>

The file is modified in-place. A backup is saved as <flight.txt>.bak.
"""

import sys
import os
import re


def nmea_checksum(body):
    """Compute NMEA checksum over the characters between '$' and '*'."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return cs


def parse_gga(sentence):
    """Extract fields from a GGA sentence."""
    fields = sentence.split(",")
    # $GNGGA,hhmmss.ss,lat,N/S,lon,E/W,quality,...
    return {
        "utc":       fields[1],      # hhmmss.ss
        "lat":       fields[2],      # DDmm.mmmmm
        "lat_dir":   fields[3],      # N/S
        "lon":       fields[4],      # DDDmm.mmmmm
        "lon_dir":   fields[5],      # E/W
        "quality":   fields[6],      # 0=none,1=GPS,2=DGPS
    }


def parse_vtg(sentence):
    """Extract fields from a VTG sentence."""
    # Strip checksum for field parsing
    body = sentence.split("*")[0]
    fields = body.split(",")
    # $GNVTG,track_true,T,track_mag,M,speed_kn,N,speed_kmh,K,mode
    return {
        "track_true": fields[1],     # degrees
        "speed_kn":   fields[5],     # knots
        "mode":       fields[9] if len(fields) > 9 else "",  # A/D/E/N
    }


def fix_quality_to_mode(quality):
    """Map GGA fix quality to RMC mode indicator."""
    # 0=none->N, 1=GPS->A, 2=DGPS->D, 4=RTK->R, 5=float RTK->F, 6=estimated->E
    mapping = {"0": "N", "1": "A", "2": "D", "4": "R", "5": "F", "6": "E"}
    return mapping.get(quality, "A")


def build_rmc(date_ddmmyy, gga, vtg):
    """Build a complete $GNRMC sentence with valid checksum.

    RMC fields (per NMEA 0183 / NovAtel OEM7 reference):
      1  UTC time           - from GGA
      2  Status             - A (valid fix)
      3  Latitude           - from GGA
      4  N/S                - from GGA
      5  Longitude          - from GGA
      6  E/W                - from GGA
      7  Speed over ground  - from VTG (knots)
      8  Track made good    - from VTG (degrees true)
      9  Date               - from command-line (DDMMYY)
      10 Magnetic variation - empty (not available)
      11 Variation dir      - empty
      12 Mode indicator     - derived from GGA fix quality
    """
    mode = fix_quality_to_mode(gga["quality"])

    body = "GNRMC,{utc},{status},{lat},{lat_dir},{lon},{lon_dir}," \
           "{speed},{track},{date},,," \
           "{mode}".format(
               utc=gga["utc"],
               status="A",
               lat=gga["lat"],
               lat_dir=gga["lat_dir"],
               lon=gga["lon"],
               lon_dir=gga["lon_dir"],
               speed=vtg["speed_kn"],
               track=vtg["track_true"],
               date=date_ddmmyy,
               mode=mode,
           )

    cs = nmea_checksum(body)
    return "${body}*{cs:02X}".format(body=body, cs=cs)


def main():
    if len(sys.argv) != 3:
        print("Usage: {} YYYY-MM-DD <flight.txt>".format(sys.argv[0]),
              file=sys.stderr)
        sys.exit(1)

    date_str = sys.argv[1]
    filepath = sys.argv[2]

    # Validate and convert date from YYYY-MM-DD to DDMMYY
    m = re.match(r"^(\d{4})-(\d{2})-(\d{2})$", date_str)
    if not m:
        print("Error: date must be in YYYY-MM-DD format", file=sys.stderr)
        sys.exit(1)

    year, month, day = m.group(1), m.group(2), m.group(3)
    yy = year[2:]  # last two digits
    date_ddmmyy = "{day}{month}{yy}".format(day=day, month=month, yy=yy)

    # Read the file
    with open(filepath, "r") as f:
        lines = f.readlines()

    # Scan for the first VTG/GGA/$PTH triplet
    vtg_data = None
    gga_data = None
    insert_index = None

    i = 0
    while i < len(lines):
        stripped = lines[i].rstrip("\r\n")

        if stripped.startswith("$GNVTG,") or stripped.startswith("$GPVTG,"):
            vtg_candidate = parse_vtg(stripped)
            # Look for GGA on the next line
            if i + 1 < len(lines):
                next_line = lines[i + 1].rstrip("\r\n")
                if next_line.startswith("$GNGGA,") or next_line.startswith("$GPGGA,"):
                    gga_candidate = parse_gga(next_line)
                    # Look for $PTH on the line after that
                    if i + 2 < len(lines):
                        pth_line = lines[i + 2].rstrip("\r\n")
                        if pth_line.startswith("$PTH,"):
                            vtg_data = vtg_candidate
                            gga_data = gga_candidate
                            insert_index = i + 3  # after the $PTH line
                            break
        i += 1

    if insert_index is None:
        print("Error: could not find a VTG/GGA/PTH triplet in the log",
              file=sys.stderr)
        sys.exit(1)

    # Check if an RMC sentence already exists near the insertion point
    for j in range(max(0, insert_index - 4), min(len(lines), insert_index + 2)):
        if "$GNRMC," in lines[j] or "$GPRMC," in lines[j]:
            print("Warning: RMC sentence already present at line {}; "
                  "file not modified.".format(j + 1), file=sys.stderr)
            sys.exit(0)

    # Build the synthesized RMC
    rmc_sentence = build_rmc(date_ddmmyy, gga_data, vtg_data)

    # Detect line ending style from the file
    eol = "\r\n" if lines[0].endswith("\r\n") else "\n"

    # Insert
    lines.insert(insert_index, rmc_sentence + eol)

    # Write backup, then overwrite original
    backup = filepath + ".bak"
    if os.path.exists(backup):
        os.remove(backup)
    os.rename(filepath, backup)

    with open(filepath, "w", newline="") as f:
        f.writelines(lines)

    print("Inserted: {}".format(rmc_sentence))
    print("  after line {} (file backed up to {})".format(insert_index, backup))


if __name__ == "__main__":
    main()
