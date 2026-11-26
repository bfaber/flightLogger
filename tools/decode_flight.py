
import argparse
import csv
import math
from pathlib import Path


def decode_calibration(hex_string):
    b = bytes.fromhex(hex_string)

    if len(b) != 21:
        raise ValueError("Expected 21 calibration bytes")

    def u16(i):
        return int.from_bytes(b[i:i+2], "little", signed=False)

    def s16(i):
        return int.from_bytes(b[i:i+2], "little", signed=True)

    def s8(i):
        return int.from_bytes(b[i:i+1], "little", signed=True)

    return {
        "t1": math.ldexp(float(u16(0)), 8),
        "t2": math.ldexp(float(u16(2)), -30),
        "t3": math.ldexp(float(s8(4)), -48),

        "p1": math.ldexp(float(s16(5)) - 16384.0, -20),
        "p2": math.ldexp(float(s16(7)) - 16384.0, -29),
        "p3": math.ldexp(float(s8(9)), -32),
        "p4": math.ldexp(float(s8(10)), -37),
        "p5": math.ldexp(float(u16(11)), 3),
        "p6": math.ldexp(float(u16(13)), -6),
        "p7": math.ldexp(float(s8(15)), -8),
        "p8": math.ldexp(float(s8(16)), -15),
        "p9": math.ldexp(float(s16(17)), -48),
        "p10": math.ldexp(float(s8(19)), -48),
        "p11": math.ldexp(float(s8(20)), -65),
    }


def compensate(pressure_adc, temperature_adc, c):
    delta = temperature_adc - c["t1"]

    t = delta * c["t2"] + delta**2 * c["t3"]

    offset = (
        c["p5"] + c["p6"] * t +
        c["p7"] * t**2 + c["p8"] * t**3
    )

    sensitivity = (
        c["p1"] + c["p2"] * t +
        c["p3"] * t**2 + c["p4"] * t**3
    )

    p = float(pressure_adc)

    pressure_pa = (
        offset +
        p * sensitivity +
        p**2 * (c["p9"] + c["p10"] * t) +
        p**3 * c["p11"]
    )

    return pressure_pa, t


def read_dump(path):
    calibration = None
    samples = []
    inside = False

    for line in Path(path).read_text(
        encoding="utf-8", errors="replace"
    ).splitlines():

        line = line.strip()

        if line == "BEGIN_FLIGHT_LOG":
            inside = True
            calibration = None
            samples = []
            continue

        if line == "END_FLIGHT_LOG":
            break

        if not inside:
            continue

        if line.startswith("CAL,"):
            calibration = decode_calibration(line[4:])

        elif line.startswith("S,"):
            _, ts, pressure, temperature = line.split(",")

            samples.append((
                int(ts), int(pressure), int(temperature)
            ))

    if calibration is None:
        raise ValueError("No calibration header found")

    if not samples:
        raise ValueError("No flight samples found")

    return calibration, samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Saved serial monitor output")
    parser.add_argument(
        "-o", "--output", default="flight.csv"
    )
    args = parser.parse_args()

    calibration, samples = read_dump(args.input)

    compensated = [
        compensate(pressure, temperature, calibration)
        for _, pressure, temperature in samples
    ]

    baseline_count = min(50, len(compensated))
    reference_pa = sum(
        p for p, _ in compensated[:baseline_count]
    ) / baseline_count

    rows = []

    for (timestamp, raw_p, raw_t), (pressure, temp) \
            in zip(samples, compensated):

        altitude_m = 44330.0 * (
            1.0 - (pressure / reference_pa)**0.19029496
        )

        rows.append({
            "time_s": timestamp / 1_000_000,
            "pressure_adc": raw_p,
            "temperature_adc": raw_t,
            "pressure_hpa": pressure / 100.0,
            "temperature_f": temp * 1.8 + 32.0,
            "altitude_ft": altitude_m * 3.28084,
        })

    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=rows[0].keys()
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Decoded {len(rows)} samples")
    print(f"Reference pressure: {reference_pa / 100:.4f} hPa")
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
