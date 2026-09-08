"""Talking to the moteus n1: query setup and the aux telemetry channel.

Two separate paths to the controller and they behave nothing alike:

  fast path   the normal CAN-FD query. One 64-byte frame, so the register list
              is tight and verify_query() checks nothing fell off the end.
  diagnostic  the aux1 telemetry stream, used for BiSS-C CRC counts and raw
              encoder values. Slow, fragile, and every call is wrapped in a
              timeout because it will happily block forever.
"""
import asyncio
import math
import re

DIAG_TIMEOUT_S = 2.0  # hard cap on any diagnostic-channel call; never block a run


def make_query_resolution(moteus):
    qr = moteus.QueryResolution()
    qr.mode = moteus.INT8
    qr.position = moteus.F32
    qr.velocity = moteus.F32
    qr.torque = moteus.F32
    qr.q_current = moteus.F32
    qr.d_current = moteus.F32
    qr.voltage = moteus.INT8
    qr.temperature = moteus.INT8
    qr.power = moteus.F32
    qr.fault = moteus.INT8
    # Encoder registers have no named QueryResolution field; add them by
    # register number via _extra so they ride along in the same fast query.
    R = moteus.Register
    qr._extra = {
        R.ENCODER_0_POSITION: moteus.F32,   # Orbis, output-referred
        R.ENCODER_0_VELOCITY: moteus.F32,
    }
    # DELIBERATELY NOT IN THE FAST QUERY, see SLOW_REGISTERS below.
    # A query reply must fit in one 64-byte CAN-FD frame. Adding
    # motor_temperature or encoder_validity here pushes it over, and the
    # firmware SILENTLY TRUNCATES the tail rather than erroring: the first
    # casualty is FAULT, which would disable the main abort check.
    # verify_query() below asserts nothing went missing. Do not add registers
    # here without re-running --dry-run and checking that assertion passes.
    qr.motor_temperature = moteus.IGNORE
    return qr


# Static-ish registers, polled at the slow cadence to keep the fast reply small.
def slow_registers(moteus):
    R = moteus.Register
    return {
        R.MOTOR_TEMPERATURE: moteus.F32,   # constant 0, no thermistor fitted
        R.ENCODER_VALIDITY: moteus.INT8,   # config bitmask, does not change
    }


FAST_EXPECTED = [
    "MODE", "POSITION", "VELOCITY", "TORQUE", "Q_CURRENT", "D_CURRENT",
    "POWER", "VOLTAGE", "TEMPERATURE", "FAULT",
    "ENCODER_0_POSITION", "ENCODER_0_VELOCITY",
]


def verify_query(moteus, result):
    """Fail loudly if the reply truncated. Returns list of missing names."""
    present = set()
    for k in result.values:
        try:
            present.add(moteus.Register(k).name)
        except ValueError:
            pass
    return [n for n in FAST_EXPECTED if n not in present]



def flatten(obj, prefix=""):
    """Flatten a moteus telemetry object into {dotted.path: value}."""
    out = {}
    if hasattr(obj, "_asdict"):
        obj = obj._asdict()
    elif hasattr(obj, "__dict__") and not isinstance(obj, (int, float, str, bytes)):
        obj = vars(obj)

    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(flatten(v, f"{prefix}{k}."))
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            out.update(flatten(v, f"{prefix}{i}."))
    else:
        out[prefix.rstrip(".")] = obj
    return out


async def read_slow(moteus, controller):
    """Separate small query for the registers kept out of the fast reply."""
    try:
        r = await controller.custom_query(slow_registers(moteus))
    except Exception:
        return {}
    R = moteus.Register
    out = {}
    if R.MOTOR_TEMPERATURE in r.values:
        out["motor_temp_C"] = r.values[R.MOTOR_TEMPERATURE]
    if R.ENCODER_VALIDITY in r.values:
        out["encoder_validity"] = r.values[R.ENCODER_VALIDITY]
    return out



class AuxReader:
    """Polls the aux1 diagnostic telemetry channel for crc_errors + raw counts.

    Field names in the aux1 struct vary between firmware revisions, so the
    names are discovered by pattern at connect time rather than hardcoded.
    """

    # bissc CRC error counter
    RE_CRC = re.compile(r"bissc.*(crc.*err|err.*crc)", re.I)
    # bissc raw value
    RE_BISSC_VAL = re.compile(r"bissc\.(value|position|raw)", re.I)
    # onboard AS5047P raw value: an spi.* value that is NOT under bissc
    RE_SPI_VAL = re.compile(r"(^|\.)spi\.(value|position|raw)", re.I)

    def __init__(self, moteus_mod, controller):
        self._moteus = moteus_mod
        self._stream = moteus_mod.Stream(controller)
        self.key_crc = None
        self.key_bissc = None
        self.key_spi = None
        self.baseline_crc = None
        self.available = False
        self.error = None
        self.last_flat = {}
        self.read_failures = 0

    async def resync(self):
        """Clear stale bytes from the diagnostic stream.

        A previous reader (tview, moteus_tool, an aborted run) can leave
        partial telemetry frames buffered. read_data then chokes on them with
        'Unexpected schema announce'. Stopping the telemetry stream and
        draining the buffer puts the channel back in a known state.

        Every call is wrapped in a timeout. `stream.command` blocks forever
        waiting for an 'OK' that never arrives if another tool left the
        telemetry channel streaming, and an unbounded diagnostic call can
        stall a run while the motor is turning. Nothing here is allowed to
        block: a failed resync degrades CRC logging, it must never hang.
        """
        try:
            await asyncio.wait_for(self._stream.command(b'tel stop'),
                                   timeout=DIAG_TIMEOUT_S)
        except Exception:
            pass
        try:
            await asyncio.wait_for(self._stream.flush_read(),
                                   timeout=DIAG_TIMEOUT_S)
        except Exception:
            pass

    async def read_flat(self):
        data = await asyncio.wait_for(self._stream.read_data("aux1"),
                                      timeout=DIAG_TIMEOUT_S)
        return flatten(data)

    async def connect(self):
        """Discover field names and capture the CRC baseline."""
        flat = None
        for attempt in range(3):
            await self.resync()
            try:
                flat = await self.read_flat()
                break
            except Exception as e:
                self.error = f"{type(e).__name__}: {e}"
                await asyncio.sleep(0.2)
        if flat is None:
            return False

        self.last_flat = flat
        for k in flat:
            if self.key_crc is None and self.RE_CRC.search(k):
                self.key_crc = k
            if self.key_bissc is None and self.RE_BISSC_VAL.search(k):
                self.key_bissc = k
            if (self.key_spi is None and self.RE_SPI_VAL.search(k)
                    and "bissc" not in k.lower()):
                self.key_spi = k

        if self.key_crc is None:
            self.error = ("no bissc crc field found in aux1 telemetry; "
                          f"fields seen: {sorted(flat)}")
            return False

        try:
            self.baseline_crc = float(flat[self.key_crc])
        except (TypeError, ValueError):
            self.baseline_crc = float("nan")

        self.available = True
        return True

    async def sample(self):
        """Return (bissc_raw, crc_abs, crc_delta, spi_raw). nan on failure."""
        nan = float("nan")
        if not self.available:
            return nan, nan, nan, nan
        try:
            flat = await self.read_flat()
        except Exception:
            # One resync attempt, then give up on this sample rather than
            # stalling the control loop.
            await self.resync()
            self.read_failures += 1
            return nan, nan, nan, nan
        self.last_flat = flat

        def num(key):
            if key is None or key not in flat:
                return nan
            try:
                return float(flat[key])
            except (TypeError, ValueError):
                return nan

        crc = num(self.key_crc)
        delta = (crc - self.baseline_crc
                 if not (math.isnan(crc) or math.isnan(self.baseline_crc)) else nan)
        return num(self.key_bissc), crc, delta, num(self.key_spi)
