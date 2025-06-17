"""
Created on 27 Sep 2020

:author: semuadmin
:copyright: SEMU Consulting © 2020
:license: BSD 3-Clause
"""

from pynmeagps import (
    SocketWrapper,
    bearing,
    ecef2llh,
    haversine,
    latlon2dmm,
    latlon2dms,
    llh2ecef,
    llh2iso6709,
    planar,
)

# Version
__version__ = "1.0.0"

# Core Constants
UBX_HDR = b"\xb5\x62"
GET = 0
SET = 1
POLL = 2
SETPOLL = 3
VALNONE = 0
VALCKSUM = 1
NMEA_PROTOCOL = 1
UBX_PROTOCOL = 2
RTCM3_PROTOCOL = 4
ERR_RAISE = 2
ERR_LOG = 1
ERR_IGNORE = 0

# Scaling factors
SCAL9 = 1e-9
SCAL8 = 1e-8
SCAL7 = 1e-7
SCAL6 = 1e-6
SCAL5 = 1e-5
SCAL4 = 1e-4
SCAL3 = 1e-3
SCAL2 = 1e-2
SCAL1 = 1e-1
SCALROUND = 12

# Attribute Types
A250 = "A250"
A256 = "A256"
C2 = "C002"
C6 = "C006"
C10 = "C010"
C30 = "C030"
C32 = "C032"
CH = "CH"
E1 = "E001"
E2 = "E002"
E4 = "E004"
I1 = "I001"
I2 = "I002"
I4 = "I004"
I8 = "I008"
L = "L001"
U1 = "U001"
U2 = "U002"
U3 = "U003"
U4 = "U004"
U5 = "U005"
U6 = "U006"
U7 = "U007"
U8 = "U008"
U9 = "U009"
U10 = "U010"
U11 = "U011"
U12 = "U012"
U15 = "U015"
U16 = "U016"
U20 = "U020"
U22 = "U022"
U23 = "U023"
U24 = "U024"
U32 = "U032"
U40 = "U040"
U64 = "U064"
X1 = "X001"
X2 = "X002"
X4 = "X004"
X6 = "X006"
X8 = "X008"
X24 = "X024"
R4 = "R004"
R8 = "R008"

# Message Classes
UBX_CLASSES = {
    b"\x01": "NAV",
    b"\x02": "RXM",
    b"\x03": "TRK",
    b"\x04": "INF",
    b"\x05": "ACK",
    b"\x06": "CFG",
    b"\x08": "TUN",
    b"\x09": "UPD",
    b"\x0a": "MON",
    b"\x0b": "AID",
    b"\x0c": "DBG",
    b"\x0d": "TIM",
    b"\x10": "ESF",
    b"\x13": "MGA",
    b"\x21": "LOG",
    b"\x27": "SEC",
    b"\x28": "HNR",
    b"\x29": "NAV2",
    b"\xf0": "NMEA-Standard",
    b"\xf1": "NMEA-Proprietary",
    b"\xf4": "RTCM2",
    b"\xf5": "RTCM3",
    b"\xf6": "SPARTN",
    b"\xf7": "NMEA-NAV2",
    b"\x66": "FOO",
}


# Exceptions
class UBXMessageError(Exception):
    """UBX Message Error Class."""


class UBXParseError(Exception):
    """UBX Parse Error Class."""


class UBXStreamError(Exception):
    """UBX Stream Error Class."""


class UBXTypeError(Exception):
    """UBX Type Error Class."""


class GNSSStreamError(Exception):
    """GNSS Stream Error Class."""


class ParameterError(Exception):
    """Parameter Error Class."""


# Helper functions
def bytes2val(bytesval: bytes, bitfield: bool = False) -> int:
    """Convert bytes to integer value."""
    if bitfield:
        return int.from_bytes(bytesval, byteorder="little")
    return int.from_bytes(bytesval, byteorder="big")


def val2bytes(val: int, length: int, bitfield: bool = False) -> bytes:
    """Convert integer value to bytes."""
    if bitfield:
        return val.to_bytes(length, byteorder="little")
    return val.to_bytes(length, byteorder="big")


# UBXMessage class
class UBXMessage:
    """UBX Message Class."""

    def __init__(self, msgclass: int, msgid: int, payload: bytes = None):
        self.msgclass = msgclass
        self.msgid = msgid
        self.payload = payload if payload else b""
        self.length = len(self.payload)

    def __str__(self) -> str:
        """String representation of UBX message."""
        return f"UBX-{self.msgclass:02X}{self.msgid:02X}"


# UBXReader class
class UBXReader:
    """UBX Reader Class."""

    def __init__(self, stream, **kwargs):
        self.stream = stream
        self.msgclass = kwargs.get("msgclass", None)
        self.msgid = kwargs.get("msgid", None)
        self.quitonerror = kwargs.get("quitonerror", False)
        self.validate = kwargs.get("validate", True)
        self.msgmode = kwargs.get("msgmode", 0)
        self.ubxonly = kwargs.get("ubxonly", False)
        self.parsebitfield = kwargs.get("parsebitfield", True)
        self.scaling = kwargs.get("scaling", True)
        self.labelmsm = kwargs.get("labelmsm", True)
        self.hpnmeamode = kwargs.get("hpnmeamode", 0)

    def read(self) -> UBXMessage:
        """Read a UBX message from the stream."""
        try:
            # Read header
            header = self.stream.read(2)
            if header != UBX_HDR:
                return None

            # Read message class and ID
            msgclass = int.from_bytes(self.stream.read(1), byteorder="little")
            msgid = int.from_bytes(self.stream.read(1), byteorder="little")

            # Read length
            length = int.from_bytes(self.stream.read(2), byteorder="little")

            # Read payload
            payload = self.stream.read(length)

            # Read checksum
            checksum = self.stream.read(2)

            # Create message
            return UBXMessage(msgclass, msgid, payload)

        except Exception as e:
            if self.quitonerror:
                raise UBXStreamError(f"Error reading UBX message: {e}")
            return None


# Export version
version = __version__
