# Copyright 2026 JSK Robotics Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Yoshiki Obinata nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import struct
import zlib

import numpy as np
import pytest

from g1_pico_teleop.wire import DATAGRAM_SIZE, PicoUdpFrame, WireError, decode_frame, encode_frame


def sample_frame() -> PicoUdpFrame:
    body = np.arange(24 * 7, dtype=np.float32).reshape(24, 7) / 10.0
    left = np.arange(26 * 7, dtype=np.float32).reshape(26, 7) / 20.0
    return PicoUdpFrame(
        sequence=42,
        device_timestamp_ns=123_456,
        source_timestamp_ns=234_567,
        body=body,
        left_hand=left,
        right_hand=None,
    )


def test_wire_round_trip_is_fixed_length_and_preserves_optional_hands() -> None:
    original = sample_frame()
    encoded = encode_frame(original)
    decoded = decode_frame(encoded)

    assert len(encoded) == DATAGRAM_SIZE == 2164
    assert decoded.sequence == 42
    assert decoded.device_timestamp_ns == 123_456
    assert decoded.source_timestamp_ns == 234_567
    np.testing.assert_array_equal(decoded.body, original.body)
    np.testing.assert_array_equal(decoded.left_hand, original.left_hand)
    assert decoded.right_hand is None


@pytest.mark.parametrize("payload", [b"", bytes(DATAGRAM_SIZE - 1), bytes(DATAGRAM_SIZE + 1)])
def test_wire_rejects_wrong_length(payload: bytes) -> None:
    with pytest.raises(WireError, match="exactly"):
        decode_frame(payload)


def test_wire_rejects_crc_corruption() -> None:
    encoded = bytearray(encode_frame(sample_frame()))
    encoded[100] ^= 0x80
    with pytest.raises(WireError, match="CRC32"):
        decode_frame(encoded)


def test_wire_rejects_nonfinite_payload_even_with_valid_crc() -> None:
    encoded = bytearray(encode_frame(sample_frame()))
    struct.pack_into("<f", encoded, 32, float("nan"))
    struct.pack_into("<I", encoded, len(encoded) - 4, zlib.crc32(encoded[:-4]) & 0xFFFFFFFF)
    with pytest.raises(WireError, match="NaN or infinity"):
        decode_frame(encoded)


def test_wire_rejects_invalid_input_before_encoding() -> None:
    body = np.zeros((24, 7), dtype=np.float32)
    body[0, 0] = np.inf
    with pytest.raises(WireError, match="NaN or infinity"):
        PicoUdpFrame(0, 0, 0, body)
    with pytest.raises(WireError, match="shape"):
        PicoUdpFrame(0, 0, 0, np.zeros((23, 7), dtype=np.float32))
    with pytest.raises(WireError, match="unsigned"):
        PicoUdpFrame(-1, 0, 0, np.zeros((24, 7), dtype=np.float32))
