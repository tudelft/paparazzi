#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2026 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
"""Small-generation GF(256) coding primitives for mesh experiments.

AC_ID values are opaque dictionary keys. Their numeric values never imply a
storage index, membership order, radio proximity, or forwarding priority.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import random
from typing import Dict, Iterable, List, Sequence, Tuple


def validate_ac_ids(ac_ids: Iterable[int]) -> Tuple[int, ...]:
    """Validate Paparazzi node IDs, preserving the caller's ordering."""
    result = tuple(ac_ids)
    if not result:
        raise ValueError("at least one AC_ID is required")
    if any(ac_id < 0 or ac_id > 254 for ac_id in result):
        raise ValueError("AC_IDs must be in 0..254; 255 is broadcast")
    if len(set(result)) != len(result):
        raise ValueError("AC_IDs must be distinct")
    return result


@dataclass(frozen=True)
class Membership:
    """Dense local storage with indexes assigned by observation order."""

    ac_ids: Tuple[int, ...]
    index_by_id: Dict[int, int] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        ids = validate_ac_ids(self.ac_ids)
        object.__setattr__(self, "ac_ids", ids)
        object.__setattr__(self, "index_by_id",
                           {ac_id: index for index, ac_id in enumerate(ids)})

    def index(self, ac_id: int) -> int:
        return self.index_by_id[ac_id]

    def contains(self, ac_id: int) -> bool:
        return ac_id in self.index_by_id


# GF(256), x^8 + x^4 + x^3 + x^2 + 1 (0x11d), generator 0x02.
_EXP = [0] * 512
_LOG = [0] * 256
_value = 1
for _index in range(255):
    _EXP[_index] = _value
    _LOG[_value] = _index
    _value <<= 1
    if _value & 0x100:
        _value ^= 0x11D
for _index in range(255, 512):
    _EXP[_index] = _EXP[_index - 255]


def gf_mul(left: int, right: int) -> int:
    if left == 0 or right == 0:
        return 0
    return _EXP[_LOG[left] + _LOG[right]]


def gf_inv(value: int) -> int:
    if value == 0:
        raise ZeroDivisionError("zero has no multiplicative inverse")
    return _EXP[255 - _LOG[value]]


def gf_axpy(destination: bytearray, source: Sequence[int], factor: int) -> None:
    if factor == 0:
        return
    for index, value in enumerate(source):
        destination[index] ^= gf_mul(value, factor)


@dataclass(frozen=True)
class CodedSymbol:
    source_id: int
    generation: int
    coefficients: bytes
    payload: bytes
    sample_time_ms: int


class GenerationDecoder:
    """Incremental reduced-row-echelon decoder for one generation."""

    def __init__(self, source_id: int, generation: int,
                 generation_size: int, symbol_size: int):
        if generation_size < 1 or generation_size > 16:
            raise ValueError("generation_size must be in 1..16")
        if symbol_size < 1 or symbol_size > 223:
            raise ValueError("symbol_size must be in 1..223")
        self.source_id = source_id
        self.generation = generation
        self.generation_size = generation_size
        self.symbol_size = symbol_size
        self.rows: Dict[int, Tuple[bytearray, bytearray]] = {}
        self.newest_sample_ms = 0

    @property
    def rank(self) -> int:
        return len(self.rows)

    @property
    def complete(self) -> bool:
        return self.rank == self.generation_size

    def add(self, symbol: CodedSymbol) -> bool:
        if symbol.source_id != self.source_id or symbol.generation != self.generation:
            raise ValueError("symbol belongs to another generation")
        if len(symbol.coefficients) != self.generation_size:
            raise ValueError("coefficient vector has the wrong size")
        if len(symbol.payload) != self.symbol_size:
            raise ValueError("coded payload has the wrong size")

        coefficients = bytearray(symbol.coefficients)
        payload = bytearray(symbol.payload)
        for pivot in sorted(self.rows):
            factor = coefficients[pivot]
            if factor:
                row_coefficients, row_payload = self.rows[pivot]
                gf_axpy(coefficients, row_coefficients, factor)
                gf_axpy(payload, row_payload, factor)

        pivot = next((index for index, value in enumerate(coefficients) if value), None)
        if pivot is None:
            return False

        inverse = gf_inv(coefficients[pivot])
        coefficients[:] = (gf_mul(value, inverse) for value in coefficients)
        payload[:] = (gf_mul(value, inverse) for value in payload)
        for other_coefficients, other_payload in self.rows.values():
            factor = other_coefficients[pivot]
            if factor:
                gf_axpy(other_coefficients, coefficients, factor)
                gf_axpy(other_payload, payload, factor)
        self.rows[pivot] = (coefficients, payload)
        self.newest_sample_ms = max(self.newest_sample_ms, symbol.sample_time_ms)
        return True

    def decoded(self) -> List[bytes]:
        if not self.complete:
            raise ValueError("generation is not decodable")
        return [bytes(self.rows[pivot][1])
                for pivot in range(self.generation_size)]

    def recode(self, rng: random.Random, density: float = 0.5) -> CodedSymbol:
        if not self.rows:
            raise ValueError("cannot recode an empty generation")
        if density <= 0.0 or density > 1.0:
            raise ValueError("density must be in (0, 1]")
        selected = [pivot for pivot in self.rows if rng.random() < density]
        if not selected:
            selected = [rng.choice(tuple(self.rows))]
        coefficients = bytearray(self.generation_size)
        payload = bytearray(self.symbol_size)
        for pivot in selected:
            factor = rng.randint(1, 255)
            row_coefficients, row_payload = self.rows[pivot]
            gf_axpy(coefficients, row_coefficients, factor)
            gf_axpy(payload, row_payload, factor)
        return CodedSymbol(self.source_id, self.generation,
                           bytes(coefficients), bytes(payload),
                           self.newest_sample_ms)


def systematic_symbols(source_id: int, generation: int,
                       payloads: Sequence[bytes],
                       sample_times_ms: Sequence[int]) -> List[CodedSymbol]:
    if not payloads or len(payloads) > 16:
        raise ValueError("a generation must contain 1..16 payloads")
    if len(sample_times_ms) != len(payloads):
        raise ValueError("one sample time is required per payload")
    symbol_size = len(payloads[0])
    if symbol_size < 1 or any(len(payload) != symbol_size for payload in payloads):
        raise ValueError("all payloads must have the same non-zero size")

    result = []
    for index, payload in enumerate(payloads):
        coefficients = bytearray(len(payloads))
        coefficients[index] = 1
        result.append(CodedSymbol(source_id, generation, bytes(coefficients),
                                  payload, sample_times_ms[index]))
    return result