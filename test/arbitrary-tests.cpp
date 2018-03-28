/*
 * Copyright (c) 2018, Philipp Classen All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "catch.hpp"
#include "cppqc/Arbitrary.h"

#include <random>
#include <unordered_set>

using namespace cppqc;

namespace {

constexpr int SAMPLES = 10000;

constexpr double COINFLIP_TOLERANCE = 0.3;
static_assert(COINFLIP_TOLERANCE < 0.5, "0.5 is the theoretical limit");

constexpr double SEEN_VALUES_TOLERANCE = 0.6;
static_assert(SEEN_VALUES_TOLERANCE < 1.0, "1.0 is the theoretical limit");

template <typename T>
struct TestArbitrary {
  void operator()() const {
    RngEngine rng;
    std::unordered_set<T> seen;
    std::array<int, 2> positiveNegative = {0, 0};

    Arbitrary<T> x;
    for (std::size_t size = 0; size < SAMPLES; size++) {
      auto value = x.unGen(rng, size);
      seen.insert(value);
      ++positiveNegative[value >= 0 ? 1 : 0];
    }

    // assumption: uniform distribution
    // (if you take N numbers, you will hit on average about 63% of the buckets)
    //
    // If the underlying distribution is not uniform, it will be worse,
    // but the tolerance factor in the comparsion should fix that.
    std::size_t maxExpectedValues = 0.63 * SAMPLES;
    if (std::numeric_limits<T>::is_signed) {
      if (SAMPLES / 2 > std::numeric_limits<T>::max()) {
        maxExpectedValues =
            static_cast<std::size_t>(std::numeric_limits<T>::max()) * 2;
      }
    } else {
      if (SAMPLES > std::numeric_limits<T>::max()) {
        maxExpectedValues = std::numeric_limits<T>::max();
      }
    }

    // exception: char is specialized to printable characters,
    // which are always positive and do not start from 0
    if (!std::is_same<T, char>::value) {
      REQUIRE(seen.size() > SEEN_VALUES_TOLERANCE * maxExpectedValues);
    }

    if (std::numeric_limits<T>::is_signed && !std::is_same<T, char>::value) {
      REQUIRE(positiveNegative[0] > COINFLIP_TOLERANCE * SAMPLES);
      REQUIRE(positiveNegative[1] > COINFLIP_TOLERANCE * SAMPLES);
    } else {
      REQUIRE(positiveNegative[1] == SAMPLES);
      REQUIRE(positiveNegative[0] == 0);
    }
  };
};

template <>
struct TestArbitrary<bool> {
  void operator()() const {
    RngEngine rng;
    std::array<int, 2> positiveNegative = {0, 0};

    Arbitrary<bool> x;
    for (std::size_t size = 0; size < SAMPLES; size++) {
      ++positiveNegative[x.unGen(rng, size) ? 1 : 0];
    }

    REQUIRE(positiveNegative[0] > COINFLIP_TOLERANCE * SAMPLES);
    REQUIRE(positiveNegative[1] > COINFLIP_TOLERANCE * SAMPLES);
  };
};

}  // namespace

TEST_CASE("Arbitrary<bool>", "[arbitrary][bool]") {
  TestArbitrary<bool>{}();
}

TEST_CASE("Arbitrary<signed char>", "[arbitrary][signed char]") {
  TestArbitrary<signed char>{}();
}

TEST_CASE("Arbitrary<unsigned char>", "[arbitrary][unsigned char]") {
  TestArbitrary<unsigned char>{}();
}

TEST_CASE("Arbitrary<signed short>", "[arbitrary][signed short]") {
  TestArbitrary<signed short>{}();
}

TEST_CASE("Arbitrary<unsigned short>", "[arbitrary][unsigned short]") {
  TestArbitrary<unsigned short>{}();
}

TEST_CASE("Arbitrary<signed int>", "[arbitrary][signed int]") {
  TestArbitrary<signed int>{}();
}

TEST_CASE("Arbitrary<unsigned int>", "[arbitrary][unsigned int]") {
  TestArbitrary<unsigned int>{}();
}

TEST_CASE("Arbitrary<signed long>", "[arbitrary][signed long]") {
  TestArbitrary<signed long>{}();
}

TEST_CASE("Arbitrary<unsigned long>", "[arbitrary][unsigned long]") {
  TestArbitrary<unsigned long>{}();
}

TEST_CASE("Arbitrary<signed long long>", "[arbitrary][signed long long]") {
  TestArbitrary<signed long long>{}();
}

TEST_CASE("Arbitrary<unsigned long long>", "[arbitrary][unsigned long long]") {
  TestArbitrary<unsigned long long>{}();
}

TEST_CASE("Arbitrary<char>", "[arbitrary][char]") {
  TestArbitrary<char>{}();
}

TEST_CASE("Arbitrary<wchar_t>", "[arbitrary][wchar_t]") {
  TestArbitrary<wchar_t>{}();
}
