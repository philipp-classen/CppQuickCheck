/*
 * Copyright (c) 2019, Gregory Rogers and Philipp Claßen
 * All rights reserved.
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

#ifndef CPPQC_ARBITRARY_H
#define CPPQC_ARBITRARY_H

#include "Generator.h"

#include <limits>
#include <random>
#include <type_traits>

namespace cppqc {

/*
 * Type trait for checking if a type is found in a parameter pack.
 */
template <class T, class... Us>
struct IsOneOf;

template <class T, class U, class... Us>
struct IsOneOf<T, U, Us...> {
  static constexpr bool value = IsOneOf<T, Us...>::value;
};

template <class T, class... Us>
struct IsOneOf<T, T, Us...> {
  static constexpr bool value = true;
};

template <class T>
struct IsOneOf<T> {
  static constexpr bool value = false;
};

/*
 * Type trait for checking if a type can be used as the template parameter of
 * std::uniform_int_distribution.
 */
template <class T>
struct IntDistributionSupported {
  static constexpr bool value = IsOneOf<T,
                                        short,
                                        int,
                                        long,
                                        long long,
                                        unsigned short,
                                        unsigned int,
                                        unsigned long,
                                        unsigned long long>::value;
};

// default generators

/*
 * Generates an integral number from a bounded domain defined by
 * the integer type.
 * The number will be chosen uniformly, that means big numbers
 * are equally likely as numbers near 0.
 */
template <class Integral>
Integral arbitraryBoundedIntegral(RngEngine& rng, std::size_t /*size*/) {
  using DistributionType =
      typename std::conditional<IntDistributionSupported<Integral>::value,
                                Integral, int>::type;
  std::uniform_int_distribution<DistributionType> dist{
      std::numeric_limits<Integral>::lowest(),
      std::numeric_limits<Integral>::max()};
  return dist(rng);
}

template <class Integral>
Integral arbitrarySizedIntegral(RngEngine& rng, std::size_t size) {
  if (size > std::numeric_limits<Integral>::max()) {
    return arbitraryBoundedIntegral<Integral>(rng, size);
  }

  std::uniform_int_distribution<Integral> dist(
      std::numeric_limits<Integral>::is_signed ? -Integral(size)
                                               : Integral(size),
      Integral(size));
  return dist(rng);
}

/*
 * Generates an integral number from a bounded domain. The number is
 * chosen from the entire range of the type, but numbers near 0 are
 * generated more often than big numbers.
 */
template <class Integral>
Integral arbitrarySizedBoundedIntegral(RngEngine& rng, std::size_t size) {
  const double limit =
      std::min(double(size), double(std::numeric_limits<Integral>::max()));

  // When dividing by 3, about 99% of the values should be
  // in the interval [-limit, limit]
  std::normal_distribution<> dist{0, 1.0 + limit / 3};
  for (;;) {
    double r = dist(rng);

    if (!std::numeric_limits<Integral>::is_signed) {
      r = std::abs(r);
    }

    if (r >= std::numeric_limits<Integral>::lowest() &&
        r <= std::numeric_limits<Integral>::max()) {
      return static_cast<Integral>(r);
    }
  }
}

template <class Real>
Real arbitrarySizedReal(RngEngine& rng, std::size_t size) {
  std::uniform_real_distribution<Real> dist{-Real(size + 1.0),
                                            Real(size + 1.0)};
  return dist(rng);
}

// default shrinkers

template <class T>
std::vector<T> shrinkNothing(const T&) {
  return std::vector<T>();
}

template <class Integral>
std::vector<Integral> shrinkIntegral(Integral x) {
  std::vector<Integral> ret;
  if (std::numeric_limits<Integral>::is_signed && x < 0) {
    if (x == std::numeric_limits<Integral>::min()) {
      ret.push_back(std::numeric_limits<Integral>::max());
    } else {
      assert(-x > 0);
      ret.push_back(-x);
    }
  }

  for (Integral n = x; n != 0; n /= 2)
    ret.push_back(x - n);
  return ret;
}

template <class Real>
std::vector<Real> shrinkReal(Real x) {
  std::vector<Real> ret;
  if (x == 0)
    return ret;
  if (x < 0)
    ret.push_back(-x);
  ret.push_back(Real(0));

  if (std::isnan(x) && std::abs(x) >= 2) {
    if (std::abs(x) < 1e100) {
      ret.push_back(x / Real(2));
    } else {
      // special case: reduce faster if the numbers are huge
      // (Note: Maybe there is a better heuristic. Looks quite crude.)
      ret.push_back(x / Real(1e20));
    }
  }
  return ret;
}

template <class T>
struct ArbitraryImpl;

// specialize Arbitrary and implement the members:
//     T unGen(RngEngine&, std::size_t);
//     std::vector<T> shrink(T);
//
// If they do not and they try to use Arbitrary<TheirClass>, a compile error
// will result.
template <class T>
struct Arbitrary {
  static T unGen(RngEngine& rng, std::size_t size) {
    return ArbitraryImpl<T>::unGen(rng, size);
  }

  template <typename U>
  static std::vector<T> shrink(U&& x) {
    return ArbitraryImpl<T>::shrink(std::forward<U>(x));
  }
};

template <>
struct ArbitraryImpl<bool> {
  static bool unGen(RngEngine& rng, std::size_t /*size*/) {
    return std::uniform_int_distribution<int>(0, 1)(rng) != 0;
  }

  static std::vector<bool> shrink(bool x) {
    std::vector<bool> ret;
    if (x)
      ret.push_back(false);
    return ret;
  }
};

namespace detail {
template <typename T>
struct ArbitrarySizedBoundedIntegral {
  static T unGen(RngEngine& rng, std::size_t size) {
    return arbitrarySizedBoundedIntegral<T>(rng, size);
  }
  static std::vector<T> shrink(T x) { return shrinkIntegral(x); }
};

template <typename T>
struct ArbitrarySizedReal {
  static T unGen(RngEngine& rng, std::size_t size) {
    return arbitrarySizedReal<T>(rng, size);
  }
  static std::vector<T> shrink(T x) { return shrinkReal(x); }
};

template <typename String>
struct ArbitraryString {
  static String unGen(RngEngine& rng, std::size_t size) {
    String ret;
    std::uniform_int_distribution<std::size_t> dist{0, size};
    std::size_t n = dist(rng);
    ret.reserve(n);
    while (n-- > 0)
      ret.push_back(Arbitrary<typename String::value_type>::unGen(rng, size));
    return ret;
  }
  static std::vector<String> shrink(const String& x) {
    std::vector<String> ret;
    ret.reserve(x.size());
    for (auto it = x.begin(); it != x.end(); ++it) {
      ret.push_back(String());
      ret.back().reserve(x.size() - 1);
      ret.back().insert(ret.back().end(), x.begin(), it);
      ret.back().insert(ret.back().end(), it + 1, x.end());
    }
    return ret;
  }
};

template <typename PairType>
struct ArbitraryPair {
  static PairType unGen(RngEngine& rng, std::size_t size) {
    return PairType(
        Arbitrary<typename PairType::first_type>::unGen(rng, size),
        Arbitrary<typename PairType::second_type>::unGen(rng, size));
  }
  static std::vector<PairType> shrink(const PairType& x) {
    std::vector<PairType> ret;
    using FirstType = typename PairType::first_type;
    using SecondType = typename PairType::second_type;
    std::vector<FirstType> shrinks1 = Arbitrary<FirstType>::shrink(x.first);
    std::vector<SecondType> shrinks2 = Arbitrary<SecondType>::shrink(x.second);
    ret.reserve(shrinks1.size() + shrinks2.size());
    for (auto it = shrinks1.begin(); it != shrinks1.end(); ++it) {
      ret.push_back(PairType(*it, x.second));
    }
    for (auto it = shrinks2.begin(); it != shrinks2.end(); ++it) {
      ret.push_back(PairType(x.first, *it));
    }
    return ret;
  }
};

}  // namespace detail

// builtin integer types
template <>
struct ArbitraryImpl<signed char>
    : detail::ArbitrarySizedBoundedIntegral<signed char> {};
template <>
struct ArbitraryImpl<unsigned char>
    : detail::ArbitrarySizedBoundedIntegral<unsigned char> {};
template <>
struct ArbitraryImpl<signed short>
    : detail::ArbitrarySizedBoundedIntegral<signed short> {};
template <>
struct ArbitraryImpl<unsigned short>
    : detail::ArbitrarySizedBoundedIntegral<unsigned short> {};
template <>
struct ArbitraryImpl<signed int>
    : detail::ArbitrarySizedBoundedIntegral<signed int> {};
template <>
struct ArbitraryImpl<unsigned int>
    : detail::ArbitrarySizedBoundedIntegral<unsigned int> {};
template <>
struct ArbitraryImpl<signed long>
    : detail::ArbitrarySizedBoundedIntegral<signed long> {};
template <>
struct ArbitraryImpl<unsigned long>
    : detail::ArbitrarySizedBoundedIntegral<unsigned long> {};
template <>
struct ArbitraryImpl<signed long long>
    : detail::ArbitrarySizedBoundedIntegral<signed long long> {};
template <>
struct ArbitraryImpl<unsigned long long>
    : detail::ArbitrarySizedBoundedIntegral<unsigned long long> {};

// builtin floating point types
template <>
struct ArbitraryImpl<float> : detail::ArbitrarySizedReal<float> {};
template <>
struct ArbitraryImpl<double> : detail::ArbitrarySizedReal<double> {};
template <>
struct ArbitraryImpl<long double> : detail::ArbitrarySizedReal<long double> {};

// builtin character types
template <>
struct ArbitraryImpl<char> {
  static char unGen(RngEngine& rng, std::size_t /*size*/) {
    std::uniform_int_distribution<int> dist{0x20, 0x7f};
    return static_cast<char>(dist(rng));
  }
  static std::vector<char> shrink(char c) {
    std::vector<char> ret;
    constexpr char possShrinks[] = {'a', 'b', 'c', 'A', 'B',  'C',
                                    '1', '2', '3', ' ', '\n', '\0'};
    for (auto possShrink : possShrinks) {
      if (possShrink < c)
        ret.push_back(possShrink);
    }
    if (isupper(c) &&
        std::find(possShrinks, possShrinks + sizeof(possShrinks), tolower(c)) ==
            possShrinks + sizeof(possShrinks)) {
      ret.push_back(tolower(c));
    }
    return ret;
  }
};

template <>
struct ArbitraryImpl<wchar_t> : detail::ArbitrarySizedBoundedIntegral<wchar_t> {
};

// builtin string types
template <class CharT, class Traits, class Alloc>
struct ArbitraryImpl<std::basic_string<CharT, Traits, Alloc>>
    : detail::ArbitraryString<std::basic_string<CharT, Traits, Alloc>> {};

// std::pair related types
template <class T1, class T2>
struct ArbitraryImpl<std::pair<T1, T2>>
    : detail::ArbitraryPair<std::pair<T1, T2>> {};

template <class T>
struct ArbitraryImpl<std::vector<T>> {
  static std::vector<T> unGen(RngEngine& rng, std::size_t size) {
    return listOf<T>().unGen(rng, size);
  }

  static std::vector<std::vector<T>> shrink(const std::vector<T>& v) {
    return listOf<T>().shrink(v);
  }
};

/// Note: N is the fixed size of the array.
/// It differs from the "size" param in the "unGen" function:
///
/// If "size" is increased, the output array will still contain
/// N elements, but each element will, in general, be more complex.
template <class T, std::size_t N>
struct ArbitraryImpl<std::array<T, N>> {
  static std::array<T, N> unGen(RngEngine& rng, std::size_t size) {
    return arrayOf<T, N>().unGen(rng, size);
  }

  static std::vector<std::array<T, N>> shrink(const std::array<T, N>& v) {
    return arrayOf<T, N>().shrink(v);
  }
};

}  // namespace cppqc
#endif
