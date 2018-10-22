/*
 * Copyright (c) 2010, Gregory Rogers All rights reserved.
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
    static constexpr bool value =
        IsOneOf<T, short, int, long, long long, unsigned short, unsigned int, unsigned long, unsigned long long>::value;
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
  using DistributionType = typename std::conditional<IntDistributionSupported<Integral>::value, Integral, int>::type;
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
struct Arbitrary {
  using unGenType = std::function<T(RngEngine&, std::size_t)>;
  using shrinkType = std::function<std::vector<T>(T)>;

  static const unGenType unGen;
  static const shrinkType shrink;
};

/*
 * specialize ArbitraryImpl and implement the members:
 *     static const Arbitrary<T>::unGenType unGen;
 *     static const Arbitrary<T>::shrinkType shrink;
 */
template <class T>
struct ArbitraryImpl {
  // no default implementation - users must specialize ArbitraryImpl
  // and give an implementation of unGen and shrink. If they do not
  // and they try to use Arbitrary<TheirClass>, a compile error will result.
};

// Note: The call is wrapped in a function to avoid issues
//       with static ordering when ArbitraryImpl is defined
//       in another compilation unit. Do not simplify it
//       by replacing it with an assignment.
template <class T>
const typename Arbitrary<T>::unGenType Arbitrary<T>::unGen =
    [](RngEngine& rng, std::size_t size) {
      return ArbitraryImpl<T>::unGen(rng, size);
    };

// (function call is needed: see above)
template <class T>
const typename Arbitrary<T>::shrinkType Arbitrary<T>::shrink =
    [](const T& v) { return ArbitraryImpl<T>::shrink(v); };

// included specializations

inline bool arbitraryBool(RngEngine& rng, std::size_t /*size*/) {
  return std::uniform_int_distribution<int>(0, 1)(rng) != 0;
}
inline std::vector<bool> shrinkBool(bool x) {
  std::vector<bool> ret;
  if (x)
    ret.push_back(false);
  return ret;
}
template <>
struct ArbitraryImpl<bool> {
  static const Arbitrary<bool>::unGenType unGen;
  static const Arbitrary<bool>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<signed char> {
  static const Arbitrary<signed char>::unGenType unGen;
  static const Arbitrary<signed char>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<unsigned char> {
  static const Arbitrary<unsigned char>::unGenType unGen;
  static const Arbitrary<unsigned char>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<signed short> {
  static const Arbitrary<signed short>::unGenType unGen;
  static const Arbitrary<signed short>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<unsigned short> {
  static const Arbitrary<unsigned short>::unGenType unGen;
  static const Arbitrary<unsigned short>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<signed int> {
  static const Arbitrary<signed int>::unGenType unGen;
  static const Arbitrary<signed int>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<unsigned int> {
  static const Arbitrary<unsigned int>::unGenType unGen;
  static const Arbitrary<unsigned int>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<signed long> {
  static const Arbitrary<signed long>::unGenType unGen;
  static const Arbitrary<signed long>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<unsigned long> {
  static const Arbitrary<unsigned long>::unGenType unGen;
  static const Arbitrary<unsigned long>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<signed long long> {
  static const Arbitrary<signed long long>::unGenType unGen;
  static const Arbitrary<signed long long>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<unsigned long long> {
  static const Arbitrary<unsigned long long>::unGenType unGen;
  static const Arbitrary<unsigned long long>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<float> {
  static const Arbitrary<float>::unGenType unGen;
  static const Arbitrary<float>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<double> {
  static const Arbitrary<double>::unGenType unGen;
  static const Arbitrary<double>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<long double> {
  static const Arbitrary<long double>::unGenType unGen;
  static const Arbitrary<long double>::shrinkType shrink;
};

inline char arbitraryChar(RngEngine& rng, std::size_t) {
  std::uniform_int_distribution<int> dist{0x20, 0x7f};
  return static_cast<char>(dist(rng));
}
inline std::vector<char> shrinkChar(char c) {
  const char possShrinks[] = {'a', 'b', 'c', 'A', 'B',  'C',
                              '1', '2', '3', ' ', '\n', '\0'};
  std::vector<char> ret;
  for (auto possShrink : possShrinks) {
    if (possShrink < c)
      ret.push_back(possShrink);
  }
  if (isupper(c) && std::find(possShrinks, possShrinks + sizeof(possShrinks),
                              tolower(c)) == possShrinks + sizeof(possShrinks))
    ret.push_back(tolower(c));
  return ret;
}
template <>
struct ArbitraryImpl<char> {
  static const Arbitrary<char>::unGenType unGen;
  static const Arbitrary<char>::shrinkType shrink;
};

template <>
struct ArbitraryImpl<wchar_t> {
  static const Arbitrary<wchar_t>::unGenType unGen;
  static const Arbitrary<wchar_t>::shrinkType shrink;
};

template <class String>
String arbitraryString(RngEngine& rng, std::size_t size) {
  std::uniform_int_distribution<std::size_t> dist{0, size};
  std::size_t n = dist(rng);
  String ret;
  ret.reserve(n);
  while (n-- > 0)
    ret.push_back(Arbitrary<typename String::value_type>::unGen(rng, size));
  return ret;
}
template <class String>
std::vector<String> shrinkString(const String& x) {
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
template <class CharT, class Traits, class Alloc>
struct ArbitraryImpl<std::basic_string<CharT, Traits, Alloc>> {
  static const typename Arbitrary<
      std::basic_string<CharT, Traits, Alloc>>::unGenType unGen;
  static const typename Arbitrary<
      std::basic_string<CharT, Traits, Alloc>>::shrinkType shrink;
};
template <class CharT, class Traits, class Alloc>
const typename Arbitrary<std::basic_string<CharT, Traits, Alloc>>::unGenType
    ArbitraryImpl<std::basic_string<CharT, Traits, Alloc>>::unGen =
        arbitraryString<std::basic_string<CharT, Traits, Alloc>>;
template <class CharT, class Traits, class Alloc>
const typename Arbitrary<std::basic_string<CharT, Traits, Alloc>>::shrinkType
    ArbitraryImpl<std::basic_string<CharT, Traits, Alloc>>::shrink =
        shrinkString<std::basic_string<CharT, Traits, Alloc>>;

template <class PairType>
PairType arbitraryPair(RngEngine& rng, std::size_t size) {
  return PairType(Arbitrary<typename PairType::first_type>::unGen(rng, size),
                  Arbitrary<typename PairType::second_type>::unGen(rng, size));
}
template <class PairType>
std::vector<PairType> shrinkPair(const PairType& x) {
  using FirstType = typename PairType::first_type;
  using SecondType = typename PairType::second_type;
  std::vector<FirstType> shrinks1 = Arbitrary<FirstType>::shrink(x.first);
  std::vector<SecondType> shrinks2 = Arbitrary<SecondType>::shrink(x.second);
  std::vector<PairType> ret;
  ret.reserve(shrinks1.size() + shrinks2.size());
  for (auto it = shrinks1.begin(); it != shrinks1.end(); ++it) {
    ret.push_back(PairType(*it, x.second));
  }
  for (auto it = shrinks2.begin(); it != shrinks2.end(); ++it) {
    ret.push_back(PairType(x.first, *it));
  }
  return ret;
}
template <class T1, class T2>
struct ArbitraryImpl<std::pair<T1, T2>> {
  static const typename Arbitrary<std::pair<T1, T2>>::unGenType unGen;
  static const typename Arbitrary<std::pair<T1, T2>>::shrinkType shrink;
};
template <class T1, class T2>
const typename Arbitrary<std::pair<T1, T2>>::unGenType
    ArbitraryImpl<std::pair<T1, T2>>::unGen = arbitraryPair<std::pair<T1, T2>>;
template <class T1, class T2>
const typename Arbitrary<std::pair<T1, T2>>::shrinkType
    ArbitraryImpl<std::pair<T1, T2>>::shrink = shrinkPair<std::pair<T1, T2>>;

template <typename T>
struct ArbitraryImpl<std::vector<T>> {
  static const typename Arbitrary<std::vector<T>>::unGenType unGen;
  static const typename Arbitrary<std::vector<T>>::shrinkType shrink;
};

template <typename T>
const typename Arbitrary<std::vector<T>>::unGenType
    ArbitraryImpl<std::vector<T>>::unGen =
        [](RngEngine& rng, std::size_t size) {
          const auto& vectorGenerator = listOf<T>();
          return vectorGenerator.unGen(rng, size);
        };

template <typename T>
const typename Arbitrary<std::vector<T>>::shrinkType
    ArbitraryImpl<std::vector<T>>::shrink = [](const std::vector<T>& v) {
      const auto& vectorGenerator = listOf<T>();
      return vectorGenerator.shrink(v);
    };

template <typename T, std::size_t N>
struct ArbitraryImpl<std::array<T, N>> {
  static const typename Arbitrary<std::array<T, N>>::unGenType unGen;
  static const typename Arbitrary<std::array<T, N>>::shrinkType shrink;
};

/// Note: N is the fixed size of the array.
/// It differs from the "size" param in the "unGen" function:
///
/// If "size" is increased, the output array will still contain
/// N elements, but each element will, in general, be more complex.
template <typename T, std::size_t N>
const typename Arbitrary<std::array<T, N>>::unGenType
    ArbitraryImpl<std::array<T, N>>::unGen =
        [](RngEngine& rng, std::size_t size) {
          const auto& arrayGenerator = arrayOf<T, N>();
          return arrayGenerator.unGen(rng, size);
        };

template <typename T, std::size_t N>
const typename Arbitrary<std::array<T, N>>::shrinkType
    ArbitraryImpl<std::array<T, N>>::shrink = [](const std::array<T, N>& arr) {
      const auto& arrayGenerator = arrayOf<T, N>();
      return arrayGenerator.shrink(arr);
    };

}  // namespace cppqc

#endif
