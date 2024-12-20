// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/base32.h"

#include <iostream>

#include "gtest/gtest.h"

TEST(Base32Test, CanEncode) {
  {
    auto result = fml::Base32Encode("hello");
    ASSERT_TRUE(result.first);
    ASSERT_EQ(result.second, "NBSWY3DP");
  }

  {
    auto result = fml::Base32Encode("helLo");
    ASSERT_TRUE(result.first);
    ASSERT_EQ(result.second, "NBSWYTDP");
  }

  {
    auto result = fml::Base32Encode("");
    ASSERT_TRUE(result.first);
    ASSERT_EQ(result.second, "");
  }

  {
    auto result = fml::Base32Encode("1");
    ASSERT_TRUE(result.first);
    ASSERT_EQ(result.second, "GE");
  }

  {
    auto result = fml::Base32Encode("helLo");
    ASSERT_TRUE(result.first);
    ASSERT_EQ(result.second, "NBSWYTDP");
  }

  {
    auto result = fml::Base32Encode("\xff\xfe\x7f\x80\x81");
    ASSERT_TRUE(result.first);
    ASSERT_EQ(result.second, "777H7AEB");
  }
}

TEST(Base32Test, CanEncodeDecodeStrings) {
  std::vector<std::string> strings = {"hello", "helLo", "", "1", "\0"};
  for (size_t i = 0; i < strings.size(); i += 1) {
    auto encode_result = fml::Base32Encode(strings[i]);
    ASSERT_TRUE(encode_result.first);
    auto decode_result = fml::Base32Decode(encode_result.second);
    ASSERT_TRUE(decode_result.first);
    const std::string& decoded = decode_result.second;
    std::string decoded_string(decoded.data(), decoded.size());
    ASSERT_EQ(strings[i], decoded_string);
  }
}

TEST(Base32Test, DecodeReturnsFalseForInvalideInput) {
  // "B" is invalid because it has a non-zero padding.
  std::vector<std::string> invalid_inputs = {"a", "1", "9", "B"};
  for (const std::string& input : invalid_inputs) {
    auto decode_result = fml::Base32Decode(input);
    if (decode_result.first) {
      std::cout << "Base32Decode should return false on " << input << std::endl;
    }
    ASSERT_FALSE(decode_result.first);
  }
}

TEST(Base32Test, CanDecodeSkSLKeys) {
  std::vector<std::string> inputs = {
      "CAZAAAACAAAAADQAAAABKAAAAAJQAAIA7777777777776EYAAEAP777777777777AAAAAABA"
      "ABTAAAAAAAAAAAAAAABAAAAAGQAGGAA",
      "CAZAAAICAAAAAAAAAAADOAAAAAJQAAIA777777Y4AAKAAEYAAEAP777777777777EAAGMAAA"
      "AAAAAAAAAAACQACNAAAAAAAAAAAAAAACAAAAAPAAMMAA",
      "CAZACAACAAAABAYACAAAAAAAAAJQAAIADQABIAH777777777777RQAAOAAAAAAAAAAAAAABE"
      "AANQAAAAAAAAAAAAAAYAAJYAAAAAAAANAAAQAAAAAAAEAAAHAAAAAAAAAAAAAAANAAAQAAAA"
      "AAAFIADKAAAAAAAAAAAAAAACAAAAAZAAMMAA"};
  for (const std::string& input : inputs) {
    auto decode_result = fml::Base32Decode(input);
    if (!decode_result.first) {
      std::cout << "Base32Decode should return true on " << input << std::endl;
    }
    ASSERT_TRUE(decode_result.first);
    auto encode_result = fml::Base32Encode(decode_result.second);
    ASSERT_TRUE(encode_result.first);
    ASSERT_EQ(encode_result.second, input);
  }
}
