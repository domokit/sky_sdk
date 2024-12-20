// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/enum_variant.h"

#include <wrl/client.h>
#include <wrl/implements.h>

#include "base/win/scoped_com_initializer.h"
#include "base/win/scoped_variant.h"
#include "testing/gtest/include/gtest/gtest.h"

namespace base {
namespace win {

TEST(EnumVariantTest, EmptyEnumVariant) {
  ScopedCOMInitializer com_initializer;

  Microsoft::WRL::ComPtr<EnumVariant> ev = Microsoft::WRL::Make<EnumVariant>(0);
  Microsoft::WRL::ComPtr<IEnumVARIANT> ienumvariant;
  ASSERT_TRUE(SUCCEEDED(ev->QueryInterface(IID_PPV_ARGS(&ienumvariant))));

  {
    base::win::ScopedVariant out_element;
    ULONG out_received = 0;
    EXPECT_EQ(S_FALSE, ev->Next(1, out_element.Receive(), &out_received));
    EXPECT_EQ(0u, out_received);
  }

  EXPECT_EQ(S_FALSE, ev->Skip(1));

  EXPECT_EQ(S_OK, ev->Reset());

  Microsoft::WRL::ComPtr<IEnumVARIANT> ev2;
  EXPECT_EQ(S_OK, ev->Clone(&ev2));

  EXPECT_NE(nullptr, ev2);
  EXPECT_NE(ev, ev2);
  EXPECT_EQ(S_FALSE, ev2->Skip(1));
  EXPECT_EQ(S_OK, ev2->Reset());
}

TEST(EnumVariantTest, SimpleEnumVariant) {
  ScopedCOMInitializer com_initializer;

  Microsoft::WRL::ComPtr<EnumVariant> ev = Microsoft::WRL::Make<EnumVariant>(3);
  ev->ItemAt(0)->vt = VT_I4;
  ev->ItemAt(0)->lVal = 10;
  ev->ItemAt(1)->vt = VT_I4;
  ev->ItemAt(1)->lVal = 20;
  ev->ItemAt(2)->vt = VT_I4;
  ev->ItemAt(2)->lVal = 30;

  // Get elements one at a time from index 0 and 2.
  base::win::ScopedVariant out_element_0;
  ULONG out_received_0 = 0;
  EXPECT_EQ(S_OK, ev->Next(1, out_element_0.Receive(), &out_received_0));
  EXPECT_EQ(1u, out_received_0);
  EXPECT_EQ(VT_I4, out_element_0.ptr()->vt);
  EXPECT_EQ(10, out_element_0.ptr()->lVal);

  EXPECT_EQ(S_OK, ev->Skip(1));

  base::win::ScopedVariant out_element_2;
  ULONG out_received_2 = 0;
  EXPECT_EQ(S_OK, ev->Next(1, out_element_2.Receive(), &out_received_2));
  EXPECT_EQ(1u, out_received_2);
  EXPECT_EQ(VT_I4, out_element_2.ptr()->vt);
  EXPECT_EQ(30, out_element_2.ptr()->lVal);

  base::win::ScopedVariant placeholder_variant;
  EXPECT_EQ(S_FALSE, ev->Next(1, placeholder_variant.Receive(), nullptr));

  // Verify the reset works for the next step.
  ASSERT_EQ(S_OK, ev->Reset());

  // Get all elements at once.
  VARIANT out_elements[3];
  ULONG out_received_multiple;
  for (int i = 0; i < 3; ++i)
    ::VariantInit(&out_elements[i]);
  EXPECT_EQ(S_OK, ev->Next(3, out_elements, &out_received_multiple));
  EXPECT_EQ(3u, out_received_multiple);
  EXPECT_EQ(VT_I4, out_elements[0].vt);
  EXPECT_EQ(10, out_elements[0].lVal);
  EXPECT_EQ(VT_I4, out_elements[1].vt);
  EXPECT_EQ(20, out_elements[1].lVal);
  EXPECT_EQ(VT_I4, out_elements[2].vt);
  EXPECT_EQ(30, out_elements[2].lVal);
  for (int i = 0; i < 3; ++i)
    ::VariantClear(&out_elements[i]);

  base::win::ScopedVariant placeholder_variant_multiple;
  EXPECT_EQ(S_FALSE,
            ev->Next(1, placeholder_variant_multiple.Receive(), nullptr));
}

TEST(EnumVariantTest, Clone) {
  ScopedCOMInitializer com_initializer;

  Microsoft::WRL::ComPtr<EnumVariant> ev = Microsoft::WRL::Make<EnumVariant>(3);
  ev->ItemAt(0)->vt = VT_I4;
  ev->ItemAt(0)->lVal = 10;
  ev->ItemAt(1)->vt = VT_I4;
  ev->ItemAt(1)->lVal = 20;
  ev->ItemAt(2)->vt = VT_I4;
  ev->ItemAt(2)->lVal = 30;

  // Clone it.
  Microsoft::WRL::ComPtr<IEnumVARIANT> ev2;
  EXPECT_EQ(S_OK, ev->Clone(&ev2));
  EXPECT_TRUE(ev2 != nullptr);

  VARIANT out_elements[3];
  for (int i = 0; i < 3; ++i)
    ::VariantInit(&out_elements[i]);
  EXPECT_EQ(S_OK, ev2->Next(3, out_elements, nullptr));
  EXPECT_EQ(VT_I4, out_elements[0].vt);
  EXPECT_EQ(10, out_elements[0].lVal);
  EXPECT_EQ(VT_I4, out_elements[1].vt);
  EXPECT_EQ(20, out_elements[1].lVal);
  EXPECT_EQ(VT_I4, out_elements[2].vt);
  EXPECT_EQ(30, out_elements[2].lVal);
  for (int i = 0; i < 3; ++i)
    ::VariantClear(&out_elements[i]);
}

}  // namespace win
}  // namespace base
