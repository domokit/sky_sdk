// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// If canonicalization uses deep structural hashing without memoizing, this
// will exhibit superlinear time.

// Compare with Dart version of this test at:
// https://github.com/dart-lang/sdk/blob/ca3ad264a64937d5d336cd04dbf2746d1b7d8fc4/tests/language_2/canonicalize/hashing_memoize_instance_test.dart

class Box {
  const Box(this.content1, this.content2);
  final Object? content1; // ignore: unreachable_from_main
  final Object? content2; // ignore: unreachable_from_main
}

const Box box1_0 = Box(null, null);
const Box box1_1 = Box(box1_0, box1_0);
const Box box1_2 = Box(box1_1, box1_1);
const Box box1_3 = Box(box1_2, box1_2);
const Box box1_4 = Box(box1_3, box1_3);
const Box box1_5 = Box(box1_4, box1_4);
const Box box1_6 = Box(box1_5, box1_5);
const Box box1_7 = Box(box1_6, box1_6);
const Box box1_8 = Box(box1_7, box1_7);
const Box box1_9 = Box(box1_8, box1_8);
const Box box1_10 = Box(box1_9, box1_9);
const Box box1_11 = Box(box1_10, box1_10);
const Box box1_12 = Box(box1_11, box1_11);
const Box box1_13 = Box(box1_12, box1_12);
const Box box1_14 = Box(box1_13, box1_13);
const Box box1_15 = Box(box1_14, box1_14);
const Box box1_16 = Box(box1_15, box1_15);
const Box box1_17 = Box(box1_16, box1_16);
const Box box1_18 = Box(box1_17, box1_17);
const Box box1_19 = Box(box1_18, box1_18);
const Box box1_20 = Box(box1_19, box1_19);
const Box box1_21 = Box(box1_20, box1_20);
const Box box1_22 = Box(box1_21, box1_21);
const Box box1_23 = Box(box1_22, box1_22);
const Box box1_24 = Box(box1_23, box1_23);
const Box box1_25 = Box(box1_24, box1_24);
const Box box1_26 = Box(box1_25, box1_25);
const Box box1_27 = Box(box1_26, box1_26);
const Box box1_28 = Box(box1_27, box1_27);
const Box box1_29 = Box(box1_28, box1_28);
const Box box1_30 = Box(box1_29, box1_29);
const Box box1_31 = Box(box1_30, box1_30);
const Box box1_32 = Box(box1_31, box1_31);
const Box box1_33 = Box(box1_32, box1_32);
const Box box1_34 = Box(box1_33, box1_33);
const Box box1_35 = Box(box1_34, box1_34);
const Box box1_36 = Box(box1_35, box1_35);
const Box box1_37 = Box(box1_36, box1_36);
const Box box1_38 = Box(box1_37, box1_37);
const Box box1_39 = Box(box1_38, box1_38);
const Box box1_40 = Box(box1_39, box1_39);
const Box box1_41 = Box(box1_40, box1_40);
const Box box1_42 = Box(box1_41, box1_41);
const Box box1_43 = Box(box1_42, box1_42);
const Box box1_44 = Box(box1_43, box1_43);
const Box box1_45 = Box(box1_44, box1_44);
const Box box1_46 = Box(box1_45, box1_45);
const Box box1_47 = Box(box1_46, box1_46);
const Box box1_48 = Box(box1_47, box1_47);
const Box box1_49 = Box(box1_48, box1_48);
const Box box1_50 = Box(box1_49, box1_49);
const Box box1_51 = Box(box1_50, box1_50);
const Box box1_52 = Box(box1_51, box1_51);
const Box box1_53 = Box(box1_52, box1_52);
const Box box1_54 = Box(box1_53, box1_53);
const Box box1_55 = Box(box1_54, box1_54);
const Box box1_56 = Box(box1_55, box1_55);
const Box box1_57 = Box(box1_56, box1_56);
const Box box1_58 = Box(box1_57, box1_57);
const Box box1_59 = Box(box1_58, box1_58);
const Box box1_60 = Box(box1_59, box1_59);
const Box box1_61 = Box(box1_60, box1_60);
const Box box1_62 = Box(box1_61, box1_61);
const Box box1_63 = Box(box1_62, box1_62);
const Box box1_64 = Box(box1_63, box1_63);
const Box box1_65 = Box(box1_64, box1_64);
const Box box1_66 = Box(box1_65, box1_65);
const Box box1_67 = Box(box1_66, box1_66);
const Box box1_68 = Box(box1_67, box1_67);
const Box box1_69 = Box(box1_68, box1_68);
const Box box1_70 = Box(box1_69, box1_69);
const Box box1_71 = Box(box1_70, box1_70);
const Box box1_72 = Box(box1_71, box1_71);
const Box box1_73 = Box(box1_72, box1_72);
const Box box1_74 = Box(box1_73, box1_73);
const Box box1_75 = Box(box1_74, box1_74);
const Box box1_76 = Box(box1_75, box1_75);
const Box box1_77 = Box(box1_76, box1_76);
const Box box1_78 = Box(box1_77, box1_77);
const Box box1_79 = Box(box1_78, box1_78);
const Box box1_80 = Box(box1_79, box1_79);
const Box box1_81 = Box(box1_80, box1_80);
const Box box1_82 = Box(box1_81, box1_81);
const Box box1_83 = Box(box1_82, box1_82);
const Box box1_84 = Box(box1_83, box1_83);
const Box box1_85 = Box(box1_84, box1_84);
const Box box1_86 = Box(box1_85, box1_85);
const Box box1_87 = Box(box1_86, box1_86);
const Box box1_88 = Box(box1_87, box1_87);
const Box box1_89 = Box(box1_88, box1_88);
const Box box1_90 = Box(box1_89, box1_89);
const Box box1_91 = Box(box1_90, box1_90);
const Box box1_92 = Box(box1_91, box1_91);
const Box box1_93 = Box(box1_92, box1_92);
const Box box1_94 = Box(box1_93, box1_93);
const Box box1_95 = Box(box1_94, box1_94);
const Box box1_96 = Box(box1_95, box1_95);
const Box box1_97 = Box(box1_96, box1_96);
const Box box1_98 = Box(box1_97, box1_97);
const Box box1_99 = Box(box1_98, box1_98);

const Box box2_0 = Box(null, null);
const Box box2_1 = Box(box2_0, box2_0);
const Box box2_2 = Box(box2_1, box2_1);
const Box box2_3 = Box(box2_2, box2_2);
const Box box2_4 = Box(box2_3, box2_3);
const Box box2_5 = Box(box2_4, box2_4);
const Box box2_6 = Box(box2_5, box2_5);
const Box box2_7 = Box(box2_6, box2_6);
const Box box2_8 = Box(box2_7, box2_7);
const Box box2_9 = Box(box2_8, box2_8);
const Box box2_10 = Box(box2_9, box2_9);
const Box box2_11 = Box(box2_10, box2_10);
const Box box2_12 = Box(box2_11, box2_11);
const Box box2_13 = Box(box2_12, box2_12);
const Box box2_14 = Box(box2_13, box2_13);
const Box box2_15 = Box(box2_14, box2_14);
const Box box2_16 = Box(box2_15, box2_15);
const Box box2_17 = Box(box2_16, box2_16);
const Box box2_18 = Box(box2_17, box2_17);
const Box box2_19 = Box(box2_18, box2_18);
const Box box2_20 = Box(box2_19, box2_19);
const Box box2_21 = Box(box2_20, box2_20);
const Box box2_22 = Box(box2_21, box2_21);
const Box box2_23 = Box(box2_22, box2_22);
const Box box2_24 = Box(box2_23, box2_23);
const Box box2_25 = Box(box2_24, box2_24);
const Box box2_26 = Box(box2_25, box2_25);
const Box box2_27 = Box(box2_26, box2_26);
const Box box2_28 = Box(box2_27, box2_27);
const Box box2_29 = Box(box2_28, box2_28);
const Box box2_30 = Box(box2_29, box2_29);
const Box box2_31 = Box(box2_30, box2_30);
const Box box2_32 = Box(box2_31, box2_31);
const Box box2_33 = Box(box2_32, box2_32);
const Box box2_34 = Box(box2_33, box2_33);
const Box box2_35 = Box(box2_34, box2_34);
const Box box2_36 = Box(box2_35, box2_35);
const Box box2_37 = Box(box2_36, box2_36);
const Box box2_38 = Box(box2_37, box2_37);
const Box box2_39 = Box(box2_38, box2_38);
const Box box2_40 = Box(box2_39, box2_39);
const Box box2_41 = Box(box2_40, box2_40);
const Box box2_42 = Box(box2_41, box2_41);
const Box box2_43 = Box(box2_42, box2_42);
const Box box2_44 = Box(box2_43, box2_43);
const Box box2_45 = Box(box2_44, box2_44);
const Box box2_46 = Box(box2_45, box2_45);
const Box box2_47 = Box(box2_46, box2_46);
const Box box2_48 = Box(box2_47, box2_47);
const Box box2_49 = Box(box2_48, box2_48);
const Box box2_50 = Box(box2_49, box2_49);
const Box box2_51 = Box(box2_50, box2_50);
const Box box2_52 = Box(box2_51, box2_51);
const Box box2_53 = Box(box2_52, box2_52);
const Box box2_54 = Box(box2_53, box2_53);
const Box box2_55 = Box(box2_54, box2_54);
const Box box2_56 = Box(box2_55, box2_55);
const Box box2_57 = Box(box2_56, box2_56);
const Box box2_58 = Box(box2_57, box2_57);
const Box box2_59 = Box(box2_58, box2_58);
const Box box2_60 = Box(box2_59, box2_59);
const Box box2_61 = Box(box2_60, box2_60);
const Box box2_62 = Box(box2_61, box2_61);
const Box box2_63 = Box(box2_62, box2_62);
const Box box2_64 = Box(box2_63, box2_63);
const Box box2_65 = Box(box2_64, box2_64);
const Box box2_66 = Box(box2_65, box2_65);
const Box box2_67 = Box(box2_66, box2_66);
const Box box2_68 = Box(box2_67, box2_67);
const Box box2_69 = Box(box2_68, box2_68);
const Box box2_70 = Box(box2_69, box2_69);
const Box box2_71 = Box(box2_70, box2_70);
const Box box2_72 = Box(box2_71, box2_71);
const Box box2_73 = Box(box2_72, box2_72);
const Box box2_74 = Box(box2_73, box2_73);
const Box box2_75 = Box(box2_74, box2_74);
const Box box2_76 = Box(box2_75, box2_75);
const Box box2_77 = Box(box2_76, box2_76);
const Box box2_78 = Box(box2_77, box2_77);
const Box box2_79 = Box(box2_78, box2_78);
const Box box2_80 = Box(box2_79, box2_79);
const Box box2_81 = Box(box2_80, box2_80);
const Box box2_82 = Box(box2_81, box2_81);
const Box box2_83 = Box(box2_82, box2_82);
const Box box2_84 = Box(box2_83, box2_83);
const Box box2_85 = Box(box2_84, box2_84);
const Box box2_86 = Box(box2_85, box2_85);
const Box box2_87 = Box(box2_86, box2_86);
const Box box2_88 = Box(box2_87, box2_87);
const Box box2_89 = Box(box2_88, box2_88);
const Box box2_90 = Box(box2_89, box2_89);
const Box box2_91 = Box(box2_90, box2_90);
const Box box2_92 = Box(box2_91, box2_91);
const Box box2_93 = Box(box2_92, box2_92);
const Box box2_94 = Box(box2_93, box2_93);
const Box box2_95 = Box(box2_94, box2_94);
const Box box2_96 = Box(box2_95, box2_95);
const Box box2_97 = Box(box2_96, box2_96);
const Box box2_98 = Box(box2_97, box2_97);
const Box box2_99 = Box(box2_98, box2_98);

Object confuse(Box x) {
  try {
    throw x; // ignore: only_throw_errors
  } catch (e) {
    return e;
  }
}

void main() {
  if (!identical(confuse(box1_99), confuse(box2_99))) {
    throw Exception('box1_99 !== box2_99');
  }
}
