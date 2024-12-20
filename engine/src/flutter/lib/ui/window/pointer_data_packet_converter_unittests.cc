// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/window/pointer_data_packet_converter.h"

#include <cstring>
#include <unordered_set>

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {

constexpr int64_t kImplicitViewId = 0;

}

class TestDelegate : public PointerDataPacketConverter::Delegate {
 public:
  // |PointerDataPacketConverter::Delegate|
  bool ViewExists(int64_t view_id) const override {
    return views_.count(view_id) != 0;
  }

  void AddView(int64_t view_id) { views_.insert(view_id); }

  void RemoveView(int64_t view_id) { views_.erase(view_id); }

 private:
  std::unordered_set<int64_t> views_;
};

void CreateSimulatedPointerData(PointerData& data,  // NOLINT
                                PointerData::Change change,
                                int64_t device,
                                double dx,
                                double dy,
                                int64_t buttons) {
  data.time_stamp = 0;
  data.change = change;
  data.kind = PointerData::DeviceKind::kTouch;
  data.signal_kind = PointerData::SignalKind::kNone;
  data.device = device;
  data.pointer_identifier = 0;
  data.physical_x = dx;
  data.physical_y = dy;
  data.physical_delta_x = 0.0;
  data.physical_delta_y = 0.0;
  data.buttons = buttons;
  data.obscured = 0;
  data.synthesized = 0;
  data.pressure = 0.0;
  data.pressure_min = 0.0;
  data.pressure_max = 0.0;
  data.distance = 0.0;
  data.distance_max = 0.0;
  data.size = 0.0;
  data.radius_major = 0.0;
  data.radius_minor = 0.0;
  data.radius_min = 0.0;
  data.radius_max = 0.0;
  data.orientation = 0.0;
  data.tilt = 0.0;
  data.platformData = 0;
  data.scroll_delta_x = 0.0;
  data.scroll_delta_y = 0.0;
  data.view_id = kImplicitViewId;
}

void CreateSimulatedMousePointerData(PointerData& data,  // NOLINT
                                     PointerData::Change change,
                                     PointerData::SignalKind signal_kind,
                                     int64_t device,
                                     double dx,
                                     double dy,
                                     double scroll_delta_x,
                                     double scroll_delta_y,
                                     int64_t buttons) {
  data.time_stamp = 0;
  data.change = change;
  data.kind = PointerData::DeviceKind::kMouse;
  data.signal_kind = signal_kind;
  data.device = device;
  data.pointer_identifier = 0;
  data.physical_x = dx;
  data.physical_y = dy;
  data.physical_delta_x = 0.0;
  data.physical_delta_y = 0.0;
  data.buttons = buttons;
  data.obscured = 0;
  data.synthesized = 0;
  data.pressure = 0.0;
  data.pressure_min = 0.0;
  data.pressure_max = 0.0;
  data.distance = 0.0;
  data.distance_max = 0.0;
  data.size = 0.0;
  data.radius_major = 0.0;
  data.radius_minor = 0.0;
  data.radius_min = 0.0;
  data.radius_max = 0.0;
  data.orientation = 0.0;
  data.tilt = 0.0;
  data.platformData = 0;
  data.scroll_delta_x = scroll_delta_x;
  data.scroll_delta_y = scroll_delta_y;
  data.view_id = kImplicitViewId;
}

void CreateSimulatedTrackpadGestureData(PointerData& data,  // NOLINT
                                        PointerData::Change change,
                                        int64_t device,
                                        double dx,
                                        double dy,
                                        double pan_x,
                                        double pan_y,
                                        double scale,
                                        double rotation) {
  data.time_stamp = 0;
  data.change = change;
  data.kind = PointerData::DeviceKind::kMouse;
  data.signal_kind = PointerData::SignalKind::kNone;
  data.device = device;
  data.pointer_identifier = 0;
  data.physical_x = dx;
  data.physical_y = dy;
  data.physical_delta_x = 0.0;
  data.physical_delta_y = 0.0;
  data.buttons = 0;
  data.obscured = 0;
  data.synthesized = 0;
  data.pressure = 0.0;
  data.pressure_min = 0.0;
  data.pressure_max = 0.0;
  data.distance = 0.0;
  data.distance_max = 0.0;
  data.size = 0.0;
  data.radius_major = 0.0;
  data.radius_minor = 0.0;
  data.radius_min = 0.0;
  data.radius_max = 0.0;
  data.orientation = 0.0;
  data.tilt = 0.0;
  data.platformData = 0;
  data.scroll_delta_x = 0.0;
  data.scroll_delta_y = 0.0;
  data.pan_x = pan_x;
  data.pan_y = pan_y;
  data.pan_delta_x = 0.0;
  data.pan_delta_y = 0.0;
  data.scale = scale;
  data.rotation = rotation;
  data.view_id = 0;
}

void UnpackPointerPacket(std::vector<PointerData>& output,  // NOLINT
                         std::unique_ptr<PointerDataPacket> packet) {
  for (size_t i = 0; i < packet->GetLength(); i++) {
    PointerData pointer_data = packet->GetPointerData(i);
    output.push_back(pointer_data);
  }
  packet.reset();
}

TEST(PointerDataPacketConverterTest, CanConvertPointerDataPacket) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(6);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kHover, 0, 3.0, 0.0, 0);
  packet->SetPointerData(1, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 3.0, 0.0, 1);
  packet->SetPointerData(2, data);
  CreateSimulatedPointerData(data, PointerData::Change::kMove, 0, 3.0, 4.0, 1);
  packet->SetPointerData(3, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 3.0, 4.0, 0);
  packet->SetPointerData(4, data);
  CreateSimulatedPointerData(data, PointerData::Change::kRemove, 0, 3.0, 4.0,
                             0);
  packet->SetPointerData(5, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)6);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].synthesized, 0);

  ASSERT_EQ(result[1].change, PointerData::Change::kHover);
  ASSERT_EQ(result[1].synthesized, 0);
  ASSERT_EQ(result[1].physical_delta_x, 3.0);
  ASSERT_EQ(result[1].physical_delta_y, 0.0);

  ASSERT_EQ(result[2].change, PointerData::Change::kDown);
  ASSERT_EQ(result[2].pointer_identifier, 1);
  ASSERT_EQ(result[2].synthesized, 0);

  ASSERT_EQ(result[3].change, PointerData::Change::kMove);
  ASSERT_EQ(result[3].pointer_identifier, 1);
  ASSERT_EQ(result[3].synthesized, 0);
  ASSERT_EQ(result[3].physical_delta_x, 0.0);
  ASSERT_EQ(result[3].physical_delta_y, 4.0);

  ASSERT_EQ(result[4].change, PointerData::Change::kUp);
  ASSERT_EQ(result[4].pointer_identifier, 1);
  ASSERT_EQ(result[4].synthesized, 0);

  ASSERT_EQ(result[5].change, PointerData::Change::kRemove);
  ASSERT_EQ(result[5].synthesized, 0);
}

TEST(PointerDataPacketConverterTest, CanSynthesizeDownAndUp) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(4);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 3.0, 0.0, 1);
  packet->SetPointerData(1, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 3.0, 4.0, 0);
  packet->SetPointerData(2, data);
  CreateSimulatedPointerData(data, PointerData::Change::kRemove, 0, 3.0, 4.0,
                             0);
  packet->SetPointerData(3, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)6);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].synthesized, 0);

  // A hover should be synthesized.
  ASSERT_EQ(result[1].change, PointerData::Change::kHover);
  ASSERT_EQ(result[1].synthesized, 1);
  ASSERT_EQ(result[1].physical_delta_x, 3.0);
  ASSERT_EQ(result[1].physical_delta_y, 0.0);
  ASSERT_EQ(result[1].buttons, 0);

  ASSERT_EQ(result[2].change, PointerData::Change::kDown);
  ASSERT_EQ(result[2].pointer_identifier, 1);
  ASSERT_EQ(result[2].synthesized, 0);
  ASSERT_EQ(result[2].buttons, 1);

  // A move should be synthesized.
  ASSERT_EQ(result[3].change, PointerData::Change::kMove);
  ASSERT_EQ(result[3].pointer_identifier, 1);
  ASSERT_EQ(result[3].synthesized, 1);
  ASSERT_EQ(result[3].physical_delta_x, 0.0);
  ASSERT_EQ(result[3].physical_delta_y, 4.0);
  ASSERT_EQ(result[3].buttons, 1);

  ASSERT_EQ(result[4].change, PointerData::Change::kUp);
  ASSERT_EQ(result[4].pointer_identifier, 1);
  ASSERT_EQ(result[4].synthesized, 0);
  ASSERT_EQ(result[4].buttons, 0);

  ASSERT_EQ(result[5].change, PointerData::Change::kRemove);
  ASSERT_EQ(result[5].synthesized, 0);
}

TEST(PointerDataPacketConverterTest, CanUpdatePointerIdentifier) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(7);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 0.0, 0.0, 1);
  packet->SetPointerData(1, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 0.0, 0.0, 0);
  packet->SetPointerData(2, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 0.0, 0.0, 1);
  packet->SetPointerData(3, data);
  CreateSimulatedPointerData(data, PointerData::Change::kMove, 0, 3.0, 0.0, 1);
  packet->SetPointerData(4, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 3.0, 0.0, 0);
  packet->SetPointerData(5, data);
  CreateSimulatedPointerData(data, PointerData::Change::kRemove, 0, 3.0, 0.0,
                             0);
  packet->SetPointerData(6, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)7);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].synthesized, 0);

  ASSERT_EQ(result[1].change, PointerData::Change::kDown);
  ASSERT_EQ(result[1].pointer_identifier, 1);
  ASSERT_EQ(result[1].synthesized, 0);

  ASSERT_EQ(result[2].change, PointerData::Change::kUp);
  ASSERT_EQ(result[2].pointer_identifier, 1);
  ASSERT_EQ(result[2].synthesized, 0);

  // Pointer count increase to 2.
  ASSERT_EQ(result[3].change, PointerData::Change::kDown);
  ASSERT_EQ(result[3].pointer_identifier, 2);
  ASSERT_EQ(result[3].synthesized, 0);

  ASSERT_EQ(result[4].change, PointerData::Change::kMove);
  ASSERT_EQ(result[4].pointer_identifier, 2);
  ASSERT_EQ(result[4].synthesized, 0);
  ASSERT_EQ(result[4].physical_delta_x, 3.0);
  ASSERT_EQ(result[4].physical_delta_y, 0.0);

  ASSERT_EQ(result[5].change, PointerData::Change::kUp);
  ASSERT_EQ(result[5].pointer_identifier, 2);
  ASSERT_EQ(result[5].synthesized, 0);

  ASSERT_EQ(result[6].change, PointerData::Change::kRemove);
  ASSERT_EQ(result[6].synthesized, 0);
}

TEST(PointerDataPacketConverterTest, AlwaysForwardMoveEvent) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(4);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 0.0, 0.0, 1);
  packet->SetPointerData(1, data);
  // Creates a move event without a location change.
  CreateSimulatedPointerData(data, PointerData::Change::kMove, 0, 0.0, 0.0, 1);
  packet->SetPointerData(2, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 0.0, 0.0, 0);
  packet->SetPointerData(3, data);

  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)4);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].synthesized, 0);

  ASSERT_EQ(result[1].change, PointerData::Change::kDown);
  ASSERT_EQ(result[1].pointer_identifier, 1);
  ASSERT_EQ(result[1].synthesized, 0);

  // Does not filter out the move event.
  ASSERT_EQ(result[2].change, PointerData::Change::kMove);
  ASSERT_EQ(result[2].pointer_identifier, 1);
  ASSERT_EQ(result[2].synthesized, 0);

  ASSERT_EQ(result[3].change, PointerData::Change::kUp);
  ASSERT_EQ(result[3].pointer_identifier, 1);
  ASSERT_EQ(result[3].synthesized, 0);
}

TEST(PointerDataPacketConverterTest, CanWorkWithDifferentDevices) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(12);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 0.0, 0.0, 1);
  packet->SetPointerData(1, data);
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 1, 0.0, 0.0, 0);
  packet->SetPointerData(2, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 1, 0.0, 0.0, 1);
  packet->SetPointerData(3, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 0.0, 0.0, 0);
  packet->SetPointerData(4, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 0.0, 0.0, 1);
  packet->SetPointerData(5, data);
  CreateSimulatedPointerData(data, PointerData::Change::kMove, 1, 0.0, 4.0, 1);
  packet->SetPointerData(6, data);
  CreateSimulatedPointerData(data, PointerData::Change::kMove, 0, 3.0, 0.0, 1);
  packet->SetPointerData(7, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 1, 0.0, 4.0, 0);
  packet->SetPointerData(8, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 3.0, 0.0, 0);
  packet->SetPointerData(9, data);
  CreateSimulatedPointerData(data, PointerData::Change::kRemove, 0, 3.0, 0.0,
                             0);
  packet->SetPointerData(10, data);
  CreateSimulatedPointerData(data, PointerData::Change::kRemove, 1, 0.0, 4.0,
                             0);
  packet->SetPointerData(11, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)12);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].device, 0);
  ASSERT_EQ(result[0].synthesized, 0);

  ASSERT_EQ(result[1].change, PointerData::Change::kDown);
  ASSERT_EQ(result[1].device, 0);
  ASSERT_EQ(result[1].pointer_identifier, 1);
  ASSERT_EQ(result[1].synthesized, 0);

  ASSERT_EQ(result[2].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[2].device, 1);
  ASSERT_EQ(result[2].synthesized, 0);

  ASSERT_EQ(result[3].change, PointerData::Change::kDown);
  ASSERT_EQ(result[3].device, 1);
  ASSERT_EQ(result[3].pointer_identifier, 2);
  ASSERT_EQ(result[3].synthesized, 0);

  ASSERT_EQ(result[4].change, PointerData::Change::kUp);
  ASSERT_EQ(result[4].device, 0);
  ASSERT_EQ(result[4].pointer_identifier, 1);
  ASSERT_EQ(result[4].synthesized, 0);

  ASSERT_EQ(result[5].change, PointerData::Change::kDown);
  ASSERT_EQ(result[5].device, 0);
  ASSERT_EQ(result[5].pointer_identifier, 3);
  ASSERT_EQ(result[5].synthesized, 0);

  ASSERT_EQ(result[6].change, PointerData::Change::kMove);
  ASSERT_EQ(result[6].device, 1);
  ASSERT_EQ(result[6].pointer_identifier, 2);
  ASSERT_EQ(result[6].synthesized, 0);
  ASSERT_EQ(result[6].physical_delta_x, 0.0);
  ASSERT_EQ(result[6].physical_delta_y, 4.0);

  ASSERT_EQ(result[7].change, PointerData::Change::kMove);
  ASSERT_EQ(result[7].device, 0);
  ASSERT_EQ(result[7].pointer_identifier, 3);
  ASSERT_EQ(result[7].synthesized, 0);
  ASSERT_EQ(result[7].physical_delta_x, 3.0);
  ASSERT_EQ(result[7].physical_delta_y, 0.0);

  ASSERT_EQ(result[8].change, PointerData::Change::kUp);
  ASSERT_EQ(result[8].device, 1);
  ASSERT_EQ(result[8].pointer_identifier, 2);
  ASSERT_EQ(result[8].synthesized, 0);

  ASSERT_EQ(result[9].change, PointerData::Change::kUp);
  ASSERT_EQ(result[9].device, 0);
  ASSERT_EQ(result[9].pointer_identifier, 3);
  ASSERT_EQ(result[9].synthesized, 0);

  ASSERT_EQ(result[10].change, PointerData::Change::kRemove);
  ASSERT_EQ(result[10].device, 0);
  ASSERT_EQ(result[10].synthesized, 0);

  ASSERT_EQ(result[11].change, PointerData::Change::kRemove);
  ASSERT_EQ(result[11].device, 1);
  ASSERT_EQ(result[11].synthesized, 0);
}

TEST(PointerDataPacketConverterTest, CanSynthesizeAdd) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(2);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 330.0, 450.0,
                             1);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kUp, 0, 0.0, 0.0, 0);
  packet->SetPointerData(1, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)4);
  // A add should be synthesized.
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].physical_x, 330.0);
  ASSERT_EQ(result[0].physical_y, 450.0);
  ASSERT_EQ(result[0].synthesized, 1);
  ASSERT_EQ(result[0].buttons, 0);

  ASSERT_EQ(result[1].change, PointerData::Change::kDown);
  ASSERT_EQ(result[1].physical_x, 330.0);
  ASSERT_EQ(result[1].physical_y, 450.0);
  ASSERT_EQ(result[1].synthesized, 0);
  ASSERT_EQ(result[1].buttons, 1);

  // A move should be synthesized.
  ASSERT_EQ(result[2].change, PointerData::Change::kMove);
  ASSERT_EQ(result[2].physical_delta_x, -330.0);
  ASSERT_EQ(result[2].physical_delta_y, -450.0);
  ASSERT_EQ(result[2].physical_x, 0.0);
  ASSERT_EQ(result[2].physical_y, 0.0);
  ASSERT_EQ(result[2].synthesized, 1);
  ASSERT_EQ(result[2].buttons, 1);

  ASSERT_EQ(result[3].change, PointerData::Change::kUp);
  ASSERT_EQ(result[3].physical_x, 0.0);
  ASSERT_EQ(result[3].physical_y, 0.0);
  ASSERT_EQ(result[3].synthesized, 0);
  ASSERT_EQ(result[3].buttons, 0);
}

TEST(PointerDataPacketConverterTest, CanSynthesizeRemove) {
  TestDelegate delegate;
  delegate.AddView(100);
  delegate.AddView(200);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(3);

  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  data.view_id = 100;
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 3.0, 4.0, 1);
  data.view_id = 100;
  packet->SetPointerData(1, data);
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  data.view_id = 200;
  packet->SetPointerData(2, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)6);
  ASSERT_EQ(result[0].synthesized, 0);
  ASSERT_EQ(result[0].view_id, 100);

  // A hover should be synthesized.
  ASSERT_EQ(result[1].change, PointerData::Change::kHover);
  ASSERT_EQ(result[1].synthesized, 1);
  ASSERT_EQ(result[1].physical_delta_x, 3.0);
  ASSERT_EQ(result[1].physical_delta_y, 4.0);
  ASSERT_EQ(result[1].buttons, 0);

  ASSERT_EQ(result[2].change, PointerData::Change::kDown);
  ASSERT_EQ(result[2].pointer_identifier, 1);
  ASSERT_EQ(result[2].synthesized, 0);
  ASSERT_EQ(result[2].buttons, 1);

  // A cancel should be synthesized.
  ASSERT_EQ(result[3].change, PointerData::Change::kCancel);
  ASSERT_EQ(result[3].pointer_identifier, 1);
  ASSERT_EQ(result[3].synthesized, 1);
  ASSERT_EQ(result[3].physical_x, 3.0);
  ASSERT_EQ(result[3].physical_y, 4.0);
  ASSERT_EQ(result[3].buttons, 1);

  // A remove should be synthesized.
  ASSERT_EQ(result[4].physical_x, 3.0);
  ASSERT_EQ(result[4].physical_y, 4.0);
  ASSERT_EQ(result[4].synthesized, 1);
  ASSERT_EQ(result[4].view_id, 100);

  ASSERT_EQ(result[5].synthesized, 0);
  ASSERT_EQ(result[5].view_id, 200);
}

TEST(PointerDataPacketConverterTest,
     CanAvoidDoubleRemoveAfterSynthesizedRemove) {
  TestDelegate delegate;
  delegate.AddView(100);
  delegate.AddView(200);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(2);

  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  data.view_id = 100;
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  data.view_id = 200;
  packet->SetPointerData(1, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)3);
  ASSERT_EQ(result[0].synthesized, 0);
  ASSERT_EQ(result[0].view_id, 100);

  // A remove should be synthesized.
  ASSERT_EQ(result[1].synthesized, 1);
  ASSERT_EQ(result[1].view_id, 100);

  ASSERT_EQ(result[2].synthesized, 0);
  ASSERT_EQ(result[2].view_id, 200);

  // Simulate a double remove.
  packet = std::make_unique<PointerDataPacket>(1);
  CreateSimulatedPointerData(data, PointerData::Change::kRemove, 0, 0.0, 0.0,
                             0);
  data.view_id = 100;
  packet->SetPointerData(0, data);
  converted_packet = converter.Convert(*packet);

  result.clear();
  UnpackPointerPacket(result, std::move(converted_packet));

  // The double remove should be ignored.
  ASSERT_EQ(result.size(), (size_t)0);
}

TEST(PointerDataPacketConverterTest, CanHandleThreeFingerGesture) {
  // Regression test https://github.com/flutter/flutter/issues/20517.
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  PointerData data;
  std::vector<PointerData> result;
  // First finger down.
  auto packet = std::make_unique<PointerDataPacket>(1);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 0, 0.0, 0.0, 1);
  packet->SetPointerData(0, data);
  auto converted_packet = converter.Convert(*packet);
  UnpackPointerPacket(result, std::move(converted_packet));
  // Second finger down.
  packet = std::make_unique<PointerDataPacket>(1);
  CreateSimulatedPointerData(data, PointerData::Change::kDown, 1, 33.0, 44.0,
                             1);
  packet->SetPointerData(0, data);
  converted_packet = converter.Convert(*packet);
  UnpackPointerPacket(result, std::move(converted_packet));
  // Triggers three cancels.
  packet = std::make_unique<PointerDataPacket>(3);
  CreateSimulatedPointerData(data, PointerData::Change::kCancel, 1, 33.0, 44.0,
                             0);
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kCancel, 0, 0.0, 0.0,
                             0);
  packet->SetPointerData(1, data);
  CreateSimulatedPointerData(data, PointerData::Change::kCancel, 2, 40.0, 50.0,
                             0);
  packet->SetPointerData(2, data);
  converted_packet = converter.Convert(*packet);
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)6);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].device, 0);
  ASSERT_EQ(result[0].physical_x, 0.0);
  ASSERT_EQ(result[0].physical_y, 0.0);
  ASSERT_EQ(result[0].synthesized, 1);
  ASSERT_EQ(result[0].buttons, 0);

  ASSERT_EQ(result[1].change, PointerData::Change::kDown);
  ASSERT_EQ(result[1].device, 0);
  ASSERT_EQ(result[1].physical_x, 0.0);
  ASSERT_EQ(result[1].physical_y, 0.0);
  ASSERT_EQ(result[1].synthesized, 0);
  ASSERT_EQ(result[1].buttons, 1);

  ASSERT_EQ(result[2].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[2].device, 1);
  ASSERT_EQ(result[2].physical_x, 33.0);
  ASSERT_EQ(result[2].physical_y, 44.0);
  ASSERT_EQ(result[2].synthesized, 1);
  ASSERT_EQ(result[2].buttons, 0);

  ASSERT_EQ(result[3].change, PointerData::Change::kDown);
  ASSERT_EQ(result[3].device, 1);
  ASSERT_EQ(result[3].physical_x, 33.0);
  ASSERT_EQ(result[3].physical_y, 44.0);
  ASSERT_EQ(result[3].synthesized, 0);
  ASSERT_EQ(result[3].buttons, 1);

  ASSERT_EQ(result[4].change, PointerData::Change::kCancel);
  ASSERT_EQ(result[4].device, 1);
  ASSERT_EQ(result[4].physical_x, 33.0);
  ASSERT_EQ(result[4].physical_y, 44.0);
  ASSERT_EQ(result[4].synthesized, 0);

  ASSERT_EQ(result[5].change, PointerData::Change::kCancel);
  ASSERT_EQ(result[5].device, 0);
  ASSERT_EQ(result[5].physical_x, 0.0);
  ASSERT_EQ(result[5].physical_y, 0.0);
  ASSERT_EQ(result[5].synthesized, 0);
  // Third cancel should be dropped
}

TEST(PointerDataPacketConverterTest, CanConvertPointerSignals) {
  PointerData::SignalKind signal_kinds[] = {
      PointerData::SignalKind::kScroll,
      PointerData::SignalKind::kScrollInertiaCancel,
      PointerData::SignalKind::kScale,
  };
  for (const PointerData::SignalKind& kind : signal_kinds) {
    TestDelegate delegate;
    delegate.AddView(kImplicitViewId);
    PointerDataPacketConverter converter(delegate);
    auto packet = std::make_unique<PointerDataPacket>(6);
    PointerData data;
    CreateSimulatedMousePointerData(data, PointerData::Change::kAdd,
                                    PointerData::SignalKind::kNone, 0, 0.0, 0.0,
                                    0.0, 0.0, 0);
    packet->SetPointerData(0, data);
    CreateSimulatedMousePointerData(data, PointerData::Change::kAdd,
                                    PointerData::SignalKind::kNone, 1, 0.0, 0.0,
                                    0.0, 0.0, 0);
    packet->SetPointerData(1, data);
    CreateSimulatedMousePointerData(data, PointerData::Change::kDown,
                                    PointerData::SignalKind::kNone, 1, 0.0, 0.0,
                                    0.0, 0.0, 1);
    packet->SetPointerData(2, data);
    CreateSimulatedMousePointerData(data, PointerData::Change::kHover, kind, 0,
                                    34.0, 34.0, 30.0, 0.0, 0);
    packet->SetPointerData(3, data);
    CreateSimulatedMousePointerData(data, PointerData::Change::kHover, kind, 1,
                                    49.0, 49.0, 50.0, 0.0, 0);
    packet->SetPointerData(4, data);
    CreateSimulatedMousePointerData(data, PointerData::Change::kHover, kind, 2,
                                    10.0, 20.0, 30.0, 40.0, 0);
    packet->SetPointerData(5, data);
    auto converted_packet = converter.Convert(*packet);

    std::vector<PointerData> result;
    UnpackPointerPacket(result, std::move(converted_packet));

    ASSERT_EQ(result.size(), (size_t)9);
    ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
    ASSERT_EQ(result[0].signal_kind, PointerData::SignalKind::kNone);
    ASSERT_EQ(result[0].device, 0);
    ASSERT_EQ(result[0].physical_x, 0.0);
    ASSERT_EQ(result[0].physical_y, 0.0);
    ASSERT_EQ(result[0].synthesized, 0);

    ASSERT_EQ(result[1].change, PointerData::Change::kAdd);
    ASSERT_EQ(result[1].signal_kind, PointerData::SignalKind::kNone);
    ASSERT_EQ(result[1].device, 1);
    ASSERT_EQ(result[1].physical_x, 0.0);
    ASSERT_EQ(result[1].physical_y, 0.0);
    ASSERT_EQ(result[1].synthesized, 0);

    ASSERT_EQ(result[2].change, PointerData::Change::kDown);
    ASSERT_EQ(result[2].signal_kind, PointerData::SignalKind::kNone);
    ASSERT_EQ(result[2].device, 1);
    ASSERT_EQ(result[2].physical_x, 0.0);
    ASSERT_EQ(result[2].physical_y, 0.0);
    ASSERT_EQ(result[2].synthesized, 0);

    // Converter will synthesize a hover to position for device 0.
    ASSERT_EQ(result[3].change, PointerData::Change::kHover);
    ASSERT_EQ(result[3].signal_kind, PointerData::SignalKind::kNone);
    ASSERT_EQ(result[3].device, 0);
    ASSERT_EQ(result[3].physical_x, 34.0);
    ASSERT_EQ(result[3].physical_y, 34.0);
    ASSERT_EQ(result[3].physical_delta_x, 34.0);
    ASSERT_EQ(result[3].physical_delta_y, 34.0);
    ASSERT_EQ(result[3].buttons, 0);
    ASSERT_EQ(result[3].synthesized, 1);

    ASSERT_EQ(result[4].change, PointerData::Change::kHover);
    ASSERT_EQ(result[4].signal_kind, kind);
    ASSERT_EQ(result[4].device, 0);
    ASSERT_EQ(result[4].physical_x, 34.0);
    ASSERT_EQ(result[4].physical_y, 34.0);
    ASSERT_EQ(result[4].scroll_delta_x, 30.0);
    ASSERT_EQ(result[4].scroll_delta_y, 0.0);

    // Converter will synthesize a move to position for device 1.
    ASSERT_EQ(result[5].change, PointerData::Change::kMove);
    ASSERT_EQ(result[5].signal_kind, PointerData::SignalKind::kNone);
    ASSERT_EQ(result[5].device, 1);
    ASSERT_EQ(result[5].physical_x, 49.0);
    ASSERT_EQ(result[5].physical_y, 49.0);
    ASSERT_EQ(result[5].physical_delta_x, 49.0);
    ASSERT_EQ(result[5].physical_delta_y, 49.0);
    ASSERT_EQ(result[5].buttons, 1);
    ASSERT_EQ(result[5].synthesized, 1);

    ASSERT_EQ(result[6].change, PointerData::Change::kHover);
    ASSERT_EQ(result[6].signal_kind, kind);
    ASSERT_EQ(result[6].device, 1);
    ASSERT_EQ(result[6].physical_x, 49.0);
    ASSERT_EQ(result[6].physical_y, 49.0);
    ASSERT_EQ(result[6].scroll_delta_x, 50.0);
    ASSERT_EQ(result[6].scroll_delta_y, 0.0);

    // Converter will synthesize an add for device 2.
    ASSERT_EQ(result[7].change, PointerData::Change::kAdd);
    ASSERT_EQ(result[7].signal_kind, PointerData::SignalKind::kNone);
    ASSERT_EQ(result[7].device, 2);
    ASSERT_EQ(result[7].physical_x, 10.0);
    ASSERT_EQ(result[7].physical_y, 20.0);
    ASSERT_EQ(result[7].synthesized, 1);

    ASSERT_EQ(result[8].change, PointerData::Change::kHover);
    ASSERT_EQ(result[8].signal_kind, kind);
    ASSERT_EQ(result[8].device, 2);
    ASSERT_EQ(result[8].physical_x, 10.0);
    ASSERT_EQ(result[8].physical_y, 20.0);
    ASSERT_EQ(result[8].scroll_delta_x, 30.0);
    ASSERT_EQ(result[8].scroll_delta_y, 40.0);
  }
}

TEST(PointerDataPacketConverterTest, CanConvertTrackpadGesture) {
  TestDelegate delegate;
  delegate.AddView(kImplicitViewId);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(3);
  PointerData data;
  CreateSimulatedTrackpadGestureData(data, PointerData::Change::kPanZoomStart,
                                     0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  packet->SetPointerData(0, data);
  CreateSimulatedTrackpadGestureData(data, PointerData::Change::kPanZoomUpdate,
                                     0, 0.0, 0.0, 3.0, 4.0, 1.0, 0.0);
  packet->SetPointerData(1, data);
  CreateSimulatedTrackpadGestureData(data, PointerData::Change::kPanZoomEnd, 0,
                                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  packet->SetPointerData(2, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)4);
  ASSERT_EQ(result[0].change, PointerData::Change::kAdd);
  ASSERT_EQ(result[0].device, 0);
  ASSERT_EQ(result[0].synthesized, 1);

  ASSERT_EQ(result[1].change, PointerData::Change::kPanZoomStart);
  ASSERT_EQ(result[1].signal_kind, PointerData::SignalKind::kNone);
  ASSERT_EQ(result[1].device, 0);
  ASSERT_EQ(result[1].physical_x, 0.0);
  ASSERT_EQ(result[1].physical_y, 0.0);
  ASSERT_EQ(result[1].synthesized, 0);

  ASSERT_EQ(result[2].change, PointerData::Change::kPanZoomUpdate);
  ASSERT_EQ(result[2].signal_kind, PointerData::SignalKind::kNone);
  ASSERT_EQ(result[2].device, 0);
  ASSERT_EQ(result[2].physical_x, 0.0);
  ASSERT_EQ(result[2].physical_y, 0.0);
  ASSERT_EQ(result[2].pan_x, 3.0);
  ASSERT_EQ(result[2].pan_y, 4.0);
  ASSERT_EQ(result[2].pan_delta_x, 3.0);
  ASSERT_EQ(result[2].pan_delta_y, 4.0);
  ASSERT_EQ(result[2].scale, 1.0);
  ASSERT_EQ(result[2].rotation, 0.0);
  ASSERT_EQ(result[2].synthesized, 0);

  ASSERT_EQ(result[3].change, PointerData::Change::kPanZoomEnd);
  ASSERT_EQ(result[3].signal_kind, PointerData::SignalKind::kNone);
  ASSERT_EQ(result[3].device, 0);
  ASSERT_EQ(result[3].physical_x, 0.0);
  ASSERT_EQ(result[3].physical_y, 0.0);
  ASSERT_EQ(result[3].synthesized, 0);
}

TEST(PointerDataPacketConverterTest, CanConvertViewId) {
  TestDelegate delegate;
  delegate.AddView(100);
  delegate.AddView(200);
  PointerDataPacketConverter converter(delegate);
  auto packet = std::make_unique<PointerDataPacket>(2);
  PointerData data;
  CreateSimulatedPointerData(data, PointerData::Change::kAdd, 0, 0.0, 0.0, 0);
  data.view_id = 100;
  packet->SetPointerData(0, data);
  CreateSimulatedPointerData(data, PointerData::Change::kHover, 0, 1.0, 0.0, 0);
  data.view_id = 200;
  packet->SetPointerData(1, data);
  auto converted_packet = converter.Convert(*packet);

  std::vector<PointerData> result;
  UnpackPointerPacket(result, std::move(converted_packet));

  ASSERT_EQ(result.size(), (size_t)2);
  ASSERT_EQ(result[0].view_id, 100);
  ASSERT_EQ(result[1].view_id, 200);
}

}  // namespace testing
}  // namespace flutter
