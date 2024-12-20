// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_POINTER_DATA_PACKET_CONVERTER_H_
#define FLUTTER_LIB_UI_WINDOW_POINTER_DATA_PACKET_CONVERTER_H_

#include <cstring>
#include <map>
#include <memory>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/window/pointer_data_packet.h"

namespace flutter {

//------------------------------------------------------------------------------
/// The current information about a pointer.
///
/// This struct is used by PointerDataPacketConverter to fill in necessary
/// information for the raw pointer packet sent from embedding. This struct also
/// stores the button state of the last pointer down, up, move, or hover event.
/// When an embedder issues a pointer up or down event where the pointer's
/// position has changed since the last move or hover event,
/// PointerDataPacketConverter generates a synthetic move or hover to notify the
/// framework. In these cases, these events must be issued with the button state
/// prior to the pointer up or down.
///
struct PointerState {
  int64_t pointer_identifier;
  bool is_down;
  bool is_pan_zoom_active;
  double physical_x;
  double physical_y;
  double pan_x;
  double pan_y;
  double scale;
  double rotation;
  int64_t buttons;
  int64_t view_id;
};

//------------------------------------------------------------------------------
/// Converter to convert the raw pointer data packet from the platforms.
///
/// Framework requires certain information to process pointer data. e.g. pointer
/// identifier and the delta of pointer moment. The converter keeps track each
/// pointer state and fill in those information appropriately.
///
/// The converter is also resposible for providing a clean pointer data stream.
/// It will attempt to correct the stream if the it contains illegal pointer
/// transitions.
///
/// Example 1 Missing Add:
///
///     Down(position x) -> Up(position x)
///
///     ###After Conversion###
///
///     Synthesized_Add(position x) -> Down(position x) -> Up(position x)
///
/// Example 2 Missing another move:
///
///     Add(position x) -> Down(position x) -> Move(position y) ->
///     Up(position z)
///
///     ###After Conversion###
///
///     Add(position x) -> Down(position x) -> Move(position y) ->
///     Synthesized_Move(position z) -> Up(position z)
///
/// Platform view is the only client that uses this class to convert all the
/// incoming pointer packet and is responsible for the life cycle of its
/// instance.
///
class PointerDataPacketConverter {
 public:
  // Used by PointerDataPacketConverter to query the system status.
  //
  // Typically RuntimeController.
  class Delegate {
   public:
    Delegate() = default;

    virtual ~Delegate() = default;

    // Returns true if the specified view exists.
    virtual bool ViewExists(int64_t view_id) const = 0;
  };

  //----------------------------------------------------------------------------
  /// @brief      Create a PointerDataPacketConverter.
  ///
  /// @param[in]  delegate   A delegate to fulfill the query to the app state.
  ///                        The delegate must exist throughout the lifetime
  ///                        of this class. Typically `RuntimeController`.
  explicit PointerDataPacketConverter(const Delegate& delegate);
  ~PointerDataPacketConverter();

  //----------------------------------------------------------------------------
  /// @brief      Converts pointer data packet into a form that framework
  ///             understands. The raw pointer data packet from embedding does
  ///             not have sufficient information and may contain illegal
  ///             pointer transitions. This method will fill out that
  ///             information and attempt to correct pointer transitions.
  ///
  ///             Pointer data with invalid view IDs will be ignored.
  ///
  /// @param[in]  packet                   The raw pointer packet sent from
  ///                                      embedding.
  ///
  /// @return     A full converted packet with all the required information
  ///             filled. It may contain synthetic pointer data as the result of
  ///             converter's attempt to correct illegal pointer transitions.
  ///
  std::unique_ptr<PointerDataPacket> Convert(const PointerDataPacket& packet);

 private:
  const Delegate& delegate_;

  // A map from pointer device ID to the state of the pointer.
  std::map<int64_t, PointerState> states_;

  int64_t pointer_ = 0;

  void ConvertPointerData(PointerData pointer_data,
                          std::vector<PointerData>& converted_pointers);

  PointerState EnsurePointerState(PointerData pointer_data);

  void UpdateDeltaAndState(PointerData& pointer_data, PointerState& state);

  void UpdatePointerIdentifier(PointerData& pointer_data,
                               PointerState& state,
                               bool start_new_pointer);

  bool LocationNeedsUpdate(const PointerData pointer_data,
                           const PointerState state);

  FML_DISALLOW_COPY_AND_ASSIGN(PointerDataPacketConverter);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_POINTER_DATA_PACKET_CONVERTER_H_
