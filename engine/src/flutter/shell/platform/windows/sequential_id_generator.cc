// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/sequential_id_generator.h"

namespace flutter {

namespace {

// Removes |key| from |first|, and |first[key]| from |second|.
template <typename T>
void Remove(uint32_t key, T* first, T* second) {
  auto iter = first->find(key);
  if (iter == first->end())
    return;

  uint32_t second_key = iter->second;
  first->erase(iter);

  iter = second->find(second_key);
  second->erase(iter);
}

}  // namespace

SequentialIdGenerator::SequentialIdGenerator(uint32_t min_id, uint32_t max_id)
    : min_id_(min_id), min_available_id_(min_id), max_id_(max_id) {}

SequentialIdGenerator::~SequentialIdGenerator() {}

uint32_t SequentialIdGenerator::GetGeneratedId(uint32_t number) {
  auto it = number_to_id_.find(number);
  if (it != number_to_id_.end())
    return it->second;

  auto id = GetNextAvailableId();
  number_to_id_.emplace(number, id);
  id_to_number_.emplace(id, number);
  return id;
}

bool SequentialIdGenerator::HasGeneratedIdFor(uint32_t number) const {
  return number_to_id_.find(number) != number_to_id_.end();
}

void SequentialIdGenerator::ReleaseNumber(uint32_t number) {
  if (number_to_id_.count(number) > 0U) {
    UpdateNextAvailableIdAfterRelease(number_to_id_[number]);
    Remove(number, &number_to_id_, &id_to_number_);
  }
}

void SequentialIdGenerator::ReleaseId(uint32_t id) {
  if (id_to_number_.count(id) > 0U) {
    UpdateNextAvailableIdAfterRelease(id);
    Remove(id_to_number_[id], &number_to_id_, &id_to_number_);
  }
}

uint32_t SequentialIdGenerator::GetNextAvailableId() {
  while (id_to_number_.count(min_available_id_) > 0 &&
         min_available_id_ < max_id_) {
    ++min_available_id_;
  }
  if (min_available_id_ >= max_id_)
    min_available_id_ = min_id_;
  return min_available_id_;
}

void SequentialIdGenerator::UpdateNextAvailableIdAfterRelease(uint32_t id) {
  if (id < min_available_id_) {
    min_available_id_ = id;
  }
}

}  // namespace flutter
