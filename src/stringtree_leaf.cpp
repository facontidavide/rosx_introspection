/***** MIT License ****
 *
 *   Copyright (c) 2016-2022 Davide Faconti
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "rosx_introspection/stringtree_leaf.hpp"

namespace RosMsgParser {

// Helper: fill bracket placeholders in a cached path template using segment memcpy.
static size_t fillBrackets(char* buf, const char* tmpl, size_t tmpl_size,
                           const uint16_t* offsets, uint8_t num_brackets,
                           const SmallVector<uint16_t, 4>& index_array) {
  size_t out_off = 0;
  size_t src_off = 0;

  for (uint8_t i = 0; i < num_brackets; i++) {
    size_t seg_len = offsets[i] - src_off;
    std::memcpy(buf + out_off, tmpl + src_off, seg_len);
    out_off += seg_len;

    buf[out_off++] = '[';
    if (i < index_array.size()) {
      out_off += print_number(buf + out_off, index_array[i]);
    }
    buf[out_off++] = ']';
    src_off = offsets[i] + 2;
  }

  size_t tail_len = tmpl_size - src_off;
  if (tail_len > 0) {
    std::memcpy(buf + out_off, tmpl + src_off, tail_len);
    out_off += tail_len;
  }

  return out_off;
}

// FieldLeaf::toStr — uses precomputed path template + bracket offset table.
void FieldLeaf::toStr(std::string& out) const {
  if (!node) {
    out.clear();
    return;
  }
  const auto& tmpl = node->cachedPath();
  const uint8_t num_brackets = node->bracketCount();

  if (num_brackets == 0 && key_suffix.empty()) {
    out = tmpl;
    return;
  }

  size_t extra = num_brackets * 5 + key_suffix.len;
  out.resize(tmpl.size() + extra);

  size_t offset = fillBrackets(out.data(), tmpl.data(), tmpl.size(),
                               node->bracketOffsets(), num_brackets, index_array);

  if (!key_suffix.empty()) {
    std::memcpy(out.data() + offset, key_suffix.data, key_suffix.len);
    offset += key_suffix.len;
  }

  out.resize(offset);
}

// FieldsVector — kept for backward compatibility
FieldsVector::FieldsVector(const FieldLeaf& leaf) : _node(leaf.node) {
  index_array = leaf.index_array;
  key_suffix = leaf.key_suffix;
}

void FieldsVector::toStr(std::string& out) const {
  if (!_node) {
    out.clear();
    return;
  }

  const auto& tmpl = _node->cachedPath();
  const uint8_t num_brackets = _node->bracketCount();

  if (num_brackets == 0 && key_suffix.empty()) {
    out = tmpl;
    return;
  }

  size_t extra = num_brackets * 5 + key_suffix.len;
  out.resize(tmpl.size() + extra);

  size_t offset = fillBrackets(out.data(), tmpl.data(), tmpl.size(),
                               _node->bracketOffsets(), num_brackets, index_array);

  if (!key_suffix.empty()) {
    std::memcpy(out.data() + offset, key_suffix.data, key_suffix.len);
    offset += key_suffix.len;
  }

  out.resize(offset);
}

}  // namespace RosMsgParser
