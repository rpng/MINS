/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "RosbagReader.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <tuple>

#ifdef MINS_HAVE_BZ2
#include <bzlib.h>
#endif
#ifdef MINS_HAVE_LZ4
#include <lz4frame.h>
#endif

using namespace mins;

namespace {

// Bag record opcodes
constexpr std::uint8_t OP_MSG_DATA = 0x02;
constexpr std::uint8_t OP_BAG_HEADER = 0x03;
constexpr std::uint8_t OP_CHUNK = 0x05;
constexpr std::uint8_t OP_CHUNK_INFO = 0x06;
constexpr std::uint8_t OP_CONNECTION = 0x07;

template <class T> T get(const std::uint8_t *p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

std::uint64_t to_ns(const std::uint8_t *p) { return get<std::uint32_t>(p) * 1000000000ull + get<std::uint32_t>(p + 4); }

/// A record header is a sequence of int32 length + "name=value" fields
std::map<std::string, std::string> parse_fields(const std::uint8_t *p, std::size_t len) {
  std::map<std::string, std::string> out;
  std::size_t i = 0;
  while (i + 4 <= len) {
    std::uint32_t n = get<std::uint32_t>(p + i);
    if (i + 4 + n > len)
      throw std::runtime_error("bag: truncated record header");
    const char *f = reinterpret_cast<const char *>(p + i + 4);
    const char *eq = static_cast<const char *>(std::memchr(f, '=', n));
    if (eq == nullptr)
      throw std::runtime_error("bag: record header field without '='");
    out.emplace(std::string(f, eq), std::string(eq + 1, f + n));
    i += 4 + n;
  }
  return out;
}

/// Walks the records in a buffer: header fields + data span
struct RecordCursor {
  RecordCursor(const std::uint8_t *begin, const std::uint8_t *end) : p(begin), end(end) {}

  const std::uint8_t *p, *end;
  std::map<std::string, std::string> header;
  const std::uint8_t *data = nullptr;
  std::uint32_t data_len = 0;

  bool next() {
    if (end - p < 8)
      return false;
    std::uint32_t hl = get<std::uint32_t>(p);
    if (static_cast<std::size_t>(end - p) < 8ull + hl)
      throw std::runtime_error("bag: truncated record");
    header = parse_fields(p + 4, hl);
    data_len = get<std::uint32_t>(p + 4 + hl);
    data = p + 8 + hl;
    if (data + data_len > end)
      throw std::runtime_error("bag: truncated record data");
    p = data + data_len;
    return true;
  }

  std::uint8_t op() const {
    auto it = header.find("op");
    return it == header.end() || it->second.empty() ? 0 : static_cast<std::uint8_t>(it->second[0]);
  }
  const std::string &field(const std::string &name) const {
    auto it = header.find(name);
    if (it == header.end())
      throw std::runtime_error("bag: record without field " + name);
    return it->second;
  }
};

#ifdef MINS_HAVE_LZ4
std::vector<std::uint8_t> decompress_lz4(const std::vector<std::uint8_t> &in, std::size_t size) {
  std::vector<std::uint8_t> out(size);
  LZ4F_dctx *ctx = nullptr;
  if (LZ4F_isError(LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION)))
    throw std::runtime_error("bag: cannot create lz4 context");
  std::size_t src_pos = 0, dst_pos = 0, ret = 1;
  while (ret != 0 && src_pos < in.size()) {
    std::size_t src_len = in.size() - src_pos, dst_len = out.size() - dst_pos;
    ret = LZ4F_decompress(ctx, out.data() + dst_pos, &dst_len, in.data() + src_pos, &src_len, nullptr);
    if (LZ4F_isError(ret)) {
      LZ4F_freeDecompressionContext(ctx);
      throw std::runtime_error(std::string("bag: lz4 ") + LZ4F_getErrorName(ret));
    }
    if (src_len == 0 && dst_len == 0) {
      LZ4F_freeDecompressionContext(ctx);
      throw std::runtime_error("bag: lz4 chunk larger than its size field");
    }
    src_pos += src_len;
    dst_pos += dst_len;
  }
  LZ4F_freeDecompressionContext(ctx);
  if (dst_pos != size)
    throw std::runtime_error("bag: lz4 chunk decompressed to the wrong size");
  return out;
}
#else
std::vector<std::uint8_t> decompress_lz4(const std::vector<std::uint8_t> &, std::size_t) {
  throw std::runtime_error("bag: this bag has lz4 chunks; install liblz4 (e.g. liblz4-dev, brew lz4) and rebuild MINS");
}
#endif

#ifdef MINS_HAVE_BZ2
std::vector<std::uint8_t> decompress_bz2(std::vector<std::uint8_t> &in, std::size_t size) {
  std::vector<std::uint8_t> out(size);
  unsigned int len = static_cast<unsigned int>(size);
  int r = BZ2_bzBuffToBuffDecompress(reinterpret_cast<char *>(out.data()), &len, reinterpret_cast<char *>(in.data()), static_cast<unsigned int>(in.size()), 0, 0);
  if (r != BZ_OK || len != size)
    throw std::runtime_error("bag: bz2 decompression failed");
  return out;
}
#else
std::vector<std::uint8_t> decompress_bz2(std::vector<std::uint8_t> &, std::size_t) {
  throw std::runtime_error("bag: this bag has bz2 chunks; install libbz2 (e.g. libbz2-dev) and rebuild MINS");
}
#endif

} // namespace

RosbagReader::RosbagReader(const std::string &path) : file(path, std::ios::binary) {
  if (!file)
    throw std::runtime_error("bag: cannot open " + path);
  char magic[13];
  file.read(magic, 13);
  if (!file || std::string(magic, 13) != "#ROSBAG V2.0\n")
    throw std::runtime_error("bag: " + path + " is not a ROS1 bag (format 2.0)");

  // The bag header points at the index section after the last chunk: connection and chunk info records.
  // Only its header fields matter; its data is padding.
  std::uint32_t hl = 0;
  file.read(reinterpret_cast<char *>(&hl), 4);
  std::vector<std::uint8_t> head(hl);
  file.read(reinterpret_cast<char *>(head.data()), hl);
  if (!file)
    throw std::runtime_error("bag: truncated bag header record");
  auto bh = parse_fields(head.data(), head.size());
  if (bh["op"].empty() || static_cast<std::uint8_t>(bh["op"][0]) != OP_BAG_HEADER || bh["index_pos"].size() != 8)
    throw std::runtime_error("bag: missing bag header record");
  std::uint64_t index_pos = get<std::uint64_t>(reinterpret_cast<const std::uint8_t *>(bh["index_pos"].data()));
  if (index_pos == 0)
    throw std::runtime_error("bag: " + path + " has no index (unclosed bag?), run 'rosbag reindex' first");

  file.clear();
  file.seekg(0, std::ios::end);
  std::uint64_t file_size = static_cast<std::uint64_t>(file.tellg());
  if (index_pos > file_size)
    throw std::runtime_error("bag: " + path + " index position is past the end of the file");
  std::vector<std::uint8_t> index(file_size - index_pos);
  file.seekg(static_cast<std::streamoff>(index_pos));
  file.read(reinterpret_cast<char *>(index.data()), static_cast<std::streamsize>(index.size()));

  RecordCursor rc(index.data(), index.data() + index.size());
  while (rc.next()) {
    if (rc.op() == OP_CONNECTION) {
      auto def = parse_fields(rc.data, rc.data_len);
      Connection c{rc.field("topic"), def["type"], def["md5sum"]};
      conns[get<std::uint32_t>(reinterpret_cast<const std::uint8_t *>(rc.field("conn").data()))] = c;
    } else if (rc.op() == OP_CHUNK_INFO) {
      ChunkInfo ci{get<std::uint64_t>(reinterpret_cast<const std::uint8_t *>(rc.field("chunk_pos").data())),
                   to_ns(reinterpret_cast<const std::uint8_t *>(rc.field("start_time").data())),
                   to_ns(reinterpret_cast<const std::uint8_t *>(rc.field("end_time").data())),
                   {}};
      for (std::uint32_t i = 0; i + 8 <= rc.data_len; i += 8)
        ci.counts[get<std::uint32_t>(rc.data + i)] = get<std::uint32_t>(rc.data + i + 4);
      chunks.push_back(ci);
    }
  }
  std::stable_sort(chunks.begin(), chunks.end(), [](const ChunkInfo &a, const ChunkInfo &b) { return a.start < b.start; });
  for (const auto &c : chunks) {
    start = (start == 0 || c.start < start) ? c.start : start;
    end = std::max(end, c.end);
  }
}

std::vector<std::uint8_t> RosbagReader::load_chunk(std::uint64_t pos) {
  file.clear();
  file.seekg(static_cast<std::streamoff>(pos));
  std::uint32_t hl;
  file.read(reinterpret_cast<char *>(&hl), 4);
  std::vector<std::uint8_t> h(hl);
  file.read(reinterpret_cast<char *>(h.data()), hl);
  auto header = parse_fields(h.data(), hl);
  if (header["op"].empty() || static_cast<std::uint8_t>(header["op"][0]) != OP_CHUNK)
    throw std::runtime_error("bag: chunk info points at a record that is not a chunk");
  std::uint32_t data_len;
  file.read(reinterpret_cast<char *>(&data_len), 4);
  std::vector<std::uint8_t> data(data_len);
  file.read(reinterpret_cast<char *>(data.data()), data_len);
  if (!file)
    throw std::runtime_error("bag: truncated chunk");

  const std::string &compression = header["compression"];
  std::size_t size = get<std::uint32_t>(reinterpret_cast<const std::uint8_t *>(header["size"].data()));
  if (compression == "none")
    return data;
  if (compression == "lz4")
    return decompress_lz4(data, size);
  if (compression == "bz2")
    return decompress_bz2(data, size);
  throw std::runtime_error("bag: unsupported chunk compression " + compression);
}

void RosbagReader::read(const std::set<std::string> &topics, const std::function<bool(const Message &)> &f) {
  // rosbag::View keeps one cursor per connection, and after every message re-sorts them by next time (descending) and
  // plays the last one. Equal record times are common (e.g. stereo images), so its tie-break is part of the play order;
  // a stable sort over cursors that start in connection id order reproduces it (View's std::sort is an insertion sort,
  // hence stable, for up to 16 connections in the bag; with more, View's order on equal times is unspecified and may
  // differ from ours).
  using Key = std::tuple<std::uint64_t, std::uint64_t, std::uint32_t>; // time, chunk position, offset: View's index order
  std::map<std::uint32_t, std::map<Key, std::vector<std::uint8_t>>> pending;
  std::map<std::uint32_t, std::uint64_t> unloaded; // messages per connection still in chunks not read yet
  auto wanted = [&](std::uint32_t c) {
    auto it = conns.find(c);
    return it != conns.end() && (topics.empty() || topics.count(it->second.topic) > 0);
  };
  for (const auto &ci : chunks)
    for (const auto &kv : ci.counts)
      if (wanted(kv.first))
        unloaded[kv.first] += kv.second;
  std::vector<std::uint32_t> cursors;
  for (const auto &kv : unloaded)
    cursors.push_back(kv.first);

  std::size_t next_chunk = 0;
  auto load_next = [&]() {
    const ChunkInfo &ci = chunks[next_chunk++];
    std::vector<std::uint8_t> buf = load_chunk(ci.pos);
    RecordCursor rc(buf.data(), buf.data() + buf.size());
    while (rc.next()) {
      if (rc.op() != OP_MSG_DATA)
        continue;
      std::uint32_t conn = get<std::uint32_t>(reinterpret_cast<const std::uint8_t *>(rc.field("conn").data()));
      if (!wanted(conn))
        continue;
      std::uint32_t offset = static_cast<std::uint32_t>(rc.data - buf.data());
      pending[conn].emplace(Key{to_ns(reinterpret_cast<const std::uint8_t *>(rc.field("time").data())), ci.pos, offset}, std::vector<std::uint8_t>(rc.data, rc.data + rc.data_len));
      if (unloaded[conn] > 0)
        unloaded[conn]--;
    }
  };
  auto next_time = [&](std::uint32_t c) { return std::get<0>(pending[c].begin()->first); };

  while (true) {
    // Every cursor needs its true next message: load until each live one has a message, and until no unread chunk
    // starts before the latest of those (it could hold an earlier message for that connection)
    while (next_chunk < chunks.size()) {
      bool missing = false;
      std::uint64_t latest = 0;
      for (std::uint32_t c : cursors) {
        if (pending[c].empty())
          missing = missing || unloaded[c] > 0;
        else
          latest = std::max(latest, next_time(c));
      }
      if (!missing && chunks[next_chunk].start > latest)
        break;
      load_next();
    }
    cursors.erase(std::remove_if(cursors.begin(), cursors.end(), [&](std::uint32_t c) { return pending[c].empty() && (unloaded[c] == 0 || next_chunk == chunks.size()); }),
                  cursors.end());
    if (cursors.empty())
      return;
    std::stable_sort(cursors.begin(), cursors.end(), [&](std::uint32_t a, std::uint32_t b) { return next_time(a) > next_time(b); });
    auto &queue = pending[cursors.back()];
    auto first = queue.begin();
    Message msg{&conns.at(cursors.back()), std::get<0>(first->first), std::move(first->second)};
    queue.erase(first);
    if (!f(msg))
      return;
  }
}
