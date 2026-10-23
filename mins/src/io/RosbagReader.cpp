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
  LZ4F_decompressionContext_t ctx = nullptr; // LZ4F_dctx is missing from 18.04 liblz4 r131
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

/// Reads one whole record (length fields included) from the current position; empty at the end of the file
std::vector<std::uint8_t> read_record(std::istream &in) {
  std::vector<std::uint8_t> rec;
  auto more = [&](std::size_t n) {
    std::size_t at = rec.size();
    rec.resize(at + n);
    in.read(reinterpret_cast<char *>(rec.data() + at), static_cast<std::streamsize>(n));
    return static_cast<std::size_t>(in.gcount());
  };
  std::size_t got = more(4);
  if (got == 0)
    return {};
  if (got != 4)
    throw std::runtime_error("bag: truncated record");
  std::uint32_t hl = get<std::uint32_t>(rec.data());
  if (more(hl + 4ull) != hl + 4ull)
    throw std::runtime_error("bag: truncated record");
  std::uint32_t dl = get<std::uint32_t>(rec.data() + 4 + hl);
  if (more(dl) != dl)
    throw std::runtime_error("bag: truncated record");
  return rec;
}

std::vector<std::uint8_t> unpack_chunk(const RecordCursor &rc) {
  std::vector<std::uint8_t> data(rc.data, rc.data + rc.data_len);
  const std::string &compression = rc.field("compression");
  std::size_t size = get<std::uint32_t>(reinterpret_cast<const std::uint8_t *>(rc.field("size").data()));
  if (compression == "none")
    return data;
  if (compression == "lz4")
    return decompress_lz4(data, size);
  if (compression == "bz2")
    return decompress_bz2(data, size);
  throw std::runtime_error("bag: unsupported chunk compression " + compression);
}

std::pair<std::uint32_t, RosbagReader::Connection> connection(const RecordCursor &rc) {
  auto def = parse_fields(rc.data, rc.data_len);
  return {get<std::uint32_t>(reinterpret_cast<const std::uint8_t *>(rc.field("conn").data())), {rc.field("topic"), def["type"], def["md5sum"]}};
}

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
  if (index_pos == 0) {
    // No index: a bag still being written, e.g. one coming through a pipe. It plays in file order, so its writer must
    // write in time order and put each connection record before that connection's first message. The start is the
    // first chunk's earliest message; the end is not known.
    stream = true;
    end = UINT64_MAX;
    std::uint32_t pad = 0;
    file.read(reinterpret_cast<char *>(&pad), 4);
    file.ignore(pad);
    for (std::vector<std::uint8_t> rec; first_chunk.empty() && !(rec = read_record(file)).empty();) {
      RecordCursor rc(rec.data(), rec.data() + rec.size());
      rc.next();
      if (rc.op() == OP_CONNECTION)
        conns.insert(connection(rc));
      else if (rc.op() == OP_CHUNK)
        first_chunk = unpack_chunk(rc);
    }
    RecordCursor rc(first_chunk.data(), first_chunk.data() + first_chunk.size());
    while (rc.next())
      if (rc.op() == OP_MSG_DATA) {
        std::uint64_t t = to_ns(reinterpret_cast<const std::uint8_t *>(rc.field("time").data()));
        start = (start == 0 || t < start) ? t : start;
      }
    return;
  }

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
      conns.insert(connection(rc));
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
  std::vector<std::uint8_t> rec = read_record(file);
  RecordCursor rc(rec.data(), rec.data() + rec.size());
  if (!rc.next() || rc.op() != OP_CHUNK)
    throw std::runtime_error("bag: chunk info points at a record that is not a chunk");
  return unpack_chunk(rc);
}

void RosbagReader::read(const std::set<std::string> &topics, const std::function<bool(const Message &)> &f) {
  // rosbag::View keeps one cursor per connection, and after every message stable-sorts them by next time (descending)
  // and plays the last one. Equal record times are common (e.g. stereo images), so its tie-break is part of the play
  // order. Followed through, the stable sort comes down to: on equal times, the connection whose next message became
  // its next most recently plays first, and connections that have not played yet go highest id first. (View's
  // std::sort is an insertion sort, hence stable, for up to 16 connections in the bag; with more, View's order on
  // equal times is unspecified and may differ from ours.)
  using Key = std::tuple<std::uint64_t, std::uint64_t, std::uint32_t>; // time, chunk position, offset: View's index order
  std::map<std::uint32_t, std::map<Key, std::vector<std::uint8_t>>> pending;
  std::map<std::uint32_t, std::uint64_t> played_at; // step at which each connection last played, 0 before its first
  auto wanted = [&](std::uint32_t c) {
    auto it = conns.find(c);
    return it != conns.end() && (topics.empty() || topics.count(it->second.topic) > 0);
  };
  if (stream) {
    read_stream(wanted, f);
    return;
  }

  std::vector<const ChunkInfo *> todo; // chunks holding a wanted message, by start time
  for (const auto &ci : chunks)
    if (std::any_of(ci.counts.begin(), ci.counts.end(), [&](const std::pair<const std::uint32_t, std::uint32_t> &kv) { return wanted(kv.first); }))
      todo.push_back(&ci);

  std::size_t next_chunk = 0;
  auto load_next = [&]() {
    const ChunkInfo &ci = *todo[next_chunk++];
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
    }
  };

  for (std::uint64_t step = 1;; step++) {
    // Pick the next message among the loaded ones, and load on only while an unread chunk starts no later than it: a
    // chunk starting later cannot hold an earlier message, so a topic that starts late or has a gap holds nothing back
    std::uint32_t best = 0;
    std::uint64_t best_time = 0;
    bool found = false;
    while (true) {
      for (const auto &kv : pending) {
        if (kv.second.empty())
          continue;
        std::uint64_t t = std::get<0>(kv.second.begin()->first);
        if (!found || t < best_time || (t == best_time && (played_at[kv.first] > played_at[best] || (played_at[kv.first] == played_at[best] && kv.first > best)))) {
          best = kv.first;
          best_time = t;
          found = true;
        }
      }
      if (next_chunk == todo.size() || (found && todo[next_chunk]->start > best_time))
        break;
      load_next();
      found = false;
    }
    if (!found)
      return;
    auto &queue = pending[best];
    auto first = queue.begin();
    Message msg{&conns.at(best), best_time, std::move(first->second)};
    queue.erase(first);
    played_at[best] = step;
    if (!f(msg))
      return;
  }
}

void RosbagReader::read_stream(const std::function<bool(std::uint32_t)> &wanted, const std::function<bool(const Message &)> &f) {
  std::vector<std::uint8_t> chunk = std::move(first_chunk);
  for (bool have = !chunk.empty(); have;) {
    RecordCursor rc(chunk.data(), chunk.data() + chunk.size());
    while (rc.next()) {
      if (rc.op() == OP_CONNECTION) {
        conns.insert(connection(rc));
      } else if (rc.op() == OP_MSG_DATA) {
        std::uint32_t conn = get<std::uint32_t>(reinterpret_cast<const std::uint8_t *>(rc.field("conn").data()));
        if (conns.count(conn) == 0)
          throw std::runtime_error("bag: message before its connection record");
        if (!wanted(conn))
          continue;
        Message msg{&conns.at(conn), to_ns(reinterpret_cast<const std::uint8_t *>(rc.field("time").data())), std::vector<std::uint8_t>(rc.data, rc.data + rc.data_len)};
        if (!f(msg))
          return;
      }
    }
    have = false;
    for (std::vector<std::uint8_t> rec; !have && !(rec = read_record(file)).empty();) {
      RecordCursor top(rec.data(), rec.data() + rec.size());
      top.next();
      if (top.op() == OP_CONNECTION) {
        conns.insert(connection(top));
      } else if (top.op() == OP_CHUNK) {
        chunk = unpack_chunk(top);
        have = true;
      }
    }
  }
}
