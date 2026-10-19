/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_IO_ROSBAGREADER_H
#define MINS_IO_ROSBAGREADER_H

#include <cstdint>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace mins {

/// Reads ROS1 bag files (format 2.0) without ROS. Chunks may be uncompressed, bz2 or lz4.
class RosbagReader {
public:
  struct Connection {
    std::string topic;
    std::string type;
    std::string md5sum;
  };

  struct Message {
    const Connection *conn;
    /// Record (receive) time in ns, the order rosbag::View plays in. Sensor time is in the message header.
    std::uint64_t time;
    std::vector<std::uint8_t> data;
  };

  explicit RosbagReader(const std::string &path);

  const std::map<std::uint32_t, Connection> &connections() const { return conns; }

  /// First and last record time in ns over the whole bag, what rosbag::View reports as begin / end
  std::uint64_t start_time() const { return start; }
  std::uint64_t end_time() const { return end; }

  /// Calls f on every message on the given topics (all topics if empty) in the order rosbag::View plays them: by record
  /// time, with View's tie-break between topics. Stops when f returns false.
  void read(const std::set<std::string> &topics, const std::function<bool(const Message &)> &f);

private:
  struct ChunkInfo {
    std::uint64_t pos;
    std::uint64_t start;
    std::uint64_t end;
    std::map<std::uint32_t, std::uint32_t> counts; // messages per connection in this chunk
  };

  std::vector<std::uint8_t> load_chunk(std::uint64_t pos);

  std::ifstream file;
  std::map<std::uint32_t, Connection> conns;
  std::vector<ChunkInfo> chunks;
  std::uint64_t start = 0, end = 0;
};

} // namespace mins

#endif // MINS_IO_ROSBAGREADER_H
