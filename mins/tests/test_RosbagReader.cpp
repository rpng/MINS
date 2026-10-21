/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Checks RosbagReader against bags written here byte by byte, so no binary fixtures are needed.
#include <gtest/gtest.h>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#ifdef MINS_HAVE_BZ2
#include <bzlib.h>
#endif
#ifdef MINS_HAVE_LZ4
#include <lz4frame.h>
#endif
#include <string>
#include <vector>
#include "io/RosbagReader.h"

using namespace mins;

namespace {
std::string u32(std::uint32_t v) { return std::string(reinterpret_cast<const char *>(&v), 4); }
std::string u64(std::uint64_t v) { return std::string(reinterpret_cast<const char *>(&v), 8); }
std::string stamp(std::uint64_t ns) { return u32(static_cast<std::uint32_t>(ns / 1000000000)) + u32(static_cast<std::uint32_t>(ns % 1000000000)); }
std::string field(const std::string &name, const std::string &value) { return u32(static_cast<std::uint32_t>(name.size() + 1 + value.size())) + name + "=" + value; }
std::string record(const std::string &header, const std::string &data) { return u32(static_cast<std::uint32_t>(header.size())) + header + u32(static_cast<std::uint32_t>(data.size())) + data; }

struct Msg {
    std::uint32_t conn;
    std::uint64_t time;
    std::string data;
};

// Writes the parts of a bag 2.0 file that RosbagReader reads: bag header, chunks, connection and chunk info records.
std::string write_bag(const std::vector<std::string> &topics, const std::vector<std::vector<Msg>> &chunks, const std::string &compression,
                      bool indexed = true) {
    std::string body;
    std::string chunk_infos;
    // rosbag pads the bag header record so its header + data are 4096 bytes, after the version line and two lengths
    const std::size_t start = 13 + 8 + 4096;
    for (const auto &chunk : chunks) {
        std::string raw;
        for (const auto &m : chunk)
            raw += record(field("op", "\x02") + field("conn", u32(m.conn)) + field("time", stamp(m.time)), m.data);
        std::string packed = raw;
#ifdef MINS_HAVE_LZ4
        if (compression == "lz4") {
            packed.resize(LZ4F_compressFrameBound(raw.size(), nullptr));
            packed.resize(LZ4F_compressFrame(&packed[0], packed.size(), raw.data(), raw.size(), nullptr));
        }
#endif
#ifdef MINS_HAVE_BZ2
        if (compression == "bz2") {
            unsigned int len = static_cast<unsigned int>(raw.size() * 2 + 600);
            packed.resize(len);
            BZ2_bzBuffToBuffCompress(&packed[0], &len, const_cast<char *>(raw.data()), static_cast<unsigned int>(raw.size()), 9, 0, 30);
            packed.resize(len);
        }
#endif
        std::uint64_t pos = start + body.size();
        body += record(field("op", "\x05") + field("compression", compression) + field("size", u32(static_cast<std::uint32_t>(raw.size()))), packed);
        std::uint64_t t0 = chunk.front().time, t1 = chunk.front().time;
        std::map<std::uint32_t, std::uint32_t> counts;
        for (const auto &m : chunk) {
            t0 = std::min(t0, m.time);
            t1 = std::max(t1, m.time);
            counts[m.conn]++;
        }
        std::string per_conn;
        for (const auto &kv : counts)
            per_conn += u32(kv.first) + u32(kv.second);
        chunk_infos += record(field("op", "\x06") + field("ver", u32(1)) + field("chunk_pos", u64(pos)) + field("start_time", stamp(t0)) +
                                  field("end_time", stamp(t1)) + field("count", u32(static_cast<std::uint32_t>(counts.size()))),
                              per_conn);
    }
    std::uint64_t index_pos = start + body.size();
    std::string conn_records;
    for (std::uint32_t c = 0; c < topics.size(); c++)
        conn_records += record(field("op", "\x07") + field("conn", u32(c)) + field("topic", topics[c]),
                               field("topic", topics[c]) + field("type", "std_msgs/String") + field("md5sum", "992ce8a1687cec8c8bd883ec73ca41d1"));
    // without an index, connections go first, as a writer streaming the bag puts them
    body = indexed ? body + conn_records + chunk_infos : conn_records + body;

    std::string header = field("op", "\x03") + field("index_pos", u64(indexed ? index_pos : 0)) +
                         field("conn_count", u32(static_cast<std::uint32_t>(topics.size()))) +
                         field("chunk_count", u32(static_cast<std::uint32_t>(chunks.size())));
    std::string bag = "#ROSBAG V2.0\n" + record(header, std::string(4096 - header.size(), ' ')) + body;

    std::string path = testing::TempDir() + "test_rosbag_" + compression + ".bag";
    std::ofstream(path, std::ios::binary) << bag;
    return path;
}

std::vector<Msg> read_all(RosbagReader &bag, const std::set<std::string> &topics = {}) {
    std::vector<Msg> out;
    bag.read(topics, [&](const RosbagReader::Message &m) {
        out.push_back({0, m.time, std::string(m.data.begin(), m.data.end()) + "@" + m.conn->topic});
        return true;
    });
    return out;
}
} // namespace

class RosbagReaderCompression : public testing::TestWithParam<std::string> {};

TEST_P(RosbagReaderCompression, ReadsEveryMessageWithItsTopicAndTime) {
    RosbagReader bag(write_bag({"/imu", "/cam"}, {{{0, 10, "a"}, {1, 20, "b"}}, {{0, 30, "c"}}}, GetParam()));
    ASSERT_EQ(bag.connections().size(), 2u);
    EXPECT_EQ(bag.connections().at(1).topic, "/cam");
    EXPECT_EQ(bag.connections().at(1).type, "std_msgs/String");
    auto msgs = read_all(bag);
    ASSERT_EQ(msgs.size(), 3u);
    EXPECT_EQ(msgs[0].data, "a@/imu");
    EXPECT_EQ(msgs[1].data, "b@/cam");
    EXPECT_EQ(msgs[2].data, "c@/imu");
    EXPECT_EQ(msgs[2].time, 30u);
}

// Compressed cases only run when the build found the library (both are optional)
const std::vector<std::string> compressions = {"none",
#ifdef MINS_HAVE_BZ2
                                               "bz2",
#endif
#ifdef MINS_HAVE_LZ4
                                               "lz4",
#endif
};
INSTANTIATE_TEST_SUITE_P(RosbagReader, RosbagReaderCompression, testing::ValuesIn(compressions));

TEST(RosbagReader, MergesOverlappingChunksIntoTimeOrder) {
    // Chunk times interleave, as they do when a writer flushes per topic; output must still be sorted by time
    RosbagReader bag(write_bag({"/a", "/b"}, {{{0, 1, "1"}, {0, 3, "3"}, {0, 5, "5"}}, {{1, 2, "2"}, {1, 4, "4"}}}, "none"));
    std::string order;
    for (const auto &m : read_all(bag))
        order += m.data.substr(0, 1);
    EXPECT_EQ(order, "12345");
}

TEST(RosbagReader, BreaksTimeTiesLikeRosbagView) {
    // View plays the last of its time-sorted cursors, which for a tie between fresh cursors is the higher connection id
    RosbagReader bag(write_bag({"/a", "/b"}, {{{0, 7, "x"}, {1, 7, "y"}, {0, 8, "z"}}}, "none"));
    std::string order;
    for (const auto &m : read_all(bag))
        order += m.data.substr(0, 1);
    EXPECT_EQ(order, "yxz");
}

TEST(RosbagReader, LoadsAheadForASparseTopic) {
    // /gps has its next message several chunks later; it must still play in time order, not after the dense topic
    RosbagReader bag(write_bag({"/imu", "/gps"}, {{{1, 1, "g"}, {0, 2, "a"}}, {{0, 3, "b"}}, {{0, 4, "c"}}, {{1, 5, "h"}, {0, 6, "d"}}}, "none"));
    std::string order;
    for (const auto &m : read_all(bag))
        order += m.data.substr(0, 1);
    EXPECT_EQ(order, "gabchd");
}

TEST(RosbagReader, LeavesChunksPastThePlayTimeUnread) {
    // /gps starts in the last chunk. Playing the first messages must not read that far ahead (a late or gappy topic
    // would hold everything before it in memory), so the last chunk is made unreadable to see whether it was read.
    std::string path = write_bag({"/imu", "/gps"}, {{{0, 1, "a"}}, {{0, 2, "b"}}, {{0, 3, "c"}, {1, 4, "g"}}}, "none");
    std::ifstream in(path, std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    bytes.replace(bytes.rfind("compression=none") + 12, 4, "nope");
    std::ofstream(path, std::ios::binary) << bytes;
    RosbagReader bag(path);
    int n = 0;
    EXPECT_NO_THROW(bag.read({}, [&](const RosbagReader::Message &) { return ++n < 2; }));
    EXPECT_EQ(n, 2);
    EXPECT_THROW(read_all(bag), std::runtime_error);
}

TEST(RosbagReader, FiltersByTopic) {
    RosbagReader bag(write_bag({"/imu", "/cam"}, {{{0, 1, "a"}, {1, 2, "b"}, {0, 3, "c"}}}, "none"));
    auto msgs = read_all(bag, {"/cam"});
    ASSERT_EQ(msgs.size(), 1u);
    EXPECT_EQ(msgs[0].data, "b@/cam");
}

TEST(RosbagReader, StopsWhenTheCallbackReturnsFalse) {
    RosbagReader bag(write_bag({"/imu"}, {{{0, 1, "a"}, {0, 2, "b"}, {0, 3, "c"}}}, "none"));
    int n = 0;
    bag.read({}, [&](const RosbagReader::Message &) { return ++n < 2; });
    EXPECT_EQ(n, 2);
}

TEST(RosbagReader, PlaysABagWithoutIndexInFileOrder) {
    // A bag read while it is written (e.g. through a pipe) has no index; it plays as written, filtered by topic
    RosbagReader bag(write_bag({"/imu", "/cam"}, {{{0, 5, "a"}, {1, 3, "b"}}, {{0, 7, "c"}, {1, 6, "d"}}}, "none", false));
    EXPECT_EQ(bag.start_time(), 3u);
    EXPECT_EQ(bag.end_time(), UINT64_MAX);
    std::string order;
    for (const auto &m : read_all(bag, {"/imu"}))
        order += m.data.substr(0, 1);
    EXPECT_EQ(order, "ac");
}

TEST(RosbagReader, RejectsAFileThatIsNotABag) {
    std::string path = testing::TempDir() + "test_rosbag_not_a_bag.bag";
    std::ofstream(path) << "hello";
    EXPECT_THROW(RosbagReader{path}, std::runtime_error);
}
