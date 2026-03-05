/***** MIT License ****
 *
 *   Copyright (c) 2016-2024 Davide Faconti
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

#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>

#include "rosbag2_cpp/reader.hpp"
#include "rosx_introspection/ros_parser.hpp"

struct TopicStats {
  size_t message_count = 0;
  size_t total_bytes = 0;
  double total_deserialize_ms = 0;
  double total_json_ms = 0;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <mcap_file> [--iterations N] [--json]" << std::endl;
    return 1;
  }

  std::string mcap_file = argv[1];
  int iterations = 1;
  bool bench_json = false;

  for (int i = 2; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--iterations" && i + 1 < argc) {
      iterations = std::stoi(argv[++i]);
    } else if (arg == "--json") {
      bench_json = true;
    }
  }

  std::cout << "Benchmark: " << mcap_file << std::endl;
  std::cout << "Iterations: " << iterations << std::endl;
  std::cout << "JSON benchmark: " << (bench_json ? "yes" : "no") << std::endl;
  std::cout << "---" << std::endl;

  for (int iter = 0; iter < iterations; iter++) {
    rosbag2_cpp::Reader reader;
    reader.open(mcap_file);

    std::unordered_map<std::string, RosMsgParser::Parser> parsers;
    std::unordered_map<std::string, TopicStats> stats;
    RosMsgParser::FlatMessage flat_msg;
    RosMsgParser::NanoCDR_Deserializer deserializer;
    std::string json_output;

    auto total_start = std::chrono::high_resolution_clock::now();

    while (reader.has_next()) {
      auto msg = reader.read_next();

      auto topic_name = msg->topic_name;
      auto it = parsers.find(topic_name);

      if (it == parsers.end()) {
        // Get topic metadata to find schema
        auto topics = reader.get_all_topics_and_types();
        std::string type_name;
        for (const auto& topic_info : topics) {
          if (topic_info.name == topic_name) {
            type_name = topic_info.type;
            break;
          }
        }

        if (type_name.empty()) {
          continue;
        }

        // Get the schema definition
        auto metadata = reader.get_metadata();
        std::string schema_def;
        for (const auto& topic_with_type : metadata.topics_with_message_count) {
          if (topic_with_type.topic_metadata.name == topic_name) {
            // Schema is obtained from the reader's offered_qos_profiles or from message
            break;
          }
        }

        // Try creating a parser - skip topic if schema is not available
        try {
          auto [inserted_it, inserted] =
              parsers.emplace(topic_name, RosMsgParser::Parser(topic_name, RosMsgParser::ROSType(type_name), ""));
          it = inserted_it;
        } catch (const std::exception& e) {
          continue;
        }
      }

      auto& parser = it->second;
      auto& topic_stats = stats[topic_name];

      RosMsgParser::Span<const uint8_t> buffer(msg->serialized_data->buffer, msg->serialized_data->buffer_length);
      topic_stats.total_bytes += buffer.size();
      topic_stats.message_count++;

      // Benchmark deserialize
      auto t0 = std::chrono::high_resolution_clock::now();
      try {
        parser.deserialize(buffer, &flat_msg, &deserializer);

        std::string field_name;
        for (const auto& pair : flat_msg.value) {
          pair.first.toStr(field_name);
        }
      } catch (const std::exception& e) {
        // Skip failed messages
        continue;
      }
      auto t1 = std::chrono::high_resolution_clock::now();
      topic_stats.total_deserialize_ms +=
          std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;

      // Optionally benchmark JSON
      if (bench_json) {
        auto t2 = std::chrono::high_resolution_clock::now();
        try {
          parser.deserializeIntoJson(buffer, &json_output, &deserializer);
          
          std::string field_name;
          for (const auto& pair : flat_msg.value) {
            pair.first.toStr(field_name);
          }
        } catch (const std::exception& e) {
          // Skip failed messages
        }
        auto t3 = std::chrono::high_resolution_clock::now();
        topic_stats.total_json_ms += std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() / 1000.0;
      }
    }

    auto total_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() / 1000.0;

    // Print results
    if (iterations > 1) {
      std::cout << "\n=== Iteration " << (iter + 1) << " ===" << std::endl;
    }

    size_t total_messages = 0;
    size_t total_bytes = 0;

    for (const auto& [topic, s] : stats) {
      total_messages += s.message_count;
      total_bytes += s.total_bytes;

      double msgs_per_sec = s.message_count / (s.total_deserialize_ms / 1000.0);
      double mb_per_sec = (s.total_bytes / (1024.0 * 1024.0)) / (s.total_deserialize_ms / 1000.0);

      std::cout << topic << ":" << std::endl;
      std::cout << "  messages: " << s.message_count << std::endl;
      std::cout << "  bytes: " << s.total_bytes << std::endl;
      std::cout << "  deserialize: " << s.total_deserialize_ms << " ms" << " (" << msgs_per_sec << " msg/s, "
                << mb_per_sec << " MB/s)" << std::endl;

      if (bench_json && s.total_json_ms > 0) {
        double json_msgs_per_sec = s.message_count / (s.total_json_ms / 1000.0);
        std::cout << "  json: " << s.total_json_ms << " ms" << " (" << json_msgs_per_sec << " msg/s)" << std::endl;
      }
    }

    std::cout << "\nTotal: " << total_messages << " messages, " << total_bytes << " bytes in " << total_ms << " ms"
              << std::endl;
  }

  return 0;
}
