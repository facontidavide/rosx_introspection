/// Standalone benchmark for IDL deserialization.
/// Reads test data from test/test_data/mcap/ and deserializes N times.
/// Usage: ./idl_benchmark [iterations]

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "rosx_introspection/ros_parser.hpp"

using namespace RosMsgParser;

static std::string readFile(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open: " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static std::vector<uint8_t> readBinaryFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open: " + path);
  }
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(f), {});
}

int main(int argc, char* argv[]) {
  const int iterations = (argc >= 2) ? std::atoi(argv[1]) : 100000;
  const std::string base_path = "test/test_data/mcap/ims_msgs__RoboticsInputs";

  // Load test data
  std::string idl_text = readFile(base_path + ".idl");
  auto cdr_data = readBinaryFile(base_path + ".cdr");

  std::cout << "IDL schema: " << idl_text.size() << " bytes" << std::endl;
  std::cout << "CDR message: " << cdr_data.size() << " bytes" << std::endl;
  std::cout << "Iterations: " << iterations << std::endl;
  std::cout << std::endl;

  // Phase 1: IDL parsing (one-time cost)
  auto t0 = std::chrono::high_resolution_clock::now();
  Parser parser("ims_msgs::RoboticsInputs", ROSType("ims_msgs/RoboticsInputs"), idl_text, DDS_IDL);
  auto t1 = std::chrono::high_resolution_clock::now();

  double parse_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "IDL parse time: " << parse_ms << " ms" << std::endl;

  // Phase 2: Deserialization benchmark
  FlatMessage flat;
  NanoCDR_Deserializer deserializer;
  Span<const uint8_t> buffer(cdr_data);

  // Warmup
  parser.deserialize(buffer, &flat, &deserializer);
  size_t values_per_msg = flat.value.size();
  std::cout << "Values per message: " << values_per_msg << std::endl;
  std::cout << std::endl;

  // Timed loop
  auto t2 = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < iterations; i++) {
    parser.deserialize(buffer, &flat, &deserializer);
  }

  auto t3 = std::chrono::high_resolution_clock::now();

  double total_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
  double per_iter_us = (total_ms * 1000.0) / iterations;
  double msg_per_sec = iterations / (total_ms / 1000.0);
  double mb_per_sec = (cdr_data.size() * iterations) / (total_ms / 1000.0) / (1024.0 * 1024.0);

  std::cout << "=== Deserialization Results ===" << std::endl;
  std::cout << "Total time:    " << total_ms << " ms" << std::endl;
  std::cout << "Per iteration: " << per_iter_us << " us" << std::endl;
  std::cout << "Throughput:    " << msg_per_sec << " msg/s" << std::endl;
  std::cout << "Throughput:    " << mb_per_sec << " MB/s" << std::endl;

  return 0;
}
