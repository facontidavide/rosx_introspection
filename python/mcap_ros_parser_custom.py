import sys
import io
import msgpack
from mcap.reader import make_reader # install with "pip install mcap"
import rosx_introspection
import pandas as pd
from collections import defaultdict
import argparse
from typing import Iterable
import re

def parse_generic_msgpack(msgpack_bytes) -> dict[str, any]:
    result = {}
    """Convert generic msgpack bytes to a dictionary"""
    unpacker = msgpack.Unpacker(io.BytesIO(msgpack_bytes), raw=False)
    flatmap_size = unpacker.read_map_header()

    # add each key/value pair in the map to the result
    for _i in range(flatmap_size):
        key = unpacker.unpack()
        value = unpacker.unpack()
        if key.startswith("/"):
            key = key[1:]  # remove leading slash
        result[key] = value
    return result

class PatternChecker:
    def __init__(self, pattern: str):
        self.pattern = pattern
        parts = pattern.split('#')
        self.prefix = parts[0]
        self.suffix = parts[-1]

    def match(self, key: str) -> bool:
        return key.startswith(self.prefix) and key.endswith(self.suffix)

def unpack_key_value(unpacker):
    key = unpacker.unpack()
    if key.startswith("/"):
        key = key[1:]  # remove leading slash
    value = unpacker.unpack()
    return key, value

def parse_RobotJointCommand(msgpack_bytes) -> dict[str, any]:
    result = {}
    names = []
    enables = []
    positions = []
    
    check_name = PatternChecker("name[#]")
    check_enable = PatternChecker("enable[#]")
    check_position = PatternChecker("position[#]")

    unpacker = msgpack.Unpacker(io.BytesIO(msgpack_bytes), raw=False)
    flatmap_size = unpacker.read_map_header()

    # print("------------------")     
    # add each key/value pair in the map to the result
    for _i in range(flatmap_size):
        key, value = unpack_key_value(unpacker)
        # print(f"  {key}: {value}")

        if check_name.match(key): names.append(value); continue
        if check_position.match(key): positions.append(value); continue
        if check_enable.match(key): enables.append(value); continue
        # default fallback
        result[key] = value

    names_count = len(names)
    if names_count == len(positions):
        for i in range(names_count):
            result[f"position.{names[i]}"] = positions[i]
    if names_count == len(enables):
        for i in range(names_count):
            result[f"enable.{names[i]}"] = enables[i]
    # print("-----")      
    # for key, value in result.items():
    #     print(f"  {key}: {value}")
    return result


def parse_RobotJointState(msgpack_bytes) -> dict[str, any]:
    result = {}
    names = []
    enables = []
    faults = []
    warns = []
    positions = []
    velocities = []
    efforts = []
    raw_efforts = []
    position_following_errors = []

    check_name = PatternChecker("name[#]")
    check_enable = PatternChecker("enable[#]")
    check_fault = PatternChecker("fault[#]")
    check_warn = PatternChecker("warn[#]")
    check_position = PatternChecker("position[#]")
    check_velocity = PatternChecker("velocity[#]")
    check_effort = PatternChecker("effort[#]")
    check_raw_effort = PatternChecker("raw_effort[#]")
    check_position_following_error = PatternChecker("position_following_error[#]")

    unpacker = msgpack.Unpacker(io.BytesIO(msgpack_bytes), raw=False)

    flatmap_size = unpacker.read_map_header()
    i = 0
    
    # unpack all the fields before "name[#]" 
    while i < flatmap_size:
        key, value = unpack_key_value(unpacker)
        i += 1
        if check_name.match(key):
            break
        result[key] = value
    
    # unpack all the fields between "name" and "position_following_error"
    # this code will work ONLY if we don't change the order of the fields in the schema
    while(check_name.match(key)):
        names.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_enable.match(key)):
        enables.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_fault.match(key)):
        faults.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_warn.match(key)):
        warns.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_position.match(key)):
        positions.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_velocity.match(key)):
        velocities.append(value);
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_effort.match(key)):
        efforts.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_raw_effort.match(key)):
        raw_efforts.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1
    while(check_position_following_error.match(key)):
        position_following_errors.append(value)
        key, value = unpack_key_value(unpacker)
        i += 1

    # save the latest unmatched check_position_following_error
    result[key] = value
    
    # unpack fields left
    while i < flatmap_size:
        key, value = unpack_key_value(unpacker)
        i += 1
        result[key] = value

    # use the name to create per-joint fields
    names_count = len(names)
    if names_count == len(enables):
        for i in range(names_count):
            result[f"enable.{names[i]}"] = enables[i]
    if names_count == len(faults):
        for i in range(names_count):
            result[f"fault.{names[i]}"] = faults[i]
    if names_count == len(warns):
        for i in range(names_count):
            result[f"warn.{names[i]}"] = warns[i]
    if names_count == len(positions):
        for i in range(names_count):
            result[f"position.{names[i]}"] = positions[i]
    if names_count == len(velocities):
        for i in range(names_count):
            result[f"velocity.{names[i]}"] = velocities[i]
    if names_count == len(efforts):
        for i in range(names_count):
            result[f"effort.{names[i]}"] = efforts[i]
    if names_count == len(raw_efforts):
        for i in range(names_count):
            result[f"raw_effort.{names[i]}"] = raw_efforts[i]
    if names_count == len(position_following_errors):
        for i in range(names_count):
            result[f"position_following_error.{names[i]}"] = position_following_errors[i]
    # print("-----")      
    # for key, value in result.items():
    #     print(f"  {key}: {value}")
    return result


def parse_mcap_file(mcap_file: str, topics_filter: Iterable[str] = None, topic_name_as_prefix: bool = True):
    """Parse an MCAP file and return DataFrames for each topic"""

    with open(mcap_file, "rb") as f:
        mcap_reader = make_reader(f)
        mcap_reader.iter_metadata()

        # we need to create one RosParser for each channel/topic
        parser_by_channel_id = {}
        # Storage for DataFrames - one per topic
        topic_dataframes = defaultdict(list)

        # read all the messages inside the mcap file
        for schema, channel, message in mcap_reader.iter_messages(topics=topics_filter):

            # if not parsed already, try to parse the schema
            if channel.id not in parser_by_channel_id:
                schema_data_str = schema.data.decode("utf-8")
                prefix = channel.topic if topic_name_as_prefix else ""
                try:
                    parser_by_channel_id[channel.id] = rosx_introspection.Parser(
                        topic_name=prefix,
                        type_name=schema.name,
                        schema=schema_data_str
                    ) 
                    print(f"Created parser for topic '{channel.topic}' with type '{schema.name}'")
                except Exception as e:
                    print(f"Failed to parse schema ID {schema.id}: {e}")
                    raise Exception("Failed to create parser")

            # get the parser for this channel
            parser = parser_by_channel_id.get(channel.id)

            try:
                # Parse the message data to msgpack
                msgpack_bytes = parser.parse_to_msgpack(message.data)
                # Create row data with timestamp
                row_data = {}

                if schema.name == "capstan/msg/RobotJointCommand":
                    row_data = parse_RobotJointCommand(msgpack_bytes)
                if schema.name == "capstan/msg/RobotJointState":
                    row_data = parse_RobotJointState(msgpack_bytes)
                else:
                    row_data = parse_generic_msgpack(msgpack_bytes)

                # Add to topic's data list
                row_data['_log_timestamp'] = message.log_time
                topic_dataframes[channel.topic].append(row_data)

            except Exception as e:
                print(f"Failed to parse message on channel {channel.topic}: {e}")

        # Convert lists to DataFrames
        final_dataframes = {}
        for topic, data_list in topic_dataframes.items():
            if data_list:  # Only create DataFrame if we have data
                # Create DataFrame with index in one step (more efficient)
                df = pd.DataFrame(data_list).set_index('_log_timestamp')

                # Only sort if needed (check if already sorted)
                if not df.index.is_monotonic_increasing:
                    df.sort_index(inplace=True)

                final_dataframes[topic] = df
        return final_dataframes

def save_dataframes_to_csv(dataframes, output_dir=None):
    """Save DataFrames to CSV files"""
    import os

    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for topic, df in dataframes.items():
        # Clean topic name for filename
        filename = topic.replace('/', '_').lstrip('_') + '.csv'
        if output_dir:
            filepath = os.path.join(output_dir, filename)
        else:
            filepath = filename

        df.to_csv(filepath)
        print(f"Saved {topic} to {filepath}")

def main():
    parser = argparse.ArgumentParser(description='Parse MCAP files and convert to pandas DataFrames')
    parser.add_argument('mcap_file', help='Path to the MCAP file to parse')
    parser.add_argument('--output-dir', '-o', help='Directory to save CSV files (optional)')
    parser.add_argument('--save-csv', action='store_true', help='Save DataFrames to CSV files')

    args = parser.parse_args()

    try:
        # Parse the MCAP file
        dataframes = parse_mcap_file(args.mcap_file, topic_name_as_prefix=False)
        
        # Summary
        print(f"\nSUMMARY:")
        print(f"Total topics: {len(dataframes)}")
        for topic, df in dataframes.items():
            print(f"  {topic}: {len(df)} messages")

        # Save to CSV if requested
        if args.save_csv:
            save_dataframes_to_csv(dataframes, args.output_dir)

        print(f"\nDataFrames are available in the returned dictionary.")
        print(f"Example: dataframes['{list(dataframes.keys())[0] if dataframes else '/topic_name'}']")

        # Return dataframes for interactive use
        return dataframes

    except Exception as e:
        print(f"Error processing MCAP file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
