import sys
import io
import msgpack
from mcap.reader import make_reader # install with "pip install mcap"
import rosx_introspection
import pandas as pd
from collections import defaultdict
import argparse
from typing import Iterable

def parse_mcap_file(mcap_file: str, topics_filter: Iterable[str] = None):
    """Parse an MCAP file and return DataFrames for each topic"""

    with open(mcap_file, "rb") as f:
        mcap_reader = make_reader(f)
        mcap_reader.iter_metadata()

        # we need to create one RosParser for each channel/topic
        parser_by_channel_id = {}

        # Storage for DataFrames - one per topic
        topic_dataframes = defaultdict(list)
        message_counts = defaultdict(int)

        # read all the messages inside the mcap file
        for schema, channel, message in mcap_reader.iter_messages(topics=topics_filter):
            parser = None
            # if not parsed already, try to parse the schema
            if channel.id not in parser_by_channel_id:
                schema_data_str = schema.data.decode("utf-8")
                try:
                    prefix = channel.topic
                    parser = rosx_introspection.Parser(
                        topic_name=prefix,
                        type_name=schema.name,
                        schema=schema_data_str
                    )
                    # save this parser to use it later
                    parser_by_channel_id[channel.id] = parser
                    
                except Exception as e:
                    print(f"Failed to parse schema ID {schema.id}: {e}")
                    raise Exception("Failed to create parser")

            # get the parser for this channel
            parser = parser_by_channel_id.get(channel.id)
            if parser:
                try:
                    # Parse the message data to msgpack
                    msgpack_bytes = parser.parse_to_msgpack(message.data)
                    # this is the most efficient way to read msgpack without creating a full dictionary
                    unpacker = msgpack.Unpacker(io.BytesIO(msgpack_bytes), raw=False)
                    # do not create a dictionary, but lets read manually instead
                    flatmap_size = unpacker.read_map_header()

                    # Create row data with timestamp
                    row_data = {
                            '_log_timestamp': message.log_time
                    }
                    # add each key/value pair in the map to the row_data
                    for _i in range(flatmap_size):
                        key = unpacker.unpack()
                        value = unpacker.unpack()
                        row_data[key] = value

                    # Add to topic's data list
                    topic_dataframes[channel.topic].append(row_data)
                    message_counts[channel.topic] += 1

                except Exception as e:
                    print(f"Failed to parse message on channel {channel.topic}: {e}")

        # Convert lists to DataFrames
        final_dataframes = {}
        for topic, data_list in topic_dataframes.items():
            if data_list:  # Only create DataFrame if we have data
                df = pd.DataFrame(data_list)
                # Set log_time as index
                df.set_index('_log_timestamp', inplace=True)
                # Sort by timestamp
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
        dataframes = parse_mcap_file(args.mcap_file)
        
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