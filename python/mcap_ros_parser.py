import sys
import io
import msgpack
from mcap.reader import make_reader # install with "pip install mcap"
import rosx_introspection
import pandas as pd
from collections import defaultdict

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <mcap_file>")
    sys.exit(1)

mcap_file = sys.argv[1]

with open(mcap_file, "rb") as f:
    mcap_reader = make_reader(f)
    mcap_reader.iter_metadata()
    
    # we need to create one RosParser for each channel/topic
    parser_by_channel_id = {}

    # Storage for DataFrames - one per topic
    topic_dataframes = defaultdict(list)
    message_counts = defaultdict(int)
    
    # read all the messages inside the mcap file
    for schema, channel, message in mcap_reader.iter_messages():
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
                print(f"Created parser for channel {channel.topic} (schema: {schema.name})")

            except Exception as e:
                print(f"Failed to parse schema ID {schema.id}: {e}")
                print(f"Schema data:\n{schema_data_str}")
                raise Exception("Failed to create parser")

        # get the parser for this channel
        parser = parser_by_channel_id.get(channel.id)
        if parser:
            try:
                msgpack_bytes = parser.parse_to_msgpack(message.data)
                # Parse the message data to msgpack
                # this is the most efficient way to read msgpack without creating a full dictionary
                unpacker = msgpack.Unpacker(io.BytesIO(msgpack_bytes), raw=False)
                # do not create a dictionary, but lets read manually instead
                flatmap_size = unpacker.read_map_header()
                
                # Create row data with timestamp
                row_data = {
                        '_log_timestamp': message.log_time
                }

                for _i in range(flatmap_size):
                    key = unpacker.unpack()
                    value = unpacker.unpack()  # read the value (could be any type)
                    row_data[key] = value

                # Add to topic's data list
                topic_dataframes[channel.topic].append(row_data)
                message_counts[channel.topic] += 1

            except Exception as e:
                print(f"Failed to parse message on channel {channel.topic}: {e}")

    # Convert lists to DataFrames
    final_dataframes = {}

    print("\nCreating DataFrames...")

    for topic, data_list in topic_dataframes.items():
        if data_list:  # Only create DataFrame if we have data
            df = pd.DataFrame(data_list)
            # Set log_time as index
            df.set_index('_log_timestamp', inplace=True)
            # Sort by timestamp
            df.sort_index(inplace=True)

            final_dataframes[topic] = df
            print(f"Created DataFrame for {topic}: {len(df)} messages, {len(df.columns)} fields")

    # Summary
    print(f"\nSUMMARY:")
    print(f"Total topics: {len(final_dataframes)}")
    for topic, df in final_dataframes.items():
        print(f"  {topic}: {len(df)} messages")

    # for debugging, you can save each dataframe to a CSV file
    for topic, df in final_dataframes.items():
        df.to_csv(f"{topic.replace('/', '_')}.csv")