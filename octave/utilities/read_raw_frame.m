function [raw_data, frame_id, timestamp_ns] = read_raw_frame(fid)
  % Read bincode serialized RawFrame from open fid (little-endian)
  % Note: Does not close fid; caller handles

  % u64 frame_id
  [val, count] = fread(fid, 1, 'uint64', 0, 'l');
  if count < 1
    error('Failed to read frame_id (EOF or corrupt file)');
  end
  frame_id = val;

  % u128 timestamp_ns (low then high u64)
  [low, count] = fread(fid, 1, 'uint64', 0, 'l');
  if count < 1
    error('Failed to read timestamp_low');
  end
  [high, count] = fread(fid, 1, 'uint64', 0, 'l');
  if count < 1
    error('Failed to read timestamp_high');
  end
  timestamp_ns = bitshift(high, 64) + low;

  % u64 raw_len
  [raw_len, count] = fread(fid, 1, 'uint64', 0, 'l');
  if count < 1
    error('Failed to read raw_len');
  end

  % raw_data as uint8 vector
  [raw_data, count] = fread(fid, raw_len, 'uint8');
  if count < raw_len
    error('Failed to read full raw_data (expected %d bytes, got %d)', raw_len, count);
  end

  disp(['Loaded frame_id: ', num2str(frame_id)]);
  disp(['Timestamp: ', timestamp_from_u128(timestamp_ns)]);
  disp(['Raw data length: ', num2str(length(raw_data))]);
end

