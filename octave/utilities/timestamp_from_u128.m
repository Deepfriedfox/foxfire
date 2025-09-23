function str = timestamp_from_u128(timestamp_ns)
  % Convert u128 nanoseconds since Unix epoch to date string
  epoch_seconds = timestamp_ns / 1e9;  % Convert ns to seconds
  epoch_datenum = datenum(1970, 1, 1) + (epoch_seconds / 86400);  % Add days to 1970-01-01
  str = datestr(epoch_datenum, 'yyyy-mm-dd HH:MM:SS.FFF');  % Format with ms precision (FFF for millis)
end
