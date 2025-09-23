function frames = read_session(filename)
  % Read multiple RawFrames from appended file
  fid = fopen(filename, 'rb');
  if fid == -1
    error('Failed to open file: %s', filename);
  end

  frames = {};
  while ~feof(fid)
    try
      disp('reading frames')
      [raw_data, frame_id, timestamp_ns] = read_raw_frame(fid);
      frames{end+1} = struct('raw_data', raw_data, 'frame_id', frame_id, 'timestamp_ns', timestamp_ns);
    catch err
      disp(['ran into error: ', err.message])
      break;  % EOF or error
    end
  end

  fclose(fid);
  disp(['Loaded ', num2str(length(frames)), ' frames']);
end


