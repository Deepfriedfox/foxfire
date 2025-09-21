function chirps = extract_chirps(frame, num_chirps, num_samples_per_chirp)
  % Unpack 12-bit packed data from uint8 frame
  total_samples = num_chirps * num_samples_per_chirp;
  expected_bytes = (total_samples * 3) / 2;  % Exact for even total
  if length(frame) < expected_bytes
    error('Frame too short: expected %d bytes, got %d', expected_bytes, length(frame));
  end

  % Reshape to rows of 3 bytes
  data = reshape(frame, 3, expected_bytes / 3)';

  fst_uint8 = data(:,1);
  mid_uint8 = data(:,2);
  lst_uint8 = data(:,3);

  % Unpack to uint12
  fst_uint12 = bitshift(fst_uint8, 4) + bitshift(mid_uint8, -4);
  snd_uint12 = bitshift(bitand(mid_uint8, 15), 8) + lst_uint8;

  % Flatten and center to signed f32
  samples = [fst_uint12; snd_uint12];
  samples = reshape(samples', 1, []) - 2048;  % Row vector, centered

  % Split into chirps (num_chirps x num_samples_per_chirp)
  chirps = reshape(samples, num_samples_per_chirp, num_chirps)';
endfunction
