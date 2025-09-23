% Generate range-Doppler map from range_matrix (extract_doppler_peaks precursor)
function rd_map = compute_range_doppler_map(range_matrix, num_chirps)
  % range_matrix: num_chirps x range_bins complex
  % num_chirps: For FFT size/shift

  num_range_bins = size(range_matrix, 2);
  rd_map = complex(zeros(num_chirps, num_range_bins));  % Doppler x Range

  % Doppler FFT on slow-time (columns)
  for bin = 1:num_range_bins
    doppler_input = range_matrix(:, bin)';  % Row for FFT
    % Mean subtract (MTI)
    mean_val = mean(doppler_input);
    doppler_input = doppler_input - mean_val;

    % Hann window
    window = 0.5 * (1 - cos(2*pi*(0:num_chirps-1)/(num_chirps-1)));
    doppler_input = doppler_input .* window;

    % FFT
    fft_out = fft(doppler_input);

    % FFT shift to center zero-Doppler
    shift = num_chirps / 2;
    fft_shifted = [fft_out(shift+1:end), fft_out(1:shift)];

    rd_map(:, bin) = fft_shifted';
  end
end
