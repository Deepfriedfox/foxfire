% Generate range matrix from chirps (compute_range_matrix equivalent)
function range_matrix = compute_range_matrix(chirps, num_samples_per_chirp, clip_bins)
  % chirps: num_chirps x num_samples_per_chirp matrix (real f32)
  % num_samples_per_chirp: Length for FFT (power of 2 ideally)
  % clip_bins: Number of low bins to zero (clutter suppression, e.g., 10)

  num_chirps = size(chirps, 1);
  range_matrix = complex(zeros(num_chirps, num_samples_per_chirp / 2 + 1));  % Positive freqs + DC

  for c = 1:num_chirps
    % FFT on chirp (real input, take positive half)
    fft_out = fft(chirps(c,:));
    fft_out = fft_out(1:num_samples_per_chirp / 2 + 1);  % DC to Nyquist

    % Clip low bins to zero
    fft_out(1:clip_bins + 1) = 0;  % +1 for DC at index 1

    range_matrix(c, :) = fft_out;
  end
end

