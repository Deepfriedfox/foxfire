addpath('utilities')

% Main: Process and plot all frames from session
filename = '/Users/justinfox/foxfire/foxfire/octave/testdata/dummy_fastball_session_smooth.bin';  % Update if needed
frames = read_session(filename);

% Load JSON config
config_str = fileread('../radar_config/fox_test/settings.json');  % Read file as string
config = jsondecode(config_str);  % Parse to struct

c = 3e8;  % Speed of light m/s

% Extract params (based on your JSON structure; use parentheses for array indexing)
seq_outer = config.sequence(1);  % First outer sequence
seq_inner = seq_outer.sequence(1);  % Inner loop
num_chirps = seq_inner.num_repetitions;  % 128
chirp = seq_inner.sequence(1);  % Chirp params
num_samples_per_chirp = chirp.num_samples;  % 256
clip_bins = 2;


% Visualization: Heatmap of RD map magnitude (dB scale)
function visualize_rd_map(rd_map)
  figure;
  imagesc(10*log10(abs(rd_map')));
  colorbar;
  title('Range-Doppler Map Magnitude (dB)');
  xlabel('Doppler Bin');
  ylabel('Range Bin');
end

% Animation: "Play" RD maps over frames if multi-frame (for session)
function animate_rd_maps(frames, num_chirps, num_samples_per_chirp, clip_bins)
  figure;
  for f = 1:length(frames)
    raw_data = frames{f}.raw_data;
    chirps = extract_chirps(raw_data, num_chirps, num_samples_per_chirp);
    range_matrix = compute_range_matrix(chirps, num_samples_per_chirp, clip_bins);
    rd_map = compute_range_doppler_map(range_matrix, num_chirps);

    imagesc(10*log10(abs(rd_map')));
    colorbar;
    title(['Range-Doppler Map - Frame ', num2str(frames{f}.frame_id)]);
    xlabel('Doppler Bin');
    ylabel('Range Bin');
    drawnow;  % Update
    pause(0.2);  % Delay for animation
  end
end


% For animation over session
animate_rd_maps(frames, num_chirps, num_samples_per_chirp, clip_bins);

