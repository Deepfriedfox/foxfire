addpath('utilities')

% Main: Process and plot all frames from session
filename = '/Users/justinfox/foxfire/foxfire/octave/testdata/dummy_fastball_session.bin';  % Update if needed
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

figure;  % New figure for plots
for f = 1:length(frames)
  raw_data = frames{f}.raw_data;
  chirps = extract_chirps(raw_data, num_chirps, num_samples_per_chirp);

  subplot(length(frames), 1, f);  % Subplot for each frame
  plot(chirps(1,:));  % Plot first chirp of frame
  title(['Frame ', num2str(frames{f}.frame_id), ' First Chirp']);
  xlabel('Sample Index');
  ylabel('Sample Value');
end
