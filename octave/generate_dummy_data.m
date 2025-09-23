% mmWave FMCW radar dummy multi-frame session simulation for 70 mph fastball

% Load JSON config
config_str = fileread('../radar_config/fox_test/settings.json');  % Read file as string
config = jsondecode(config_str);  % Parse to struct

c = 3e8;  % Speed of light m/s

% Extract params (based on your JSON structure; use parentheses for array indexing)
seq_outer = config.sequence(1);  % First outer sequence
seq_inner = seq_outer.sequence(1);  % Inner loop
frame_time = seq_outer.repetition_time_s; % Outer repetition time s (frame rate 10 Hz)
num_chirps = seq_inner.num_repetitions;  % 128
prt = seq_inner.repetition_time_s;  % 2.8e-05 s
chirp = seq_inner.sequence(1);  % Chirp params
num_samples = chirp.num_samples;  % 256
sample_rate = chirp.sample_rate_hz;  % 4000000
bw = chirp.end_frequency_hz - chirp.start_frequency_hz;  % 5e9 Hz
lambda = 3e8 / ((chirp.start_frequency_hz + chirp.end_frequency_hz)/2);  % Wavelength m

disp('Loaded config params:');
disp(['num_chirps: ', num2str(num_chirps)]);
disp(['num_samples_per_chirp: ', num2str(num_samples)]);
disp(['prt: ', num2str(prt)]);
disp(['chirp_bw_hz: ', num2str(bw)]);
disp(['lambda: ', num2str(lambda)]);

num_frames = 50;  % For ~5 seconds simulation

% Slower/smoother: 11 mph (~5 m/s), start 20 m
v = 5;
r_start = 20;
spin_rpm = 2000;
r_ball = 0.0365;
spin_rad_s = spin_rpm * 2*pi / 60;
v_spin = spin_rad_s * r_ball;

% Open file
filename = './testdata/dummy_fastball_session_smooth.bin';
fid = fopen(filename, 'wb');

for frame_idx = 0:num_frames-1
  r_initial = r_start - v * frame_idx * frame_time;
  if r_initial < 0
    r_initial = 0;
  end

  % Simulate samples
  samples = zeros(1, num_chirps * num_samples);
  chirp_time = num_samples / sample_rate;
  for chirp_idx = 0:num_chirps-1
    t_chirp_start = chirp_idx * prt;
    r = r_initial - v * t_chirp_start;
    beat_freq = min((2 * bw * r) / (c * chirp_time), sample_rate / 2 - 1e3);  % Cap with margin

    start_idx = chirp_idx * num_samples + 1;
    t_samples = linspace(0, chirp_time, num_samples);

    chirp_sig = sin(2 * pi * beat_freq * t_samples) * 1000;  % Lower amp

    phase_doppler = 4 * pi * v * t_chirp_start / lambda;
    chirp_sig = chirp_sig .* cos(phase_doppler);

    mod_freq = v_spin / lambda;
    chirp_sig = chirp_sig + sin(2 * pi * mod_freq * t_samples) * 100;

    chirp_sig = chirp_sig + randn(1, num_samples) * 100;  % Lower noise

    samples(start_idx : start_idx + num_samples - 1) = chirp_sig;
  end

  % Clip, pack (same as before)
  samples = max(min(samples, 2047), -2048);
  samples_uint12 = uint16(samples + 2048);

  expected_bytes = (num_chirps * num_samples * 3) / 2;
  raw_data = zeros(1, expected_bytes, 'uint8');
  j = 1;
  for i = 1:2:(num_chirps * num_samples)
    fst = samples_uint12(i);
    snd = samples_uint12(i+1);

    raw_data(j) = bitshift(fst, -4);
    raw_data(j+1) = bitshift(bitand(fst, 15), 4) + bitshift(snd, -8);
    raw_data(j+2) = bitand(snd, 255);

    j = j + 3;
  end

  % Serialize
  frame_id = frame_idx;
  timestamp_ns = uint64((time() + frame_idx * frame_time) * 1e9);
  timestamp_low = bitand(timestamp_ns, 2^64 - 1);
  timestamp_high = bitshift(timestamp_ns, -64);
  raw_len = length(raw_data);

  fwrite(fid, frame_id, 'uint64', 0, 'l');
  fwrite(fid, timestamp_low, 'uint64', 0, 'l');
  fwrite(fid, timestamp_high, 'uint64', 0, 'l');
  fwrite(fid, raw_len, 'uint64', 0, 'l');
  fwrite(fid, raw_data, 'uint8');
end

fclose(fid);
disp(['Wrote improved ', num2str(num_frames), '-frame dummy to ', filename]);
