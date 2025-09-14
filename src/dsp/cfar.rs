pub fn run_cfar(input: &[f32]) -> Vec<usize> {
    let mut detections = Vec::new();
    let guard = 2;
    let training = 4;
    let threshold_scale = 3.0;

    for i in training + guard..input.len() - training - guard {
        let noise_level: f32 = input[i - training - guard..i - guard]
            .iter()
            .chain(input[i + guard + 1..i + guard + 1 + training].iter())
            .copied()
            .sum::<f32>()
            / (2 * training) as f32;

        if input[i] > noise_level * threshold_scale {
            detections.push(i);
        }
    }
    detections
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cfar_detects_peak() {
        let mut signal = vec![1.0; 50];
        signal[25] = 20.0; // Inject strong peak

        let detections = run_cfar(&signal);

        assert!(detections.contains(&25));
    }

    #[test]
    fn test_cfar_no_false_alarm() {
        let signal = vec![1.0; 50]; // flat noise floor
        let detections = run_cfar(&signal);

        assert!(detections.is_empty());
    }
}
