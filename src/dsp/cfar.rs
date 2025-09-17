use rand::{Rng};
use rand_distr::{Normal, Distribution, Uniform, Poisson};

pub fn cfar(magnitudes: &[f32]) -> Vec<usize> {
    if magnitudes.len() < 5 {
        return vec![];
    }
    let n = magnitudes.len();
    let guard_cells = 2;
    let ref_cells = 8;
    let pfa_multiplier = 3.0;

    (guard_cells..n.saturating_sub(guard_cells))
        .filter_map(|i| {
            let start = i.saturating_sub(guard_cells + ref_cells);
            let end = (i + guard_cells + ref_cells).min(n);
            if end <= start { return None; }
            let clutter_sum: f32 = magnitudes[start..end].iter().sum();
            let clutter_avg = clutter_sum / (end - start) as f32;
            let threshold = pfa_multiplier * clutter_avg;

            if magnitudes[i] > threshold {
                Some(i)
            } else {
                None
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cfar_detects_peak() {
        let magnitudes = vec![1.0, 2.0, 1.0, 10.0, 1.0, 2.0, 1.0];
        let detections = cfar(&magnitudes);
        assert_eq!(detections, vec![3]);
    }

    #[test]
    fn test_cfar_no_false_alarm() {
        let magnitudes = vec![1.0; 20];
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_cfar_small_array() {
        let magnitudes = vec![1.0; 4];
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_cfar_empty() {
        let magnitudes = vec![];
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_cfar_edge_peaks() {
        let mut magnitudes = vec![1.0; 20];
        magnitudes[0] = 10.0;
        magnitudes[19] = 10.0;
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Edges guarded");
    }

    #[test]
    fn test_cfar_overflow_prevention() {
        let magnitudes = vec![1.0; 10];
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "No overflow in small n");
    }

    #[test]
    fn test_cfar_high_noise_peak() {
        let mut magnitudes = vec![5.0; 20];
        magnitudes[10] = 20.0;
        let detections = cfar(&magnitudes);
        assert_eq!(detections, vec![10]);
    }

    #[test]
    fn test_cfar_low_contrast() {
        let mut magnitudes = vec![1.0; 20];
        magnitudes[10] = 2.0;
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Low contrast filtered");
    }

    #[test]
    fn test_cfar_diagnostic() {
        let mut magnitudes = vec![1.0; 20];
        magnitudes[10] = 3.0;
        let detections = cfar(&magnitudes);
        if !detections.is_empty() {
            println!("Diagnostic: Unexpected detection at {}", detections[0]);
        }
        assert!(detections.is_empty(), "Diagnostic low contrast");
    }

    #[test]
    fn test_cfar_variable_ref() {
        let mut magnitudes = vec![1.0; 50];
        magnitudes[25] = 10.0;
        let detections = cfar(&magnitudes);
        assert_eq!(detections, vec![25]);
    }

    #[test]
    fn test_cfar_uneven_noise() {
        let mut magnitudes = vec![1.0; 10];
        magnitudes.extend(vec![5.0; 10]);
        magnitudes[15] = 20.0;
        let detections = cfar(&magnitudes);
        assert_eq!(detections, vec![15], "Detect in high noise area");
    }

    #[test]
    fn test_cfar_gaussian_noise_no_detection() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 1.0).unwrap();
        let magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 5.0).collect();  // Mean 5, std 1
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Gaussian noise filtered");
    }

    #[test]
    fn test_cfar_gaussian_noise_with_peak() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 1.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 5.0).collect();
        magnitudes[50] = 20.0;  // Clear peak
        let detections = cfar(&magnitudes);
        assert_eq!(detections, vec![50], "Peak detected in Gaussian noise");
    }

    #[test]
    fn test_cfar_varying_noise_no_detection() {
        let mut magnitudes = vec![];
        for i in 0..50 {
            magnitudes.push((i as f32 / 10.0) + 1.0);  // Increasing noise
        }
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Varying noise filtered");
    }

    #[test]
    fn test_cfar_noise_with_small_peaks() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 1.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 5.0).collect();
        magnitudes[20] = 8.0;  // Small
        magnitudes[80] = 9.0;  // Small
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Small peaks in noise filtered");
    }

    #[test]
    fn test_cfar_random_noise_diagnostic() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 1.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 5.0).collect();
        magnitudes[50] = 6.0;  // Borderline
        let detections = cfar(&magnitudes);
        if !detections.is_empty() {
            println!("Diagnostic: Unexpected detection in random noise at {}", detections[0]);
        }
        assert!(detections.is_empty(), "Random noise filtered");
    }

    #[test]
    fn test_cfar_uniform_noise_no_detection() {
        let mut rng = rand::rng();
        let uniform = Uniform::new(0.0, 10.0).unwrap();
        let magnitudes: Vec<f32> = (0..100).map(|_| rng.sample(&uniform)).collect();
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Uniform noise filtered");
    }

    #[test]
    fn test_cfar_uniform_noise_with_peak() {
        let mut rng = rand::rng();
        let uniform = Uniform::new(0.0, 10.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| rng.sample(&uniform)).collect();
        magnitudes[50] = 50.0;  // High peak
        let detections = cfar(&magnitudes);
        assert!(!detections.is_empty(), "Peak in uniform noise detected");
    }

    #[test]
    fn test_cfar_poisson_noise_no_detection() {
        let mut rng = rand::rng();
        let poisson = Poisson::new(5.0).unwrap();
        let magnitudes: Vec<f32> = (0..100).map(|_| poisson.sample(&mut rng) as f32).collect();
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Poisson noise filtered");
    }

    #[test]
    fn test_cfar_poisson_noise_with_peak() {
        let mut rng = rand::rng();
        let poisson = Poisson::new(5.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| poisson.sample(&mut rng) as f32).collect();
        magnitudes[50] = 50.0;  // Peak
        let detections = cfar(&magnitudes);
        assert!(!detections.is_empty(), "Peak in Poisson noise detected");
    }

    #[test]
    fn test_cfar_high_variance_noise_no_detection() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 5.0).unwrap();
        let magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 10.0).collect();
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "High variance noise filtered");
    }

    #[test]
    fn test_cfar_high_variance_noise_with_peak() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 5.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 10.0).collect();
        magnitudes[50] = 50.0;  // Peak
        let detections = cfar(&magnitudes);
        assert!(!detections.is_empty(), "Peak in high variance noise detected");
    }

    #[test]
    fn test_cfar_noise_with_multiple_small_peaks() {
        let mut rng = rand::rng();
        let normal = Normal::new(0.0, 1.0).unwrap();
        let mut magnitudes: Vec<f32> = (0..100).map(|_| normal.sample(&mut rng) + 5.0).collect();
        magnitudes[20] = 7.0;  // Small
        magnitudes[50] = 8.0;  // Small
        magnitudes[80] = 9.0;  // Small
        let detections = cfar(&magnitudes);
        assert!(detections.is_empty(), "Multiple small peaks filtered");
    }
}