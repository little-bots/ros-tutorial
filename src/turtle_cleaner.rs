/// https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
/// https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/issues/index.htm
/// https://automaticaddison.com/yaw-pitch-and-roll-diagrams-using-2d-coordinate-systems/
use rosrust::Publisher;
use rosrust::{ros_err, ros_info};
use rosrust_msg::geometry_msgs::Twist;
use std::time::SystemTime;

mod msg {
    rosrust::rosmsg_include!(turtlesim / Pose);
}

#[allow(dead_code)]
fn calculate_distance_2d(x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt()
}

/// moves forward (by publishing Twist messages with given linear speed)
/// until defined distance is travelled by defined speed
/// then linear speed is set to zero to stop.
fn move_forward(
    velocity_publisher: Publisher<Twist>,
    speed: f64,
    distance: f64,
    move_forward: bool,
) {
    let mut velocity_msg = Twist::default();
    if move_forward {
        velocity_msg.linear.x = speed.abs();
    } else {
        velocity_msg.linear.x = -speed.abs();
    }

    let t0 = SystemTime::now();
    let loop_rate = rosrust::rate(10.0); // 10 MHz loop rate
    let mut current_distance = 0.0;
    while current_distance < distance {
        velocity_publisher.send(velocity_msg.clone()).unwrap();

        match t0.elapsed() {
            Ok(elapsed) => current_distance = speed * elapsed.as_secs() as f64,
            Err(e) => {
                ros_err!("move_forward elapsed time error: {:?}", e);
            }
        }
        loop_rate.sleep();
    }
    velocity_msg.linear.x = 0.0;
    velocity_publisher.send(velocity_msg).unwrap();
}

fn move_forward_caller(args: Vec<String>, velocity_publisher: Publisher<Twist>) {
    let speed = args[2].parse::<f64>().unwrap();
    let distance = args[3].parse::<f64>().unwrap();
    let forward_flg = args[4].parse::<bool>().unwrap();
    ros_info!(
        "calling move_forward. speed: {} distance: {} forward_flg: {}",
        speed,
        distance,
        forward_flg
    );
    move_forward(velocity_publisher, speed, distance, forward_flg);
}

fn main() {
    rosrust::init("turtle_cleaner");

    let velocity_publisher =
        rosrust::publish::<rosrust_msg::geometry_msgs::Twist>("/turtle1/cmd_vel", 100).unwrap();

    let args = rosrust::args();

    ros_info!("turtle_cleaner initialized");

    let switch_value = args[1].parse::<i16>().unwrap();
    match switch_value {
        1 => move_forward_caller(args, velocity_publisher),
        _ => {
            ros_err!("unsupported action specified {}", switch_value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_distance_2d() {
        assert_eq!(calculate_distance_2d(0.0, 0.0, 3.0, 3.0), 18.0_f64.sqrt());
        assert_eq!(calculate_distance_2d(0.0, 0.0, 5.0, 1.0), 26.0_f64.sqrt());
        assert_eq!(calculate_distance_2d(0.0, 0.0, -5.0, -1.0), 26.0_f64.sqrt());
    }
}
