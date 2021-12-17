use lazy_static::lazy_static;
/// https://automaticaddison.com/yaw-pitch-and-roll-diagrams-using-2d-coordinate-systems/
/// https://wumbo.net/formula/angle-between-two-vectors-2d/
/// https://stackoverflow.com/questions/42258637/how-to-know-the-angle-between-two-vectors
/// https://mathinsight.org/vector_introduction
/// https://mathinsight.org/vectors_cartesian_coordinates_2d_3d
/// https://wumbo.net/function/arc-tangent-2/
///
use rosrust::Publisher;
use rosrust::{ros_debug, ros_err, ros_info};
use rosrust_msg::geometry_msgs::Twist;
use std::sync::RwLock;
use std::time::SystemTime;

mod msg {
    rosrust::rosmsg_include!(turtlesim / Pose);
}

#[derive(Default, Debug)]
struct TurtlePosition {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

lazy_static! {
    static ref TURTLE_POSITION: RwLock<TurtlePosition> = RwLock::new(TurtlePosition::default());
}

/// helper function to get current turtle position
fn get_current_position() -> TurtlePosition {
    let rl = TURTLE_POSITION.read().unwrap();
    let turtle_position = &*rl;

    TurtlePosition {
        x: turtle_position.x,
        y: turtle_position.y,
        yaw: turtle_position.yaw,
    }
}

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
            Ok(elapsed) => {
                current_distance = speed * elapsed.as_secs() as f64;
                ros_debug!("moved: {}/{}", current_distance, distance);
            }
            Err(e) => {
                ros_err!("move_forward elapsed time error: {:?}", e);
            }
        }
        loop_rate.sleep();
    }
    velocity_msg.linear.x = 0.0;
    velocity_publisher.send(velocity_msg).unwrap();
}

/// rotates by given angular speed until rotation by given
/// angle (rotation_degree) in radians in CW or CCW direction is achieved
fn rotate(
    velocity_publisher: Publisher<Twist>,
    angular_speed: f64,
    rotation_degree: f64,
    clockwise: bool,
) {
    let mut velocity_msg = Twist::default();
    if clockwise {
        velocity_msg.angular.z = -angular_speed.abs();
    } else {
        velocity_msg.angular.z = angular_speed.abs();
    }

    let t0 = SystemTime::now();
    let loop_rate = rosrust::rate(10.0); // 10 MHz loop rate

    let mut current_angle = 0.0;
    loop {
        velocity_publisher.send(velocity_msg.clone()).unwrap();

        let time_elapsed = t0.elapsed().unwrap().as_secs() as f64;

        if current_angle > rotation_degree {
            break;
        }

        current_angle = angular_speed * time_elapsed;

        ros_debug!("rotated: {}/{}", current_angle, rotation_degree);
        ros_debug!(
            "rotated: {}/{}. Current pos: {:?}",
            current_angle,
            rotation_degree,
            get_current_position()
        );

        loop_rate.sleep();
    }

    velocity_msg.angular.z = 0.0;
    velocity_publisher.send(velocity_msg).unwrap();
}

/// moves from current position to target position
/// linear and angular speed is proportionally controlled
/// by k_linear and k_angular constants. To closer to target
/// the slower we are moving.
fn go_to_target(
    velocity_publisher: Publisher<Twist>,
    target_x: f64,
    target_y: f64,
    k_linear: f64,
    k_angular: f64,
) {
    let mut velocity_msg = Twist::default();

    let loop_rate = rosrust::rate(10.0); // 10 MHz loop rate

    loop {
        let turtle_position = get_current_position();

        ros_debug!("turtle_position {:?}", turtle_position);

        let target_distance =
            calculate_distance_2d(turtle_position.x, turtle_position.y, target_x, target_y);

        let linear_speed = k_linear * target_distance;
        let desired_angle = (target_y - turtle_position.y).atan2(target_x - turtle_position.x);
        let angular_speed = k_angular * (desired_angle - turtle_position.yaw);

        velocity_msg.linear.x = linear_speed;
        velocity_msg.angular.z = angular_speed;

        velocity_publisher.send(velocity_msg.clone()).unwrap();

        if target_distance < 0.01 {
            break;
        }

        loop_rate.sleep();
    }

    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = 0.0;
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

fn rotate_caller(args: Vec<String>, velocity_publisher: Publisher<Twist>) {
    let angular_speed = args[2].parse::<f64>().unwrap();

    // for convenience specified in degrees. function internally recalculates to radians
    let rotation_degree = args[3].parse::<f64>().unwrap();
    let clockwise = args[4].parse::<bool>().unwrap();
    ros_info!(
        "calling rotate. angular_speed: {} rotation_degree: {} clockwise: {}",
        angular_speed,
        rotation_degree,
        clockwise
    );
    rotate(
        velocity_publisher,
        angular_speed,
        rotation_degree.to_radians(),
        clockwise,
    );
}

fn go_to_target_caller(args: Vec<String>, velocity_publisher: Publisher<Twist>) {
    let target_x = args[2].parse::<f64>().unwrap();
    let target_y = args[3].parse::<f64>().unwrap();

    let mut k_linear = 0.5;
    let mut k_angular = 4.0;

    if args.len() >= 5 {
        k_linear = args[4].parse::<f64>().unwrap_or(0.5);
    }

    if args.len() >= 6 {
        k_angular = args[5].parse::<f64>().unwrap_or(4.0);
    }

    ros_info!(
        "calling go_to_target. target_x: {} target_y: {} k_linear: {} k_angular: {}",
        target_x,
        target_y,
        k_linear,
        k_angular
    );

    go_to_target(velocity_publisher, target_x, target_y, k_linear, k_angular);
}

fn main() {
    rosrust::init("turtle_cleaner");

    let velocity_publisher =
        rosrust::publish::<rosrust_msg::geometry_msgs::Twist>("/turtle1/cmd_vel", 100).unwrap();

    let _raii_subscriber =
        rosrust::subscribe("/turtle1/pose", 100, move |pose: msg::turtlesim::Pose| {
            let mut wl = TURTLE_POSITION.write().unwrap();
            let turtle_position = &mut *wl;
            turtle_position.x = pose.x as f64;
            turtle_position.y = pose.y as f64;
            turtle_position.yaw = pose.theta as f64;
        })
        .unwrap();

    let args = rosrust::args();

    ros_info!("turtle_cleaner initialized");

    let switch_value = args[1].parse::<i16>().unwrap();
    match switch_value {
        1 => move_forward_caller(args, velocity_publisher),
        2 => rotate_caller(args, velocity_publisher),
        3 => go_to_target_caller(args, velocity_publisher),
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

    #[test]
    fn test_atan2() {
        // https://en.wikipedia.org/wiki/Atan2
        // https://wumbo.net/function/arc-tangent-2/

        let x = 1f64;
        let y = 1f64;
        assert_eq!(y.atan2(x).to_degrees(), 45_f64);

        let x = 1f64;
        let y = -1f64;
        assert_eq!(y.atan2(x).to_degrees(), -45_f64);

        let x = -1f64;
        let y = -1f64;
        assert_eq!(y.atan2(x).to_degrees(), -135_f64);

        let x = -1f64;
        let y = 1f64;
        assert_eq!(y.atan2(x).to_degrees(), 135_f64);
    }
}
