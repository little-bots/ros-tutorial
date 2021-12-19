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

#[cfg(not(feature = "strict-pos-sync"))]
use std::sync::RwLock;

#[cfg(feature = "strict-pos-sync")]
use std::sync::Mutex;

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

#[cfg(not(feature = "strict-pos-sync"))]
lazy_static! {
    static ref TURTLE_POSITION: RwLock<TurtlePosition> = RwLock::new(TurtlePosition::default());
}

#[cfg(feature = "strict-pos-sync")]
lazy_static! {
    static ref TURTLE_POSITION: Mutex<TurtlePosition> = Mutex::new(TurtlePosition::default());
}

/// helper function to get current turtle position
#[cfg(not(feature = "strict-pos-sync"))]
fn get_current_position() -> TurtlePosition {
    let rl = TURTLE_POSITION.read().unwrap();
    let turtle_position = &*rl;

    TurtlePosition {
        x: turtle_position.x,
        y: turtle_position.y,
        yaw: turtle_position.yaw,
    }
}

/// helper function to set current turtle position
#[cfg(not(feature = "strict-pos-sync"))]
fn set_current_position(new_turtle_position: &TurtlePosition) {
    let mut wl = TURTLE_POSITION.write().unwrap();
    let turtle_position = &mut *wl;
    turtle_position.x = new_turtle_position.x;
    turtle_position.y = new_turtle_position.y;
    turtle_position.yaw = new_turtle_position.yaw;
}

/// helper function to get current turtle position
#[cfg(feature = "strict-pos-sync")]
fn get_current_position() -> TurtlePosition {
    let rl = TURTLE_POSITION.lock().unwrap();
    let turtle_position = &*rl;

    TurtlePosition {
        x: turtle_position.x,
        y: turtle_position.y,
        yaw: turtle_position.yaw,
    }
}

/// helper function to set current turtle position
#[cfg(feature = "strict-pos-sync")]
fn set_current_position(new_turtle_position: &TurtlePosition) {
    let mut wl = TURTLE_POSITION.lock().unwrap();
    let turtle_position = &mut *wl;
    turtle_position.x = new_turtle_position.x;
    turtle_position.y = new_turtle_position.y;
    turtle_position.yaw = new_turtle_position.yaw;
}

/// calculates euclidean distance of two points in 2D ( [x0,y0] and [x1, y1] )
fn calculate_distance_2d(x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt()
}

/// Given two points in 2D ( [x_current, y_current] and [x_target, y_target] )
/// calculates angle between vectors
/// [x_current, y_current] -> [x_current2, y_current] where x_current2 > x_current1
///
/// [x_current, y_current] -> [x_target, y_target]
///
/// Example
///     y-axis
///     ^
///   3 |          *--------> (vec from [3,3] with yaw = 0)
///     |       .
///   1 |   *
///     |
///     -- -- -- --> x-axis
///         1     3
///
/// To get from point [3, 3] to point [1, 1] assuming robot yaw in [3, 3] is 0 (i.e. heading/facing east)
/// robot needs to rotate -135 degrees (-90 degrees to face south + additional -45 degrees to face towards [1, 1])
///
fn angle_to_target_2d(x_current: f64, y_current: f64, x_target: f64, y_target: f64) -> f64 {
    (y_target - y_current).atan2(x_target - x_current)
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
                current_distance = speed * elapsed.as_secs_f64() as f64;
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
/// angle (rotation_rad) in radians in CW or CCW direction is achieved
fn rotate(
    velocity_publisher: Publisher<Twist>,
    angular_speed: f64,
    rotation_rad: f64,
    clockwise: bool,
) {
    let mut velocity_msg = Twist::default();
    if clockwise {
        velocity_msg.angular.z = -angular_speed.abs();
    } else {
        velocity_msg.angular.z = angular_speed.abs();
    }

    let t0 = SystemTime::now();
    let loop_rate = rosrust::rate(10.0); // 10 Hz loop rate

    let mut current_angle = 0.0;
    loop {
        velocity_publisher.send(velocity_msg.clone()).unwrap();

        let time_elapsed = t0.elapsed().unwrap().as_secs_f64() as f64;

        if current_angle > rotation_rad {
            break;
        }

        current_angle = angular_speed * time_elapsed;

        ros_debug!("rotated: {}/{}. time elapsed: {}", current_angle, rotation_rad, time_elapsed);
        ros_debug!(
            "rotated: {}/{}. Current pos: {:?}",
            current_angle,
            rotation_rad,
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
        let angle_to_target =
            angle_to_target_2d(turtle_position.x, turtle_position.y, target_x, target_y);
        let angular_speed = k_angular * (angle_to_target - turtle_position.yaw);

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

/// sets new yaw. uses rotate to change the robot position accordingly.
fn set_yaw(velocity_publisher: Publisher<Twist>, angular_speed: f64, new_yaw: f64) {
    let turtle_position = get_current_position();

    let angle_to_rotate = new_yaw - turtle_position.yaw;
    if angle_to_rotate == 0.0 {
        return;
    }

    let clockwise = if angle_to_rotate < 0.0 { true } else { false };
    rotate(
        velocity_publisher,
        angular_speed,
        angle_to_rotate.abs(),
        clockwise,
    );
}

/// moves the robot from current location in spiral clockwise move. this is achieved
/// by maintaining same angular velocity and gradual increase of initial linear velocity
fn spiral_move(velocity_publisher: Publisher<Twist>, linear_speed_init: f64, angular_speed: f64) {
    let mut velocity_msg = Twist::default();

    let loop_rate = rosrust::rate(1.0);

    let mut linear_speed = linear_speed_init;
    loop {
        let turtle_position = get_current_position();
        if turtle_position.x > 10.5 || turtle_position.y > 10.5 {
            break;
        }
        linear_speed = linear_speed + 1.0;
        velocity_msg.linear.x = linear_speed;
        velocity_msg.angular.z = angular_speed;

        velocity_publisher.send(velocity_msg.clone()).unwrap();
        loop_rate.sleep();
    }

    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = 0.0;
    velocity_publisher.send(velocity_msg).unwrap();
}

///
/// caller functions below. called form main. after parsing command line args
/// they will call core movement functions.
///
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

fn  rotate_caller(args: Vec<String>, velocity_publisher: Publisher<Twist>) {
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

fn set_yaw_caller(args: Vec<String>, velocity_publisher: Publisher<Twist>) {
    let angular_speed = args[2].parse::<f64>().unwrap();
    let new_yaw = args[3].parse::<f64>().unwrap();

    ros_info!(
        "calling set_yaw. angular_speed: {} new_yaw: {}",
        angular_speed,
        new_yaw,
    );

    // yaw angle is specified in degrees for convenience
    set_yaw(velocity_publisher, angular_speed, new_yaw.to_radians());
}

fn spiral_move_caller(args: Vec<String>, velocity_publisher: Publisher<Twist>) {
    let linear_speed_init = args[2].parse::<f64>().unwrap();
    let angular_speed = args[3].parse::<f64>().unwrap();

    ros_info!(
        "calling spiral_move. linear_speed_init: {} angular_speed: {}",
        linear_speed_init,
        angular_speed,
    );

    spiral_move(velocity_publisher, linear_speed_init, angular_speed);
}

fn grid_clean() {
    go_to_target(get_publisher(), 1.0, 1.0, 0.5, 4.0);

    /* for i in (2..5).step_by(1) {
        go_to_target(get_publisher(), i as f64, 1.0, 0.5, 4.0);
        go_to_target(get_publisher(), i as f64, 10.0, 0.5, 4.0);
        go_to_target(get_publisher(), i as f64 + 1.0, 10.0, 0.5, 4.0);
        go_to_target(get_publisher(), i as f64 + 1.0, 1.0, 0.5, 4.0);
    } */

    let angle90 = 90.0_f64.to_radians();
    let angular_speed = 0.25; // rotate slowly sot hat it is precise
    let linear_speed = 2.0;

    // face to east
    set_yaw(get_publisher(), angular_speed, 0.0);

    for _ in 1..5 {
        move_forward(get_publisher(), linear_speed, 1., true);
        rotate(get_publisher(), angular_speed, angle90, false);
        move_forward(get_publisher(), linear_speed, 9., true);
        rotate(get_publisher(), angular_speed, angle90, true);
        move_forward(get_publisher(), linear_speed, 1., true);
        rotate(get_publisher(), angular_speed, angle90, true);
        move_forward(get_publisher(), linear_speed, 9., true);
        rotate(get_publisher(), angular_speed, angle90, false);
    }
}

fn spiral_clean() {
    spiral_move(get_publisher(), 0.0, 2.0);
}

// quick & dirty. instead we should create publisher once and pass mutable reference.
fn get_publisher() -> Publisher<Twist> {
    rosrust::publish::<rosrust_msg::geometry_msgs::Twist>("/turtle1/cmd_vel", 100).unwrap()
}

fn main() {
    if cfg!(feature = "strict-pos-sync") {
        println!("strict-pos-sync enabled");
    } else {
        println!("strict-pos-sync disabled");
    }

    rosrust::init("turtle_cleaner");

    let velocity_publisher = get_publisher();

    let _raii_subscriber =
        rosrust::subscribe("/turtle1/pose", 100, move |pose: msg::turtlesim::Pose| {
            set_current_position(&TurtlePosition {
                x: pose.x as f64,
                y: pose.y as f64,
                yaw: pose.theta as f64,
            });
        })
        .unwrap();

    let args = rosrust::args();

    ros_info!("turtle_cleaner initialized");

    let switch_value = args[1].parse::<i16>().unwrap();
    match switch_value {
        1 => move_forward_caller(args, velocity_publisher),
        2 => rotate_caller(args, velocity_publisher),
        3 => go_to_target_caller(args, velocity_publisher),
        4 => set_yaw_caller(args, velocity_publisher),
        5 => spiral_move_caller(args, velocity_publisher),
        6 => grid_clean(),
        7 => spiral_clean(),
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

    #[test]
    fn test_angle_to_target_2d() {
        assert_eq!(
            angle_to_target_2d(3.0, 3.0, 1.0, 1.0).to_degrees(),
            -135_f64
        );
        assert_eq!(angle_to_target_2d(3.0, 3.0, 3.0, 1.0).to_degrees(), -90_f64);
        assert_eq!(angle_to_target_2d(3.0, 3.0, 4.0, 4.0).to_degrees(), 45_f64);
        assert_eq!(angle_to_target_2d(3.0, 3.0, 3.0, 4.0).to_degrees(), 90_f64);
        assert_eq!(angle_to_target_2d(3.0, 3.0, 2.0, 4.0).to_degrees(), 135_f64);
    }
}
