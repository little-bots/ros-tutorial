use rosrust_msg::geometry_msgs::Twist;
use std::cell::RefCell;

mod msg {
    rosrust::rosmsg_include!(turtlesim / Pose);
}

fn main() {
    rosrust::init("turtle");

    let ros_publisher = rosrust::publish::<msg::turtlesim::Pose>("/turtle1/pose", 100).unwrap();

    let turtle_position = RefCell::new(msg::turtlesim::Pose::default());

    rosrust::ros_info!("Turtle initiated. Initial position: {:#?}", turtle_position);

    let _subscriber = rosrust::subscribe("/turtle1/cmd_vel", 100, move |t: Twist| {
        rosrust::ros_info!("Received command: {:?}", t);

        // quick and dirty, will panic if two callbacks will try to access turtle_position in parallel
        // proper implementation would use Mutex/RwLock
        let mut position = turtle_position.borrow_mut();

        rosrust::ros_info!("current position: {:?}", position);

        // dummy: no real calculation here! we are not
        // really maintaining turtle position here
        // that would require some math ;)
        position.x = position.x + t.linear.x as f32;
        position.y = position.y + t.linear.y as f32;

        rosrust::ros_info!("new position: {:?}", position);
        ros_publisher.send(position.clone()).unwrap();
    })
    .unwrap();

    rosrust::spin();
}
