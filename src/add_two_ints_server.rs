mod srv {
    rosrust::rosmsg_include!(rosservices / AddTwoInts);
}

fn main() {
    // Initialize node
    rosrust::init("add_two_ints_server");

    // Create service
    // The service is stopped when the returned object is destroyed
    let _service_raii =
        rosrust::service::<srv::rosservices::AddTwoInts, _>("add_two_ints", move |req| {
            // Callback for handling requests
            let sum = req.a + req.b;

            // Log each request
            rosrust::ros_info!("{} + {} = {}", req.a, req.b, sum);

            Ok(srv::rosservices::AddTwoIntsRes { sum })
        })
        .unwrap();

    rosrust::ros_info!("add_two_ints service running!");

    // Block the thread until a shutdown signal is received
    rosrust::spin();
}
