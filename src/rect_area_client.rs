use std::time;
// rosmsg_include works properly only with linux line end i.e. LF
// it does now work properly with windows CR LF!!
mod srv {
    rosrust::rosmsg_include!(rosservices / RectArea);
}

fn main() {
    // Fetch args that are not meant for rosrust
    let args: Vec<_> = rosrust::args();

    if args.len() != 3 {
        println!("usage: rect_area_client X Y");
        return;
    }

    let width = args[1].parse::<i64>().unwrap();
    let length = args[2].parse::<i64>().unwrap();

    // Initialize node
    rosrust::init("rect_area_client");

    // Wait ten seconds for the service to appear
    rosrust::ros_info!("checking for availability of rect_area_service...");
    rosrust::wait_for_service("rect_area_service", Some(time::Duration::from_secs(10))).unwrap();
    rosrust::ros_info!("rect_area_service found! calling...");

    // Create client for the service
    let client = rosrust::client::<srv::rosservices::RectArea>("rect_area_service").unwrap();

    // Synchronous call that blocks the thread until a response is received
    rosrust::ros_info!(
        "{} * {} = {}",
        width,
        length,
        client
            .req(&srv::rosservices::RectAreaReq { width, length })
            .unwrap()
            .unwrap()
            .area
    );
}
