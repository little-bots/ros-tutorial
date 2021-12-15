use std::time;

mod srv {
    rosrust::rosmsg_include!(rosservices / AddTwoInts);
}

fn main() {
    // Fetch args that are not meant for rosrust
    let args: Vec<_> = rosrust::args();

    if args.len() != 3 {
        println!("usage: client X Y");
        return;
    }

    let a = args[1].parse::<i64>().unwrap();
    let b = args[2].parse::<i64>().unwrap();

    // Initialize node
    rosrust::init("add_two_ints_client");

    // Wait ten seconds for the service to appear
    rosrust::ros_info!("checking for availability of add_two_ints service...");
    rosrust::wait_for_service("add_two_ints", Some(time::Duration::from_secs(10))).unwrap();
    rosrust::ros_info!("add_two_ints service found! calling...");

    // Create client for the service
    let client = rosrust::client::<srv::rosservices::AddTwoInts>("add_two_ints").unwrap();

    // Synchronous call that blocks the thread until a response is received
    rosrust::ros_info!(
        "{} + {} = {}",
        a,
        b,
        client
            .req(&srv::rosservices::AddTwoIntsReq { a, b })
            .unwrap()
            .unwrap()
            .sum
    );

    // Asynchronous call that can be resolved later on/ Multiply a and b by two just to distinguish from previous call
    let retval = client.req_async(srv::rosservices::AddTwoIntsReq { a: a * 2, b: b * 2 });
    rosrust::ros_info!(
        "{} + {} = {}",
        a * 2,
        b * 2,
        retval.read().unwrap().unwrap().sum
    );
}
