// rosmsg_include works properly only with linux line end i.e. LF
// it does now work properly with windows CR LF!!
mod srv {
    rosrust::rosmsg_include!(rosservices / RectArea);
}

fn main() {
    rosrust::init("rect_area_server");

    // Create service
    // The service is stopped when the returned object is destroyed
    let _service_raii =
        rosrust::service::<srv::rosservices::RectArea, _>("rect_area_service", move |req| {
            let area = req.width * req.length;

            rosrust::ros_info!("{} * {} = {}", req.width, req.length, area);

            Ok(srv::rosservices::RectAreaRes { area })
        })
        .unwrap();

    rosrust::ros_info!("rect_area_service running!");

    rosrust::spin();
}
