use opencv::{core, highgui, imgcodecs, imgproc, prelude::*, videoio, Result};

fn read_image(
    image_path: &str,
    imread_flags: i32,
    show_flg: bool,
    window_name: Option<String>,
) -> Result<core::Mat> {
    let image = imgcodecs::imread(image_path, imread_flags)?;
    if show_flg {
        show_image(&image, window_name)?;
    }

    Ok(image)
}

fn show_image(image: &core::Mat, window_name: Option<String>) -> Result<()> {
    let window = window_name.unwrap_or("sample".to_string());
    highgui::named_window(&window, 0)?;
    highgui::imshow(&window, &image)?;
    Ok(())
}

#[allow(dead_code)]
fn create_empty_image(rows: i32, cols: i32, typ: i32) -> Result<core::MatExpr> {
    core::Mat::zeros(rows, cols, typ)
}

fn adaptive_tresholding(
    gray_image: &dyn core::ToInputArray,
    binary_image: &mut dyn core::ToOutputArray,
    threshold_value: f64,
    adaptive_method: i32,
    threshold_type: i32,
    block_size: i32,
    c: f64,
) -> Result<()> {
    imgproc::adaptive_threshold(
        gray_image,
        binary_image,
        threshold_value,
        adaptive_method,
        threshold_type,
        block_size,
        c,
    )?;
    Ok(())
}

#[allow(dead_code)]
fn show_trees() -> Result<()> {
    let _ = read_image(
        "/tmp/tree.jpg",
        imgcodecs::IMREAD_COLOR,
        true,
        Some("opencv sample".to_string()),
    );

    let gray_image = read_image("/tmp/tree.jpg", imgcodecs::IMREAD_GRAYSCALE, false, None)?;

    // let mut binary_image = create_empty_image(gray_image.rows(), gray_image.cols(), core::CV_32F)?;
    let mut binary_image_inv = core::Mat::default();
    let mut binary_image = core::Mat::default();

    adaptive_tresholding(
        &gray_image,
        &mut binary_image_inv,
        155.0,
        core::BORDER_REPLICATE,
        imgproc::THRESH_BINARY_INV,
        255,
        2.0,
    )?;

    adaptive_tresholding(
        &gray_image,
        &mut binary_image,
        155.0,
        core::BORDER_REPLICATE,
        imgproc::THRESH_BINARY,
        255,
        2.0,
    )?;

    show_image(&binary_image, Some("binary image".to_string()))?;
    show_image(&binary_image_inv, Some("binary image inverted".to_string()))?;

    highgui::wait_key(0)?;
    highgui::destroy_all_windows()?;
    Ok(())
}

/// taken from https://github.com/twistedfall/opencv-rust/blob/master/examples/video_to_gray.rs
/// analogy to https://github.com/aniskoubaa/ros_essentials_cpp/blob/ros-noetic/src/topic03_perception/read_video.py
#[allow(dead_code)]
fn read_video() -> Result<()> {
    let window = "video capture";
    highgui::named_window(window, 1)?;
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    let opened = videoio::VideoCapture::is_opened(&cam)?;
    if !opened {
        panic!("Unable to open default camera!");
    }
    loop {
        let mut frame = core::Mat::default();
        cam.read(&mut frame)?;
        if frame.size()?.width > 0 {
            let mut gray = core::Mat::default();
            let mut gray_resized = core::Mat::default();
            imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0)?;
            imgproc::resize(
                &gray,
                &mut gray_resized,
                core::Size::new(0, 0),
                0.5,
                0.5,
                imgproc::INTER_LINEAR,
            )?;
            // highgui::imshow(window, &gray)?;
            highgui::imshow(window, &gray_resized)?;
        }
        if highgui::wait_key(10)? > 0 {
            break;
        }
    }
    Ok(())
}

fn main() -> Result<()> {
    // show_trees()?;
    read_video()?;
    Ok(())
}
