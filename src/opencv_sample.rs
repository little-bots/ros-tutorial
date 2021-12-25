use opencv::{core::Mat, highgui, imgcodecs, Result};

fn read_image(image_path: &str, show_flg: bool, window_name: Option<String>) -> Result<Mat> {
    let image = imgcodecs::imread(image_path, 0)?;
    if show_flg {
        let window = window_name.unwrap_or("sample".to_string());
        highgui::named_window(&window, 0)?;
        highgui::imshow(&window, &image)?;
    }

    Ok(image)
}

fn main() -> Result<()> {
    let image = read_image("/tmp/tree.jpg", true, Some("opencv sample".to_string()));
    highgui::wait_key(10000)?;
    Ok(())
}
