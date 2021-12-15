use crossterm::cursor;
use crossterm::event::{read, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::execute;
use crossterm::style::Print;
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, Clear, ClearType};
use crossterm::Result;
use rosrust;
use std::io::stdout;

mod msg {
    rosrust::rosmsg_include!(turtlesim / Pose);
}

fn main() -> Result<()> {
    // let mut stdout = stdout();
    //going into raw mode
    enable_raw_mode()?;
    let info_message = ">>>Press CTRL + q or CTRL + c to exit<<<";

    execute!(
        stdout(),
        Clear(ClearType::All),
        cursor::MoveTo(0, 0),
        Print(info_message)
    )?;

    rosrust::init("turtle_locator");

    let _subscriber = rosrust::subscribe("/turtle1/pose", 100, move |v: msg::turtlesim::Pose| {
        // rosrust::ros_info!("Received position: {:#?}", v);

        execute!(
            stdout(),
            Clear(ClearType::All),
            Print(info_message),
            cursor::MoveTo(0, 0),
            Print(format!("\nReceived position: {:?}", v))
        )
        .unwrap();
    })
    .unwrap();

    loop {
        execute!(stdout(), cursor::MoveTo(0, 0))?;

        match read().unwrap() {
            Event::Key(KeyEvent {
                code: KeyCode::Char('q'),
                modifiers: KeyModifiers::CONTROL,
            })
            | Event::Key(KeyEvent {
                code: KeyCode::Char('c'),
                modifiers: KeyModifiers::CONTROL,
            }) => {
                execute!(
                    stdout(),
                    Clear(ClearType::All),
                    Print("Quiting now. Bye!\n\n")
                )?;
                break;
            }

            _ => (),
        }
    }

    //disabling raw mode
    disable_raw_mode()?;

    Ok(())
}
