use crossterm::cursor;
use crossterm::event::{read, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::execute;
use crossterm::style::Print;
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, Clear, ClearType};
use std::io::stdout;

fn main() {
    let mut stdout = stdout();
    enable_raw_mode().unwrap();
    let info_message =
        ">>>Press CTRL + q or CTRL + c to exit, use arrows to navigate the turtle bot.<<<";

    execute!(
        stdout,
        Clear(ClearType::All),
        cursor::MoveTo(0, 0),
        Print(info_message)
    )
    .unwrap();

    rosrust::init("turtle_operator");

    let ros_publisher =
        rosrust::publish::<rosrust_msg::geometry_msgs::Twist>("/turtle1/cmd_vel", 100).unwrap();

    /*
    See http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right

    y-axis
    ^
    |
    |
    |
    |
    |---------------> x-axis
    [0,0]

    */

    loop {
        execute!(stdout, cursor::MoveTo(0, 0)).unwrap();

        match read().unwrap() {
            Event::Key(KeyEvent {
                code: KeyCode::Up,
                modifiers: KeyModifiers::NONE,
            }) => {
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print(info_message),
                    cursor::MoveTo(0, 0),
                    Print("\nGoing Up")
                )
                .unwrap();
                let mut msg = rosrust_msg::geometry_msgs::Twist::default();
                msg.linear.x = 1.0;
                ros_publisher.send(msg).unwrap();
            }

            Event::Key(KeyEvent {
                code: KeyCode::Down,
                modifiers: KeyModifiers::NONE,
            }) => {
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print(info_message),
                    cursor::MoveTo(0, 0),
                    Print("\nGoing down")
                )
                .unwrap();
                let mut msg = rosrust_msg::geometry_msgs::Twist::default();
                msg.linear.x = -1.0;
                ros_publisher.send(msg).unwrap();
            }

            Event::Key(KeyEvent {
                code: KeyCode::Left,
                modifiers: KeyModifiers::NONE,
            }) => {
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print(info_message),
                    cursor::MoveTo(0, 0),
                    Print("\nRotating counter clockwise")
                )
                .unwrap();
                let mut msg = rosrust_msg::geometry_msgs::Twist::default();
                msg.angular.z = 0.5;
                ros_publisher.send(msg).unwrap();
            }

            Event::Key(KeyEvent {
                code: KeyCode::Right,
                modifiers: KeyModifiers::NONE,
            }) => {
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print(info_message),
                    cursor::MoveTo(0, 0),
                    Print("\nRotating clockwise")
                )
                .unwrap();

                let mut msg = rosrust_msg::geometry_msgs::Twist::default();
                msg.angular.z = -0.5;
                ros_publisher.send(msg).unwrap();
            }

            Event::Key(KeyEvent {
                code: KeyCode::Char('q'),
                modifiers: KeyModifiers::CONTROL,
            })
            | Event::Key(KeyEvent {
                code: KeyCode::Char('c'),
                modifiers: KeyModifiers::CONTROL,
            }) => {
                execute!(
                    stdout,
                    Clear(ClearType::All),
                    Print("Quiting now. Bye!\n\n")
                )
                .unwrap();
                break;
            }

            _ => (),
        }
    }

    disable_raw_mode().unwrap();
}
