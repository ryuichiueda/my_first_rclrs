//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: Apache-2.0

use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;
use std_msgs::msg::Header;
use rclrs::Clock;
use builtin_interfaces::msg::Time;

struct RepublisherNode {
    node: Arc<rclrs::Node>,
    //_subscription: Arc<rclrs::Subscription<LaserScan>>,
    _subscription: Arc<rclrs::Subscription<LaserScan>>,
    data: Arc<Mutex<Option<LaserScan>>>,
    publisher: Arc<rclrs::Publisher<LaserScan>>,
    obstacle_map: Arc<rclrs::Publisher<OccupancyGrid>>,
}

impl RepublisherNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "republisher")?;
        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);
        let _subscription = node.create_subscription(
            "scan",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: LaserScan| { *data_cb.lock().unwrap() = Some(msg); },
        )?;

        let publisher = node.create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)?;
        let obstacle_map = node.create_publisher("obstacle_map", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            _subscription,
            publisher,
            data,
            obstacle_map,
        })
    }

    fn republish(&self, frame_id: usize) -> Result<(), rclrs::RclrsError> {
        if let Some(s) = &*self.data.lock().unwrap() {
            self.publisher.publish(s)?;
        }

        let clock = Clock::system();
        let now = clock.now();
        let nanosec = (now.nsec as u32 ) % 1_000_000_000;
        let sec = (now.nsec / 1_000_000_000) as i32;

        let header = Header {
            stamp: Time{ nanosec, sec },
            frame_id: frame_id.to_string(),
        };

        /*
        {
            let mut i = self.frame_id.lock().unwrap();
            *i += 1;
        }*/


        dbg!("{:?}", &header);

        let map = OccupancyGrid::default();
        self.obstacle_map.publish(map)?;

        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(RepublisherNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        let mut frame_id = 0;
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            republisher_other_thread.republish(frame_id)?;

            frame_id += 1;
        }
    });

    rclrs::spin(Arc::clone(&republisher.node))
}
