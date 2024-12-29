//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-License-Identifier: Apache-2.0

use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

struct RepublisherNode {
    node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}

impl RepublisherNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "republisher")?;
        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);
        let _subscription = node.create_subscription(
            "in_topic",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: StringMsg| {
                *data_cb.lock().unwrap() = Some(msg);
                dbg!("{:?}", &data_cb);
            },
        )?;
        Ok(Self {
            node,
            _subscription,
            data,
        })
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = RepublisherNode::new(&context)?;
    rclrs::spin(republisher.node)
}
