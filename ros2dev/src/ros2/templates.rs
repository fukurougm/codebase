//! Package and file templates for ROS 2

use anyhow::Result;
use std::path::Path;

/// Template generator for ROS 2 packages and files
pub struct Ros2Templates;

impl Ros2Templates {
    /// Generate package.xml content
    pub fn package_xml(
        name: &str,
        description: &str,
        maintainer: &str,
        email: &str,
        build_type: &str,
        dependencies: &[String],
    ) -> String {
        let dep_lines: String = dependencies
            .iter()
            .map(|d| format!("  <depend>{}</depend>", d))
            .collect::<Vec<_>>()
            .join("\n");

        format!(
            r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>{description}</description>
  <maintainer email="{email}">{maintainer}</maintainer>
  <license>MIT</license>

  <buildtool_depend>{build_type}</buildtool_depend>

{dep_lines}

  <export>
    <build_type>{build_type}</build_type>
  </export>
</package>
"#
        )
    }

    /// Generate CMakeLists.txt for C++ package
    pub fn cmake_lists(name: &str, nodes: &[String], dependencies: &[String]) -> String {
        let find_deps: String = dependencies
            .iter()
            .map(|d| format!("find_package({} REQUIRED)", d))
            .collect::<Vec<_>>()
            .join("\n");

        let dep_list = dependencies.join(" ");

        let node_targets: String = nodes
            .iter()
            .map(|n| {
                format!(
                    r#"
add_executable({node} src/{node}.cpp)
ament_target_dependencies({node} {deps})
install(TARGETS {node}
  DESTINATION lib/${{PROJECT_NAME}})"#,
                    node = n,
                    deps = dep_list
                )
            })
            .collect::<Vec<_>>()
            .join("\n");

        format!(
            r#"cmake_minimum_required(VERSION 3.8)
project({name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
{find_deps}
{node_targets}

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${{PROJECT_NAME}}/launch
  OPTIONAL
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${{PROJECT_NAME}}/config
  OPTIONAL
)

ament_package()
"#
        )
    }

    /// Generate setup.py for Python package
    pub fn setup_py(name: &str, nodes: &[String], maintainer: &str, email: &str) -> String {
        let entry_points: String = nodes
            .iter()
            .map(|n| format!("            '{n} = {name}.{n}:main',", n = n, name = name))
            .collect::<Vec<_>>()
            .join("\n");

        format!(
            r#"from setuptools import find_packages, setup

package_name = '{name}'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{maintainer}',
    maintainer_email='{email}',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
{entry_points}
        ],
    }},
)
"#
        )
    }

    /// Generate setup.cfg for Python package
    pub fn setup_cfg(name: &str) -> String {
        format!(
            r#"[develop]
script_dir=$base/lib/{name}
[install]
install_scripts=$base/lib/{name}
"#
        )
    }

    /// Generate C++ node template
    pub fn cpp_node(node_name: &str, class_name: &str) -> String {
        format!(
            r#"#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class {class_name} : public rclcpp::Node
{{
public:
    {class_name}() : Node("{node_name}")
    {{
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("output", 10);

        // Create subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "input", 10,
            std::bind(&{class_name}::topic_callback, this, std::placeholders::_1));

        // Create timer (1 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&{class_name}::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "{class_name} node started");
    }}

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {{
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }}

    void timer_callback()
    {{
        auto message = std_msgs::msg::String();
        message.data = "Hello from {node_name}";
        publisher_->publish(message);
    }}

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
}};

int main(int argc, char * argv[])
{{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<{class_name}>());
    rclcpp::shutdown();
    return 0;
}}
"#
        )
    }

    /// Generate Python node template
    pub fn python_node(node_name: &str, class_name: &str) -> String {
        format!(
            r#"#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'output', 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'input',
            self.listener_callback,
            10)
        
        # Create timer (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('{class_name} node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{{msg.data}}"')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from {node_name}'
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = {class_name}()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"#
        )
    }

    /// Generate Python launch file
    pub fn launch_py(package_name: &str, nodes: &[(&str, &str)]) -> String {
        let node_actions: String = nodes
            .iter()
            .map(|(pkg, node)| {
                format!(
                    r#"
        Node(
            package='{pkg}',
            executable='{node}',
            name='{node}',
            output='screen',
            parameters=[],
        ),"#
                )
            })
            .collect::<Vec<_>>()
            .join("");

        format!(
            r#"#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([{node_actions}
    ])
"#
        )
    }

    /// Generate XML launch file
    pub fn launch_xml(nodes: &[(&str, &str)]) -> String {
        let node_elements: String = nodes
            .iter()
            .map(|(pkg, node)| {
                format!(
                    r#"
    <node pkg="{pkg}" exec="{node}" name="{node}" output="screen"/>"#
                )
            })
            .collect::<Vec<_>>()
            .join("");

        format!(
            r#"<?xml version="1.0"?>
<launch>{node_elements}
</launch>
"#
        )
    }

    /// Generate subscriber-only Python node
    pub fn python_subscriber(node_name: &str, topic: &str, msg_type: &str) -> String {
        // Extract package and type
        let parts: Vec<&str> = msg_type.split('/').collect();
        let (msg_pkg, msg_name): (&str, &str) = if parts.len() >= 2 {
            (parts[0], *parts.last().unwrap_or(&"String"))
        } else {
            ("std_msgs.msg", "String")
        };

        format!(
            r#"#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from {msg_pkg}.msg import {msg_name}


class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.subscription = self.create_subscription(
            {msg_name},
            '{topic}',
            self.listener_callback,
            10)
        self.get_logger().info('Subscribing to {topic}')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {{msg}}')


def main(args=None):
    rclpy.init(args=args)
    node = {class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"#,
            msg_pkg = msg_pkg.replace("/", "."),
            msg_name = msg_name,
            class_name = Self::to_class_name(node_name),
            node_name = node_name,
            topic = topic,
        )
    }

    /// Generate publisher test node
    pub fn python_publisher(node_name: &str, topic: &str, msg_type: &str) -> String {
        let parts: Vec<&str> = msg_type.split('/').collect();
        let (msg_pkg, msg_name): (&str, &str) = if parts.len() >= 2 {
            (parts[0], *parts.last().unwrap_or(&"String"))
        } else {
            ("std_msgs.msg", "String")
        };

        format!(
            r#"#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from {msg_pkg}.msg import {msg_name}


class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.publisher_ = self.create_publisher({msg_name}, '{topic}', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Publishing to {topic}')

    def timer_callback(self):
        msg = {msg_name}()
        # TODO: Fill message fields
        self.publisher_.publish(msg)
        self.count += 1
        self.get_logger().info(f'Published message #{{self.count}}')


def main(args=None):
    rclpy.init(args=args)
    node = {class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"#,
            msg_pkg = msg_pkg.replace("/", "."),
            msg_name = msg_name,
            class_name = Self::to_class_name(node_name),
            node_name = node_name,
            topic = topic,
        )
    }

    /// Generate service client test node
    pub fn python_service_client(node_name: &str, service: &str, srv_type: &str) -> String {
        let parts: Vec<&str> = srv_type.split('/').collect();
        let (srv_pkg, srv_name): (&str, &str) = if parts.len() >= 2 {
            (parts[0], *parts.last().unwrap_or(&"Empty"))
        } else {
            ("std_srvs.srv", "Empty")
        };

        format!(
            r#"#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from {srv_pkg}.srv import {srv_name}


class {class_name}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.client = self.create_client({srv_name}, '{service}')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service {service} available')

    def send_request(self):
        request = {srv_name}.Request()
        # TODO: Fill request fields
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = {class_name}()
    
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)
    
    try:
        response = future.result()
        node.get_logger().info(f'Response: {{response}}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {{e}}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"#,
            srv_pkg = srv_pkg.replace("/", "."),
            srv_name = srv_name,
            class_name = Self::to_class_name(node_name),
            node_name = node_name,
            service = service,
        )
    }

    /// Convert snake_case to PascalCase for class names
    pub fn to_class_name(name: &str) -> String {
        name.split('_')
            .map(|part| {
                let mut chars = part.chars();
                match chars.next() {
                    None => String::new(),
                    Some(first) => first.to_uppercase().chain(chars).collect(),
                }
            })
            .collect()
    }

    /// Create a new package on disk
    pub fn create_package(
        workspace_src: &Path,
        name: &str,
        build_type: &str,
        description: &str,
        maintainer: &str,
        email: &str,
        dependencies: &[String],
        initial_node: Option<&str>,
    ) -> Result<()> {
        let pkg_path = workspace_src.join(name);
        std::fs::create_dir_all(&pkg_path)?;

        // Create package.xml
        let package_xml = Self::package_xml(name, description, maintainer, email, build_type, dependencies);
        std::fs::write(pkg_path.join("package.xml"), package_xml)?;

        // Create directories
        std::fs::create_dir_all(pkg_path.join("launch"))?;

        match build_type {
            "ament_cmake" => {
                std::fs::create_dir_all(pkg_path.join("src"))?;
                std::fs::create_dir_all(pkg_path.join("include").join(name))?;

                let nodes = if let Some(node) = initial_node {
                    vec![node.to_string()]
                } else {
                    Vec::new()
                };

                let cmake = Self::cmake_lists(name, &nodes, dependencies);
                std::fs::write(pkg_path.join("CMakeLists.txt"), cmake)?;

                if let Some(node) = initial_node {
                    let class_name = Self::to_class_name(node);
                    let cpp_content = Self::cpp_node(node, &class_name);
                    std::fs::write(pkg_path.join("src").join(format!("{}.cpp", node)), cpp_content)?;
                }
            }
            "ament_python" => {
                let py_pkg = pkg_path.join(name);
                std::fs::create_dir_all(&py_pkg)?;
                std::fs::create_dir_all(pkg_path.join("resource"))?;

                // Create __init__.py
                std::fs::write(py_pkg.join("__init__.py"), "")?;

                // Create resource marker
                std::fs::write(pkg_path.join("resource").join(name), "")?;

                let nodes = if let Some(node) = initial_node {
                    vec![node.to_string()]
                } else {
                    Vec::new()
                };

                let setup_py = Self::setup_py(name, &nodes, maintainer, email);
                std::fs::write(pkg_path.join("setup.py"), setup_py)?;

                let setup_cfg = Self::setup_cfg(name);
                std::fs::write(pkg_path.join("setup.cfg"), setup_cfg)?;

                if let Some(node) = initial_node {
                    let class_name = Self::to_class_name(node);
                    let py_content = Self::python_node(node, &class_name);
                    std::fs::write(py_pkg.join(format!("{}.py", node)), py_content)?;
                }
            }
            _ => {}
        }

        Ok(())
    }
}
