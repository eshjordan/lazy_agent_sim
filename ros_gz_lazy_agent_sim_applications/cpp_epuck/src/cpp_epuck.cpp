#include <cpp_epuck/UDPComms.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    robot_id_type robot_id                 = 0;
    HostSizeString manager_host            = "localhost";
    uint16_t manager_port                  = 12345;
    HostSizeString robot_comms_host        = "localhost";
    uint16_t robot_comms_request_port      = 12346;
    HostSizeString robot_knowledge_host    = "localhost";
    uint16_t robot_knowledge_exchange_port = 12347;

    auto robot_model = RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>(
        robot_id, manager_host, manager_port, robot_comms_host, robot_comms_request_port, robot_knowledge_host,
        robot_knowledge_exchange_port);

    rclcpp::spin(std::make_shared<rclcpp::Node>("cpp_epuck"));
    return 0;
}
