#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <std_msgs/msg/string.hpp>

class FigyeloNode : public rclcpp::Node {
public:
    FigyeloNode() : Node("figyelo_node") {
        homerseklet_fogado = create_subscription<sensor_msgs::msg::Temperature>(
            "/homerseklet", 10, std::bind(&FigyeloNode::homerseklet_ellenorzes, this, std::placeholders::_1));

        paratartalom_fogado = create_subscription<sensor_msgs::msg::RelativeHumidity>(
            "/paratartalom", 10, std::bind(&FigyeloNode::paratartalom_ellenorzes, this, std::placeholders::_1));

        riasztas_kuldo = create_publisher<std_msgs::msg::String>("/riasztas", 10);
    }

private:
    void homerseklet_ellenorzes(const sensor_msgs::msg::Temperature::SharedPtr msg) {
        if (msg->temperature > 25.0) {
            std_msgs::msg::String riasztas;
            riasztas.data = "Magas hőmérséklet: " + std::to_string(msg->temperature) + " °C";
            riasztas_kuldo->publish(riasztas);
        }
    }

    void paratartalom_ellenorzes(const sensor_msgs::msg::RelativeHumidity::SharedPtr msg) {
        if (msg->relative_humidity > 0.6) {  // 60% helyesen
            std_msgs::msg::String riasztas;
            riasztas.data = "Magas páratartalom: " + std::to_string(msg->relative_humidity * 100) + "%";
            riasztas_kuldo->publish(riasztas);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr homerseklet_fogado;
    rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>::SharedPtr paratartalom_fogado;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr riasztas_kuldo;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FigyeloNode>());
    rclcpp::shutdown();
    return 0;
}
