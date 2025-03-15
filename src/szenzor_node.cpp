#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <random>

class SzenzorNode : public rclcpp::Node {
public:
    SzenzorNode() : Node("szenzor_node"), generator(std::random_device{}()),
        homerseklet_dist(15.0, 30.0), paratartalom_dist(30.0, 70.0) {

        homerseklet_kuldo = create_publisher<sensor_msgs::msg::Temperature>("/homerseklet", 10);
        paratartalom_kuldo = create_publisher<sensor_msgs::msg::RelativeHumidity>("/paratartalom", 10);
        idozito = create_wall_timer(std::chrono::seconds(1), std::bind(&SzenzorNode::adat_kuld, this));
    }

private:
    void adat_kuld() {
        sensor_msgs::msg::Temperature homerseklet_uzenet;
        homerseklet_uzenet.temperature = homerseklet_dist(generator);
        homerseklet_uzenet.header.stamp = this->now();
        homerseklet_kuldo->publish(homerseklet_uzenet);

        sensor_msgs::msg::RelativeHumidity paratartalom_uzenet;
        paratartalom_uzenet.relative_humidity = paratartalom_dist(generator) / 100.0;
        paratartalom_uzenet.header.stamp = this->now();
        paratartalom_kuldo->publish(paratartalom_uzenet);
    }

    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr homerseklet_kuldo;
    rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr paratartalom_kuldo;
    rclcpp::TimerBase::SharedPtr idozito;
    std::mt19937 generator;
    std::uniform_real_distribution<double> homerseklet_dist;
    std::uniform_real_distribution<double> paratartalom_dist;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SzenzorNode>());
    rclcpp::shutdown();
    return 0;
}
