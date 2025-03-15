#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

class AdatMegjelenito : public rclcpp::Node {
public:
    AdatMegjelenito() : Node("adat_megjelenito") {
        homerseklet_fogado = create_subscription<sensor_msgs::msg::Temperature>(
            "/homerseklet", 10, std::bind(&AdatMegjelenito::homerseklet_kiir, this, std::placeholders::_1));

        paratartalom_fogado = create_subscription<sensor_msgs::msg::RelativeHumidity>(
            "/paratartalom", 10, std::bind(&AdatMegjelenito::paratartalom_kiir, this, std::placeholders::_1));

        nyomas_fogado = create_subscription<sensor_msgs::msg::FluidPressure>(
            "/nyomas", 10, std::bind(&AdatMegjelenito::nyomas_kiir, this, std::placeholders::_1));
    }

private:
    void homerseklet_kiir(const sensor_msgs::msg::Temperature::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "🌡️ Hőmérséklet: %.1f °C", msg->temperature);
    }

    void paratartalom_kiir(const sensor_msgs::msg::RelativeHumidity::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "💧 Páratartalom: %.1f%%", msg->relative_humidity * 100);
    }

    void nyomas_kiir(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "🔵 Légnyomás: %.0f Pa", msg->fluid_pressure);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr homerseklet_fogado;
    rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>::SharedPtr paratartalom_fogado;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr nyomas_fogado;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdatMegjelenito>());
    rclcpp::shutdown();
    return 0;
}
