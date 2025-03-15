#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <iomanip>

class FigyeloNode : public rclcpp::Node {
public:
    FigyeloNode() : Node("figyelo_node") {
        homerseklet_fogado = create_subscription<sensor_msgs::msg::Temperature>(
            "/homerseklet", 10, std::bind(&FigyeloNode::homerseklet_ellenorzes, this, std::placeholders::_1));

        paratartalom_fogado = create_subscription<sensor_msgs::msg::RelativeHumidity>(
            "/paratartalom", 10, std::bind(&FigyeloNode::paratartalom_ellenorzes, this, std::placeholders::_1));

        nyomas_fogado = create_subscription<sensor_msgs::msg::FluidPressure>(
            "/nyomas", 10, std::bind(&FigyeloNode::nyomas_ellenorzes, this, std::placeholders::_1));

        riasztas_kuldo = create_publisher<std_msgs::msg::String>("/riasztas", 10);
    }

private:
    void homerseklet_ellenorzes(const sensor_msgs::msg::Temperature::SharedPtr msg) {
        if (msg->temperature > 27.5) {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1) << msg->temperature;

            std_msgs::msg::String riasztas;
            riasztas.data = "[⚠️RIASZTÁS⚠️] Magas hőmérséklet észlelve: "
                            + stream.str() + " °C – azonnali ellenőrzés szükséges!";
            riasztas_kuldo->publish(riasztas);
        }
    }

    void paratartalom_ellenorzes(const sensor_msgs::msg::RelativeHumidity::SharedPtr msg) {
        if (msg->relative_humidity > 0.65) {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(1) << msg->relative_humidity * 100;
            std_msgs::msg::String riasztas;
            riasztas.data = "[⚠️RIASZTÁS⚠️] Magas páratartalom észlelve: " 
                            + stream.str() 
                            + " % – azonnali ellenőrzés szükséges!";
            riasztas_kuldo->publish(riasztas);
        }
    }

    void nyomas_ellenorzes(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
        if (msg->fluid_pressure < 99500.0) {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(0) << msg->fluid_pressure;
            std_msgs::msg::String riasztas;
            riasztas.data = "[⚠️RIASZTÁS⚠️] Alacsony légnyomás észlelve: " 
                            + stream.str() 
                            + " Pa – azonnali ellenőrzés szükséges!";
            riasztas_kuldo->publish(riasztas);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr homerseklet_fogado;
    rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>::SharedPtr paratartalom_fogado;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr nyomas_fogado;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr riasztas_kuldo;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FigyeloNode>());
    rclcpp::shutdown();
    return 0;
}
