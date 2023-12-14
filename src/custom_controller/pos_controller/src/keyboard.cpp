#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <termios.h>

class Keyboard : public rclcpp::Node
{
  public:
    Keyboard() : Node("keyboard")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard", 10);
      printf("Press 'q' to exit.\n");
      while (true) {
        auto message = std_msgs::msg::String();
        message.data = getch();
        printf("\n");
        if (message.data == "q") {
            exit(0);
        }
        publisher_->publish(message);
      }
    }

  private:
    int getch()
    {
      static struct termios oldt, newt;
      tcgetattr( STDIN_FILENO, &oldt);           // save old settings
      newt = oldt;
      newt.c_lflag &= ~(ICANON);                 // disable buffering      
      tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

      int c = getchar();  // read character (non-blocking)

      tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
      return c;
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Keyboard>());
  rclcpp::shutdown();
  return 0;
}
