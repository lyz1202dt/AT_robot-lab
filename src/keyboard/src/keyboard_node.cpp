#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/remote.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {

constexpr uint32_t bit(int index)
{
    return 1u << index;
}

class TerminalRawMode {
public:
    TerminalRawMode()
    {
        if (!isatty(STDIN_FILENO)) {
            return;
        }

        if (tcgetattr(STDIN_FILENO, &original_) != 0) {
            return;
        }

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
            active_ = true;
        }
    }

    ~TerminalRawMode()
    {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
        }
    }

    bool active() const
    {
        return active_;
    }

private:
    termios original_{};
    bool active_{false};
};

}  // namespace

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode()
        : Node("keyboard_node")
    {
        linear_value_ = static_cast<float>(declare_parameter<double>("linear_value", 1200.0));
        angular_value_ = static_cast<float>(declare_parameter<double>("angular_value", 1200.0));
        command_timeout_ = std::chrono::duration<double>(
            declare_parameter<double>("command_timeout", 0.5));
        pulse_duration_ = std::chrono::duration<double>(
            declare_parameter<double>("key_pulse_duration", 0.2));
        const auto publish_rate = declare_parameter<double>("publish_rate", 50.0);

        remote_pub_ = create_publisher<robot_msgs::msg::Remote>("remote", 10);

        print_help();

        input_thread_ = std::thread([this]() { input_loop(); });

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publish_remote(); });
    }

    ~KeyboardNode() override
    {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    enum class Mode {
        Manual,
        Auto,
        Record,
    };

    struct Axis {
        float value{0.0f};
        std::chrono::steady_clock::time_point updated{};
    };

    void print_help()
    {
        RCLCPP_INFO(get_logger(), "Keyboard remote publisher started.");
        RCLCPP_INFO(get_logger(), "Move: w/s forward/back, a/d left/right, q/e spin.");
        RCLCPP_INFO(get_logger(), "Modes: 1 manual, 2 auto, 3 record.");
        RCLCPP_INFO(get_logger(), "Policy keys: z stand, x walk, c sand, v stairs, b robot_lab_bar(bit 12), n robot_lab_slope(bit 11), h robot_lab_bridge(bit 13), m record point.");
        RCLCPP_INFO(get_logger(), "Use Ctrl-C to exit.");
    }

    void input_loop()
    {
        TerminalRawMode terminal;
        if (!terminal.active()) {
            RCLCPP_WARN(get_logger(), "stdin is not a TTY; keyboard input may be unavailable.");
        }

        pollfd fd{};
        fd.fd = STDIN_FILENO;
        fd.events = POLLIN;

        while (running_ && rclcpp::ok()) {
            const int ready = poll(&fd, 1, 50);
            if (ready <= 0 || !(fd.revents & POLLIN)) {
                continue;
            }

            char ch = 0;
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                handle_key(ch);
            }
        }
    }

    void handle_key(char ch)
    {
        const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        const auto now = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(mutex_);

        switch (key) {
            case 'w':
                ly_axis_ = Axis{linear_value_, now};
                break;
            case 's':
                ly_axis_ = Axis{-linear_value_, now};
                break;
            case 'a':
                lx_axis_ = Axis{-linear_value_, now};
                break;
            case 'd':
                lx_axis_ = Axis{linear_value_, now};
                break;
            case 'q':
                rx_axis_ = Axis{-angular_value_, now};
                break;
            case 'e':
                rx_axis_ = Axis{angular_value_, now};
                break;
            case ' ':
                lx_axis_ = Axis{};
                ly_axis_ = Axis{};
                rx_axis_ = Axis{};
                break;
            case '2':
                set_mode(Mode::Manual, "manual");
                break;
            case '3':
                set_mode(Mode::Auto, "auto");
                break;
            case '1':
                set_mode(Mode::Record, "record");
                break;
            case 'z':
                pulse_key_bit(4, "stand");
                break;
            case 'x':
                pulse_key_bit(5, "walk");
                break;
            case 'c':
                pulse_key_bit(6, "sand");
                break;
            case 'v':
                pulse_key_bit(3, "stairs");
                break;
            case 'b':
                pulse_key_bit(12, "robot_lab_bar");
                break;
            case 'n':
                pulse_key_bit(11, "robot_lab_slope");
                break;
            case 'h':
                pulse_key_bit(13, "robot_lab_bridge");
                break;
            case 'm':
                pulse_key_bit(14, "record_point");
                break;
            default:
                break;
        }
    }

    void set_mode(Mode mode, const std::string& name)
    {
        if (mode_ == mode) {
            return;
        }

        mode_ = mode;
        RCLCPP_INFO(get_logger(), "Keyboard mode: %s", name.c_str());
    }

    void pulse_key_bit(int index, const std::string& name)
    {
        pulse_bit_ = bit(index);
        pulse_until_ = std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(pulse_duration_);
        RCLCPP_INFO(get_logger(), "Keyboard key pulse: %s (bit %d)", name.c_str(), index);
    }

    void publish_remote()
    {
        robot_msgs::msg::Remote msg;
        const auto now = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            msg.lx = axis_value(lx_axis_, now);
            msg.ly = axis_value(ly_axis_, now);
            msg.rx = axis_value(rx_axis_, now);
            msg.ry = 0.0f;
            msg.key = mode_key_bits();

            if (now <= pulse_until_) {
                msg.key |= pulse_bit_;
            } else {
                pulse_bit_ = 0;
            }
        }

        msg.just_reconnected = false;
        remote_pub_->publish(msg);
    }

    float axis_value(const Axis& axis, std::chrono::steady_clock::time_point now) const
    {
        if (axis.updated.time_since_epoch().count() == 0) {
            return 0.0f;
        }
        if (now - axis.updated > command_timeout_) {
            return 0.0f;
        }
        return axis.value;
    }

    uint32_t mode_key_bits() const
    {
        switch (mode_) {
            case Mode::Auto:
                return bit(1);
            case Mode::Record:
                return bit(2);
            case Mode::Manual:
            default:
                return 0;
        }
    }

    rclcpp::Publisher<robot_msgs::msg::Remote>::SharedPtr remote_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::thread input_thread_;
    std::atomic_bool running_{true};
    std::mutex mutex_;

    float linear_value_{1200.0f};
    float angular_value_{1200.0f};
    std::chrono::duration<double> command_timeout_{0.5};
    std::chrono::duration<double> pulse_duration_{0.2};

    Axis lx_axis_;
    Axis ly_axis_;
    Axis rx_axis_;
    Mode mode_{Mode::Manual};
    uint32_t pulse_bit_{0};
    std::chrono::steady_clock::time_point pulse_until_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
