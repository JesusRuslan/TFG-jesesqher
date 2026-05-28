#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

// -- Goals ---------------------------------------------------------------------------
struct Goal
{
    double x, y;
    std::string name;
};

static const std::vector<Goal> GOALS = {
    {0.0, 1.0, "warmup"},
    {-13.0, 19.0, "goal_dinamico"},
};

// -- Muestra temporal ---------------------------------------------------------------------------
struct Sample
{
    double timestamp_s = 0.0;  // momento en el que se toma el sample
    double pos_real_x = 0.0;   // coordenada x real del robot
    double pos_real_y = 0.0;   // coordenada y real del robot
    double pos_plan_x = 0.0;   // coordenada x planificada
    double pos_plan_y = 0.0;   // coordenada y planificada
    double desviacion_m = 0.0; // desviación en metros de la posición real respecto al punto más cercano dentro de la trayectoria
    double eta_s = 0.0;        // eta aproximado calculado en cada momento
    double vel_lineal = 0.0;   // velocidad lineal del robot
    double vel_angular = 0.0;  // velocidad angular del robot
};

// -- Nodo ---------------------------------------------------------------------------
class NavMetricsDynamicNode : public rclcpp::Node
{
public:
    NavMetricsDynamicNode()
        : Node("nav_metrics_dynamic_node"),
          current_goal_idx_(0),
          navigating_(false),
          start_time_(this->get_clock()->now())
    {
        // -- Parámetros ---------------------------------------------------------------------------

        // -- Action client para Nav2 ---------------------------------------------------------------------------
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // -- Suscripciones a /plan y /odom ---------------------------------------------------------------------------
        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&NavMetricsDynamicNode::plan_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&NavMetricsDynamicNode::odom_callback, this, std::placeholders::_1));

        // -- Parámetros configurables ---------------------------------------------------------------------------
        this->declare_parameter("results_dir",
                                std::string(std::getenv("HOME")) + "/TFG-jesesqher/ros2/Simulated/Results/");
        results_dir_ = this->get_parameter("results_dir").as_string();

        this->declare_parameter("planner_name", std::string("dijkstra"));
        std::string planner_name = this->get_parameter("planner_name").as_string();

        this->declare_parameter("smooth", true);
        bool smooth = this->get_parameter("smooth").as_bool();

        // -- Directorio de resultados ---------------------------------------------------------------------------
        std::filesystem::create_directories(results_dir_);

        // -- Fichero de resultados ---------------------------------------------------------------------------
        std::string suffix = smooth ? "" : "_nosmooth";
        csv_filename_ = "metrics_" + planner_name + suffix + "_dinamico.csv";

        filepath_ = results_dir_ + csv_filename_;

        if (std::filesystem::exists(filepath_))
        {
            std::filesystem::remove(filepath_);
            RCLCPP_INFO(this->get_logger(), "CSV anterior eliminado: %s", filepath_.c_str());
        }
        
        // -- Timer de muestreo a 5 Hz ---------------------------------------------------------------------------
        sample_timer_ = this->create_wall_timer(
            200ms,
            std::bind(&NavMetricsDynamicNode::sample_callback, this));

        // -- Signal handler para Ctrl+C ---------------------------------------------------------------------------
        std::signal(SIGINT, NavMetricsDynamicNode::signal_handler);
        instance_ = this;

        RCLCPP_INFO(this->get_logger(), "Nodo dinámico iniciado. CSV: %s. Esperando a Nav2", filepath_.c_str());

        // -- Timer de arranque ---------------------------------------------------------------------------
        start_timer_ = this->create_wall_timer(
            5s, [this]()
            {
                start_timer_->cancel();
                send_next_goal(); });

    }

    // -- Guardar CSV al salir ---------------------------------------------------------------------------
    void save_and_shutdown()
    {
        RCLCPP_INFO(this->get_logger(), "Guardando %zu muestras en %s",
                    samples_.size(), filepath_.c_str());
        save_csv();
        rclcpp::shutdown();
    }

private:
    // -- Signal handler estático ---------------------------------------------------------------------------
    static void signal_handler(int)
    {
        if (instance_)
            instance_->save_and_shutdown();
    }

    // -- Callback de odometría ---------------------------------------------------------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        vel_lineal_ = msg->twist.twist.linear.x;
        vel_angular_ = msg->twist.twist.angular.z;
    }

    // -- Callback de plan ---------------------------------------------------------------------------
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
            return;
        current_plan_ = msg->poses;
        RCLCPP_INFO(this->get_logger(), "Plan recibido con %zu puntos",
                    current_plan_.size());
    }

    // -- Muestreo a 5 Hz ---------------------------------------------------------------------------
    void sample_callback()
    {
        if (!navigating_ || current_plan_.empty())
            return;

        // Punto más cercano del plan a la posición real
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        for (size_t i = 0; i < current_plan_.size(); ++i)
        {
            double dx = current_plan_[i].pose.position.x - current_x_;
            double dy = current_plan_[i].pose.position.y - current_y_;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < min_dist)
            {
                min_dist = d;
                closest_idx = i;
            }
        }

        Sample s;
        s.timestamp_s = (this->get_clock()->now() - nav_start_time_).seconds();
        s.pos_real_x = current_x_;
        s.pos_real_y = current_y_;
        s.pos_plan_x = current_plan_[closest_idx].pose.position.x;
        s.pos_plan_y = current_plan_[closest_idx].pose.position.y;
        s.desviacion_m = min_dist;
        s.eta_s = current_eta_s_;
        s.vel_lineal = vel_lineal_;
        s.vel_angular = vel_angular_;

        samples_.push_back(s);
    }

    // -- Guardar CSV ---------------------------------------------------------------------------
    void save_csv()
    {
        std::ofstream file(filepath_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir: %s", filepath_.c_str());
            return;
        }

        file << "timestamp_s,pos_real_x,pos_real_y,pos_plan_x,pos_plan_y,"
             << "desviacion_m,eta_s,vel_lineal,vel_angular\n";

        for (const auto &s : samples_)
        {
            file << std::fixed << std::setprecision(4)
                 << s.timestamp_s << ","
                 << s.pos_real_x << ","
                 << s.pos_real_y << ","
                 << s.pos_plan_x << ","
                 << s.pos_plan_y << ","
                 << s.desviacion_m << ","
                 << s.eta_s << ","
                 << s.vel_lineal << ","
                 << s.vel_angular << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "CSV guardado: %s", filepath_.c_str());
    }

    // -- Enviar siguiente goal ---------------------------------------------------------------------------
    void send_next_goal()
    {
        if (current_goal_idx_ >= static_cast<int>(GOALS.size()))
        {
            save_csv();
            rclcpp::shutdown();
            return;
        }

        const Goal &goal = GOALS[current_goal_idx_];
        RCLCPP_INFO(this->get_logger(), "Enviando goal: %s (%.2f, %.2f)",
                    goal.name.c_str(), goal.x, goal.y);

        if (!nav_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server no disponible.");
            rclcpp::shutdown();
            return;
        }

        auto nav_goal = NavigateToPose::Goal();
        nav_goal.pose.header.frame_id = "map";
        nav_goal.pose.header.stamp = this->get_clock()->now();
        nav_goal.pose.pose.position.x = goal.x;
        nav_goal.pose.pose.position.y = goal.y;
        nav_goal.pose.pose.orientation.w = 1.0;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr &handle)
        {
            if (!handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal rechazado por Nav2.");
                rclcpp::shutdown();
            }
        };

        options.feedback_callback =
            [this](GoalHandleNav::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            current_eta_s_ = feedback->estimated_time_remaining.sec +
                             feedback->estimated_time_remaining.nanosec * 1e-9;
        };

        options.result_callback =
            [this](const GoalHandleNav::WrappedResult &result)
        {
            navigating_ = false;
            const std::string &name = GOALS[current_goal_idx_].name;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal %s completado.", name.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Goal %s no completado.", name.c_str());
            }

            current_goal_idx_++;
            // Si era el warmup, limpiar muestras antes del goal real
            if (name == "warmup")
            {
                samples_.clear();
                current_plan_.clear();
            }

            next_goal_timer_ = this->create_wall_timer(
                3s, [this]()
                {
                        next_goal_timer_->cancel();
                        send_next_goal(); });
        };

        // Activar muestreo solo para el goal real
        if (goal.name != "warmup")
        {
            navigating_ = true;
            nav_start_time_ = this->get_clock()->now();
        }

        nav_client_->async_send_goal(nav_goal, options);
    }

    // -- Miembros ---------------------------------------------------------------------------
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr sample_timer_;
    rclcpp::TimerBase::SharedPtr next_goal_timer_;

    int current_goal_idx_;
    bool navigating_;
    double current_x_{0.0}, current_y_{0.0};
    double vel_lineal_{0.0}, vel_angular_{0.0};
    double current_eta_s_{0.0};

    std::vector<geometry_msgs::msg::PoseStamped> current_plan_;
    std::vector<Sample> samples_;

    rclcpp::Time start_time_;
    rclcpp::Time nav_start_time_;

    std::string results_dir_;
    std::string csv_filename_;
    std::string filepath_;

    static NavMetricsDynamicNode *instance_;
};

NavMetricsDynamicNode *NavMetricsDynamicNode::instance_ = nullptr;

// -- Main ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavMetricsDynamicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}