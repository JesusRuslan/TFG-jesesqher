#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

// -- Goals ---------------------------------------------------------------------------
// TODO: sustituir por los 4 puntos definitivos del mapa warehouse
struct Goal
{
    double x, y;
    std::string name;
};

static const std::vector<Goal> GOALS = {
    {0.0, 1.0, "warmup"},            // goal de calentamiento para cargar caches (no recogido en métricas)
    {-5.0, 1.0, "goal1_recto"},      // placeholder
    {-5.0, 1.0, "goal2_obstaculos"}, // placeholder
    {-5.0, 1.0, "goal3_lejos"},      // placeholder
    {-5.0, 1.0, "goal4_atras"},      // placeholder
};

// -- Struct para almacenar métricas ---------------------------------------------------------------------------
struct Metrics
{
    std::string goal_name;
    double goal_x, goal_y;

    double plan_time_ms = 0.0;    // tiempo de planificación (ms)
    double path_length_m = 0.0;   // longitud total de la trayectoria (m)
    double straight_line_m = 0.0; // distancia en línea recta origen-destino (m)
    double ratio_length = 0.0;    // ratio longitud / línea recta
    double mean_curvature = 0.0;  // curvatura media (rad/m)
    double smoothness = 0.0;      // suma de cambios de ángulo (rad)
    int num_waypoints = 0;        // número de puntos intermedios
    double eta_s = 0.0;           // duración estimada del viaje
};

// -- Nodo principal ---------------------------------------------------------------------------
class NavMetricsNode : public rclcpp::Node
{
public:
    NavMetricsNode() : Node("nav_metrics_node"), current_goal_idx_(0), waiting_for_plan_(false)
    {
        // -- Parámetros configurables ---------------------------------------------------------------------------

        // -- Action client para Nav2 ---------------------------------------------------------------------------
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // -- Suscripción a /plan ---------------------------------------------------------------------------
        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&NavMetricsNode::plan_callback, this, std::placeholders::_1));

        // -- Parámetros configurables ---------------------------------------------------------------------------
        this->declare_parameter("results_dir",
                                std::string(std::getenv("HOME")) + "/TFG-jesesqher/ros2/Simulated/Results/");
        results_dir_ = this->get_parameter("results_dir").as_string();

        this->declare_parameter("planner_name", std::string("dijkstra"));
        std::string planner_name = this->get_parameter("planner_name").as_string();

        this->declare_parameter("smooth", true);
        bool smooth = this->get_parameter("smooth").as_bool();

        // -- Fichero de resultados ---------------------------------------------------------------------------
        std::string suffix = smooth ? "" : "_nosmooth";
        csv_filename_ = "metrics_" + planner_name + suffix + ".csv";

        std::string filepath = results_dir_ + csv_filename_;
        if (std::filesystem::exists(filepath))
        {
            std::filesystem::remove(filepath);
            RCLCPP_INFO(this->get_logger(), "CSV anterior eliminado: %s", filepath.c_str());
        }

        // -- Directorio de resultados ---------------------------------------------------------------------------
        std::filesystem::create_directories(results_dir_);

        RCLCPP_INFO(this->get_logger(), "Nodo de métricas iniciado. Esperando Nav2...");

        // -- Timer para esperar a que Nav2 esté listo antes de empezar ---------------------------------------------------------------------------
        start_timer_ = this->create_wall_timer(
            5s, [this]()
            {
                start_timer_->cancel();
                send_next_goal(); });
    }

private:
    // -- Calcular métricas sobre el plan recibido ---------------------------------------------------------------------------
    Metrics compute_metrics(
        const nav_msgs::msg::Path &path,
        const Goal &goal,
        double plan_time_ms)
    {
        Metrics m;
        m.goal_name = goal.name;
        m.goal_x = goal.x;
        m.goal_y = goal.y;
        m.plan_time_ms = plan_time_ms;
        m.num_waypoints = static_cast<int>(path.poses.size());

        if (path.poses.size() < 2)
            return m;

        // Longitud total
        double total_length = 0.0;
        std::vector<double> angles;

        for (size_t i = 1; i < path.poses.size(); ++i)
        {
            double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
            double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
            double seg = std::sqrt(dx * dx + dy * dy);
            total_length += seg;
            angles.push_back(std::atan2(dy, dx));
        }
        m.path_length_m = total_length;

        // ETA aproximado
        m.eta_s = total_length / 0.306;

        // Distancia en línea recta (primer punto → último punto)
        double dx0 = path.poses.back().pose.position.x - path.poses.front().pose.position.x;
        double dy0 = path.poses.back().pose.position.y - path.poses.front().pose.position.y;
        m.straight_line_m = std::sqrt(dx0 * dx0 + dy0 * dy0);

        // Ratio longitud / línea recta
        if (m.straight_line_m > 0.0)
            m.ratio_length = m.path_length_m / m.straight_line_m;

        // Suavidad = suma de ángulos
        double total_angle_change = 0.0;
        for (size_t i = 1; i < angles.size(); ++i)
        {
            double diff = std::abs(angles[i] - angles[i - 1]);
            // Normalizar a [-pi, pi]
            while (diff > M_PI)
                diff -= 2 * M_PI;
            while (diff < -M_PI)
                diff += 2 * M_PI;
            total_angle_change += std::abs(diff);
        }
        m.smoothness = total_angle_change;

        // Curvatura media = cambio de ángulo total / longitud total
        if (m.path_length_m > 0.0)
            m.mean_curvature = m.smoothness / m.path_length_m;

        return m;
    }

    // -- Guardar métricas en CSV ---------------------------------------------------------------------------
    void save_metrics(const Metrics &m)
    {
        bool file_exists = std::filesystem::exists(filepath);

        std::ofstream file(filepath, std::ios::app);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo: %s", filepath.c_str());
            return;
        }

        // Cabecera solo si el archivo es nuevo
        if (!file_exists)
        {
            file << "goal_name,"
                 << "goal_x,"
                 << "goal_y,"
                 << "plan_time_ms,"
                 << "path_length_m,"
                 << "straight_line_m,"
                 << "ratio_length,"
                 << "smoothness_rad,"
                 << "mean_curvature_rad_m,"
                 << "num_waypoints,eta_s\n";
        }

        file << std::fixed << std::setprecision(4)
             << m.goal_name << ","
             << m.goal_x << ","
             << m.goal_y << ","
             << m.plan_time_ms << ","
             << m.path_length_m << ","
             << m.straight_line_m << ","
             << m.ratio_length << ","
             << m.smoothness << ","
             << m.mean_curvature << ","
             << m.num_waypoints << ","
             << m.eta_s << "\n";

        RCLCPP_INFO(this->get_logger(), "Métricas guardadas en %s", filepath.c_str());
    }

    // -- Enviar el siguiente goal ---------------------------------------------------------------------------
    void send_next_goal()
    {
        if (current_goal_idx_ >= static_cast<int>(GOALS.size()))
        {
            RCLCPP_INFO(this->get_logger(), "Todos los goals completados. Resultados en %s",
                        (results_dir_ + csv_filename_).c_str());
            rclcpp::shutdown();
            return;
        }

        const Goal &goal = GOALS[current_goal_idx_];
        RCLCPP_INFO(this->get_logger(), "Enviando goal %d/%zu: %s (%.2f, %.2f)",
                    current_goal_idx_ + 1, GOALS.size(), goal.name.c_str(), goal.x, goal.y);

        if (!nav_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server no disponible.");
            rclcpp::shutdown();
            return;
        }

        // Anotar timestamp antes de enviar el goal
        goal_sent_time_ = this->get_clock()->now();
        waiting_for_plan_ = true;

        auto nav_goal = NavigateToPose::Goal();
        nav_goal.pose.header.frame_id = "map";
        nav_goal.pose.header.stamp = this->get_clock()->now();
        nav_goal.pose.pose.position.x = goal.x;
        nav_goal.pose.pose.position.y = goal.y;
        nav_goal.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr &handle)
        {
            if (!handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal rechazado por Nav2.");
                rclcpp::shutdown();
            }
        };

        nav_client_->async_send_goal(nav_goal, send_goal_options);
    }

    // -- Callback de /plan ---------------------------------------------------------------------------
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!waiting_for_plan_ || msg->poses.empty())
            return;
        waiting_for_plan_ = false;

        // Tiempo de planificación
        auto now = this->get_clock()->now();
        double plan_time_ms = (now - goal_sent_time_).seconds() * 1000.0;

        const Goal &goal = GOALS[current_goal_idx_];
        Metrics m = compute_metrics(*msg, goal, plan_time_ms);

        // Log en consola
        RCLCPP_INFO(this->get_logger(),
                    "----- Métricas %s -----\n"
                    "   Tiempo planificación    : %.1f ms\n"
                    "   Longitud trayectoria    : %.3f m\n"
                    "   Línea recta             : %.3f m\n"
                    "   Ratio longitud          : %.3f\n"
                    "   Suavidad                : %.3f rad\n"
                    "   Curvatura media         : %.3f rad/m\n"
                    "   Num. waypoints          : %d\n"
                    "   ETA                     : %.1f s",
                    goal.name.c_str(),
                    m.plan_time_ms,
                    m.path_length_m,
                    m.straight_line_m,
                    m.ratio_length,
                    m.smoothness,
                    m.mean_curvature,
                    m.num_waypoints,
                    m.eta_s);

        if (goal.name != "warmup")
        {
            save_metrics(m);
        }

        // Cancelar navegación y pasar al siguiente goal
        nav_client_->async_cancel_all_goals();
        current_goal_idx_++;

        // Esperar un poco antes del siguiente goal para que Nav2 se estabilice
        next_goal_timer_ = this->create_wall_timer(
            3s, [this]()
            {
                next_goal_timer_->cancel();
                send_next_goal(); });
    }

    // -- Miembros ---------------------------------------------------------------------------
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr next_goal_timer_;

    int current_goal_idx_;
    bool waiting_for_plan_;

    rclcpp::Time goal_sent_time_;

    std::string results_dir_;
    std::string csv_filename_;
    std::string filepath;

    std::vector<Metrics> all_metrics_;
};

// -- Main ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavMetricsNode>());
    rclcpp::shutdown();
    return 0;
}