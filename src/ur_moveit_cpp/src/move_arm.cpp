#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>              
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>      
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "ur_move_cpp_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Empleamos hilos para no bloquear el programa, es necesario
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;       // Controlar el brazo
    using moveit::planning_interface::PlanningSceneInterface;   // Para trabjar con objetos

    MoveGroupInterface move_group(node, "ur_manipulator");
    PlanningSceneInterface planning_scene_interface;

    // Información útil: en qué frame se expresan las poses y cuál es el efector
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s | End effector: %s",
                move_group.getPlanningFrame().c_str(),
                move_group.getEndEffectorLink().c_str());

    // 1. OBSTÁCULO: Un pilar central delgado (10x10 cm y 40 cm de alto)
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "pilar_central";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.10;
    primitive.dimensions[primitive.BOX_Y] = 0.10;
    primitive.dimensions[primitive.BOX_Z] = 0.40;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.25; // Frente al robot
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.20; // Mitad de la altura

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    RCLCPP_INFO(node->get_logger(), "Añadiendo pilar de colisión...");
    planning_scene_interface.applyCollisionObject(collision_object);

    // Nos aseguramos de que se cree la trayectoria con el objeto en el camino o escena
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Bloque de código para elegir pose estricta del robot
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.orientation.w = 1.0;
    // target_pose.position.x = 0.45; 
    // target_pose.position.y = 0.15;
    // target_pose.position.z = 0.20;
    
    // 2. DEFINIR OBJETIVO CARTESIANO (Solo XYZ, ignorando la rotación de la muñeca)
    // Nos movemos a la izquierda del pilar y un poco hacia adelante. 
    // La distancia total es de unos 350 mm, perfectamente dentro del radio del UR3.
    move_group.setPositionTarget(0.20, 0.25, 0.30);     // Mandamos la pose a la que queremos que llegue el robot

    // --- 3. PLANIFICAR Y EJECUTAR ---
    MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(node->get_logger(), "Calculando ruta esquivando el muro...");
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "¡Ruta segura encontrada! Moviendo robot...");
        // execute() también puede fallar (p. ej. controlador de trayectoria inactivo)
        const auto exec_result = move_group.execute(my_plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(),
                         "La ejecución falló (código %d). ¿Está activo "
                         "scaled_joint_trajectory_controller?", exec_result.val);
            success = false;
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Colisión inminente: Imposible encontrar ruta segura.");
    }

    // Cierre ordenado: parar el executor, esperar al hilo y luego apagar ROS
    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
}