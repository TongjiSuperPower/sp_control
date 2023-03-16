#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


namespace manipulator_control
{
   

    class Scene
    {
        public:
            Scene() = default;

            bool init();

            moveit_msgs::CollisionObject generate_collision_obj(const std::string &obj_id, shapes::Mesh *mesh);

            moveit_msgs::AttachedCollisionObject generate_attach_collision_obj(const std::string &obj_id, shapes::Mesh *mesh);

            bool add(moveit_msgs::CollisionObject &add_obj, const geometry_msgs::Pose &pose);
    
            bool remove(const moveit_msgs::CollisionObject &remove_obj);
   
            bool move(moveit_msgs::CollisionObject &move_obj, const geometry_msgs::Pose &pose);

            bool attach(moveit_msgs::AttachedCollisionObject &attach_obj, moveit::planning_interface::MoveGroupInterface &grip_group_interface);

            bool detach(const moveit_msgs::AttachedCollisionObject &detach_obj, moveit::planning_interface::MoveGroupInterface &grip_group_interface);



        private:
            std::vector<moveit_msgs::CollisionObject> collision_objects;
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            ros::NodeHandle nh;
            
            
    };

}