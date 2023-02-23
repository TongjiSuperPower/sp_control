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

            bool generate(const geometry_msgs::Pose &pose, const shapes::Mesh *mesh, moveit_msgs::CollisionObject &obj);

            //bool delete(const moveit_msgs::CollisionObject &obj);
   
            //bool move(const geometry_msgs::Pose &pose, const moveit_msgs::CollisionObject &obj);



        private:
            std::vector<moveit_msgs::CollisionObject> collision_objects;
            moveit_msgs::PlanningScene planning_scene; 
            moveit::planning_interface::PlanningSceneInterface current_scene;
            ros::Publisher planning_scene_diff_publisher;
            ros::NodeHandle nh;
            
            
    };

}