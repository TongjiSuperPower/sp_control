#include "manipulator_control/scene_generate.h"


namespace manipulator_control
{
    bool Scene::init()
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
        return true;
    }

    moveit_msgs::CollisionObject Scene::generate_collision_obj(const std::string &obj_id, shapes::Mesh *mesh)
    {
        moveit_msgs::CollisionObject obj;
        obj.id = obj_id;   
        obj.header.frame_id = "base_link";
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(mesh, shape_msg);
        obj.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));      
        ROS_INFO_STREAM("SUCCEED TO GENERATE A MESH");
        return obj;       
    }

    moveit_msgs::AttachedCollisionObject Scene::generate_attach_collision_obj(const std::string &obj_id, shapes::Mesh *mesh)
    {
        moveit_msgs::AttachedCollisionObject obj;
        obj.object.id = obj_id;   
        obj.object.header.frame_id = "base_link";
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(mesh, shape_msg);
        obj.object.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));      
        ROS_INFO_STREAM("SUCCEED TO GENERATE A MESH");
        return obj;       
    }

    bool Scene::add(moveit_msgs::CollisionObject &add_obj, const geometry_msgs::Pose &pose)
    {
        add_obj.operation = add_obj.ADD;    
        add_obj.mesh_poses.push_back(pose);
        collision_objects.clear();
        collision_objects.push_back(add_obj);     
        planning_scene_interface.addCollisionObjects(collision_objects);
        ROS_INFO_STREAM("SUCCEED TO ADD A MESH");
        return true;
    }

    bool Scene::remove(const moveit_msgs::CollisionObject &remove_obj)
    {
        std::vector<std::string> object_ids;
        object_ids.push_back(remove_obj.id);
        planning_scene_interface.removeCollisionObjects(object_ids);
        ROS_INFO_STREAM("SUCCEED TO REMOVE A MESH");
        return true;
    }

    bool Scene::move(moveit_msgs::CollisionObject &move_obj, const geometry_msgs::Pose &pose)
    {
        remove(move_obj);
        add(move_obj, pose);
        ROS_INFO_STREAM("SUCCEED TO MOVE A MESH");
        return true;
    }

    bool Scene::attach(moveit_msgs::AttachedCollisionObject &attach_obj, moveit::planning_interface::MoveGroupInterface &grip_group_interface)
    {
        attach_obj.object.header.frame_id = grip_group_interface.getEndEffectorLink();
        ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
        grip_group_interface.attachObject(attach_obj.object.id, "link7");
        attach_obj.touch_links = std::vector<std::string>{"link7"};
        return true;
    }

    bool Scene::detach(const moveit_msgs::AttachedCollisionObject &detach_obj, moveit::planning_interface::MoveGroupInterface &grip_group_interface)
    {
        ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
        grip_group_interface.detachObject(detach_obj.object.id);
        return true;
    }



    

    



            
  

}