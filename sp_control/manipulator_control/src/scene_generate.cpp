#include "manipulator_control/scene_generate.h"


namespace manipulator_control
{
    bool Scene::init()
    {
        planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        moveit_msgs::CollisionObject obj;
        obj.header.frame_id = "base_link";
        obj.id="golden_ore";
        geometry_msgs::Pose initial_pose;
        initial_pose.orientation.w = 1.0;
        initial_pose.position.x = 0.00;
        initial_pose.position.y = 0.50;
        initial_pose.position.z = 0.10;     
        shapes::Mesh *mesh=shapes::createMeshFromResource("package://sp_description/meshes/scene/gloden_ore.STL" );
        bool success = generate(initial_pose, mesh, obj);
        return true;

    }


    bool Scene::generate(const geometry_msgs::Pose &pose, const shapes::Mesh *mesh, moveit_msgs::CollisionObject &obj)
    {
        obj.operation = obj.ADD;
        if(mesh == NULL) {
            ROS_WARN_STREAM("FAIL TO GENERATE A MESH");
            return false;
            
        }
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(mesh, shape_msg);
        obj.meshes.push_back(boost::get<shape_msgs::Mesh>(shape_msg));
        obj.mesh_poses.push_back(pose);
        planning_scene.world.collision_objects.push_back(obj);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
        collision_objects.push_back(obj);
        current_scene.addCollisionObjects(collision_objects);
        ROS_INFO_STREAM("SUCCEED TO GENERATE A MESH");
        return true;       
    }

/*    void Scene::delete(const string obj_id)
    {
        moveit_msgs::CollisionObject remove_obj;
        remove_obj.id = obj_id;
        remove_obj.operation = remoe_obj.REMOVE;
        planning_scene.world.collision_objects.
        planning_scene.world.collision_objects.push_back(remove_obj);
        planning_scene_diff_publisher.publish(planning_scene);
    }

    bool Scene::move(const string obj_id)
    {
        moveit_msgs::CollisionObject move_obj;
        move_obj.id="box";
        move_obj.header.frame_id="base_link";
        geometry_msgs::Pose new_pose = pose;
        new_pose.position.x=pose.position.x+1;
        move_obj.mesh_poses.push_back(new_pose);
        move_obj.operation=move_obj.MOVE;
        planning_scene.world.collision_objects.clear();
        planning_scene.world.collision_objects.push_back(remove_obj);
        //发布消息
        planning_scene_diff_publisher.publish(planning_scene);

    }
    */



            
  

}