function sysCall_init()

    -- {position, orientation, front, back, block, insects}
    
    hoop1 = {} --green

    hoop1[1] = sim.getObjectHandle('Position_hoop1')
    hoop1[2] = sim.getObjectHandle('Orientation_hoop1')
    hoop1[3] = sim.getObjectHandle('Front_hoop1')
    hoop1[4] = sim.getObjectHandle('Back_hoop1')
    hoop1[5] = sim.getObjectHandle('Block1')
    hoop1[6] = 1

    hoop2 = {} --yellow

    hoop2[1] = sim.getObjectHandle('Position_hoop2')
    hoop2[2] = sim.getObjectHandle('Orientation_hoop2')
    hoop2[3] = sim.getObjectHandle('Front_hoop2')
    hoop2[4] = sim.getObjectHandle('Back_hoop2')
    hoop2[5] = sim.getObjectHandle('Block2')
    hoop2[6] = 1

    hoop3 = {} --red

    hoop3[1] = sim.getObjectHandle('Position_hoop3')
    hoop3[2] = sim.getObjectHandle('Orientation_hoop3')
    hoop3[3] = sim.getObjectHandle('Front_hoop3')
    hoop3[4] = sim.getObjectHandle('Back_hoop3')
    hoop3[5] = sim.getObjectHandle('Block3')
    hoop3[6] = 1

    obstacle1 = sim.getObjectHandle('obstacle_1')
    obstacle2 = sim.getObjectHandle('obstacle_2')

    home_tree = hoop3

    current_tree = nil

    start_handle = sim.getObjectHandle('Start')
    drone_handle = sim.getObjectHandle('eDrone')

    collection_handles= sim.getCollectionHandle('Obstacles')

    zero_pos = {0,0,0}

    sim.setObjectPosition(hoop1[5],hoop1[2],zero_pos)
    sim.setObjectPosition(hoop2[5],hoop2[2],zero_pos)
    sim.setObjectPosition(hoop3[5],hoop3[2],zero_pos)

    t=simOMPL.createTask('ta')
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-1.35,-0.8,0.3},{0.6,0.8,2},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})
    no_of_path_points_required = 30

    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    --psstate = simROS.subscribe('/psstate', 'std_msgs/Float64', 'plan_path1234')

    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')

    --plan_path()
    hoop_pos = {0,0,0}
    quat = {0,0,0,1}
end

function plan_path()
    local path = {}
    --sim.setObjectPosition(home_tree[5],home_tree[3],zero_pos)
    joinpath(path,calcpath(start_handle,home_tree[4]))
    joinpath(path,calcpath(home_tree[4],home_tree[3]))
    joinpath(path,calcpath(home_tree[3],home_tree[4]))
    --sim.setObjectPosition(home_tree[5],home_tree[2],zero_pos)

    if (hoop1 ~= home_tree) then
        joinpath(path,traverse_hoop(hoop1))
    end

    if (hoop2 ~= home_tree) then
        joinpath(path,traverse_hoop(hoop2))
    end

    if (hoop3 ~= home_tree) then
        joinpath(path,traverse_hoop(hoop3))
    end

    --sim.setObjectPosition(home_tree[5],home_tree[3],zero_pos)
    joinpath(path,calcpath(home_tree[4],start_handle))
    --sim.setObjectPosition(home_tree[5],home_tree[2],zero_pos)

    print("final_path",#path)
    visualizePath(path)
    local message = packdata(path)
    print("message_length",#message.poses)
    simROS.publish(path_pub,message)
    print("Done")
    
end

function traverse_hoop(hoop_arg)
    local path = {}
    --sim.setObjectPosition(hoop_arg[5],hoop_arg[3],zero_pos)
    --sim.setObjectPosition(home_tree[5],home_tree[4],zero_pos)
    local p1 = calcpath(home_tree[4],hoop_arg[4])
    --print("traverse_hoop p1",#p1)
    --sim.setObjectPosition(hoop_arg[5],hoop_arg[4],zero_pos)
    --sim.setObjectPosition(home_tree[5],home_tree[3],zero_pos)
    --print("traverse_hoop p2",#p2)
    --sim.setObjectPosition(home_tree[5],home_tree[2],zero_pos)
    --sim.setObjectPosition(hoop_arg[5],hoop_arg[2],zero_pos)

    --print("traverse_hoop_before_join_1",#path)
    joinpath(path,p1)
    --joinpath(path,calcpath(hoop_arg[4],hoop_arg[3]))

    joinpath(path,singlePoint(hoop_arg[4]))
    joinpath(path,calcpath(hoop_arg[4],hoop_arg[3]))
    joinpath(path,singlePoint(hoop_arg[3]))

    joinpath(path,calcpath(hoop_arg[3],hoop_arg[4]))
    joinpath(path,singlePoint(hoop_arg[4]))

    local p2 = calcpath(hoop_arg[4],home_tree[4])
    joinpath(path,p2)
    --joinpath(path,getpose(home_tree[3],-1))

    joinpath(path,calcpath(home_tree[4],home_tree[3]))
    joinpath(path,calcpath(home_tree[3],home_tree[4]))
    --joinpath(path,getpose(home_tree[3],-1))
    --print("traverse_hoop_before_join_2",#path)
    --joinpath(path,p2)
    --print("traverse_hoop_before_join_3",#path)
    --joinpath(path,getpose(home_tree[3],-1))

    return path
end

function calcpath(from_handle,to_handle)
    local start_pose = getpose(from_handle,-1)
    local goal_pose = getpose(to_handle,-1)

    simOMPL.setStartState(t,start_pose)
    
    simOMPL.setGoalState(t,goal_pose)
    
    local status = false
    local path

    repeat
        status,path=simOMPL.compute(t,10,-1,no_of_path_points_required)
    until (status)

    print("calc_path",status,#path)

    return path
end

function singlePoint(handle)
    path = {}
    path = getpose(handle,-1)

    for i=1,3,1 do
        for j=1,7,1 do
            path[i*7+j] = path[j]
        end
    end

    return path
end

function joinpath(path1,path2)
    local size = #path1
    for i=1,#path2,1 do
        path1[size+i] = path2[i]
    end
end

function sysCall_actuation()
    
end

function sysCall_sensing()
    
end

function sysCall_cleanup()
    -- do some clean-up here
    simROS.shutdownPublisher(path_pub)
    --simROS.shutdownSubscriber(psstate)
end

function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,#path-6,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }
        pose.position.x = path[i]
        pose.position.y = path[i+1]
        pose.position.z = path[i+2]

        pose = realtowhycon(pose)

        sender.poses[math.floor(i/7) + 1] = pose
    end

    --for i=1,10,1 do
    --    sender.poses[no_of_path_points_required+i] = sender.poses[no_of_path_points_required]
    --end

    --print(sender)

    return sender
end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    --orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],0,0,0,1}
    return pose
end

function realtowhycon(pose)
    --local whycon_origin = {-0.16,0.05,32.4}
    --local scale_factors = {9.814,-10.057,-14.44}
    --local whycon_origin = {0.1,0.12,27.31}
    --local scale_factors = {9.686,-10.157,-14.673}
    --local scale_factors = {9.186,-10.11,-8.643}
    --local whycon_origin = {-0.03,-0.11,23.544}    --26.31
    local scale_factors = {8.945,-9.679,-8.16}
    local whycon_origin = {0.33,-0.424,23.868}    --26.31

    pose.position.x = pose.position.x*scale_factors[1] + whycon_origin[1]
    pose.position.y = pose.position.y*scale_factors[2] + whycon_origin[2]
    pose.position.z = pose.position.z*scale_factors[3] + whycon_origin[3]

    return pose
end

function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectQuaternion
    quat = {0,0,0,1}
    quat[1] = (msg.markers[1].pose.pose.orientation.x)
    quat[2] = (msg.markers[1].pose.pose.orientation.y)
    quat[3] = (msg.markers[1].pose.pose.orientation.z)
    quat[4] = (msg.markers[1].pose.pose.orientation.w)


    local x = quat[1]
    local y = quat[2]
    local z = quat[3]
    local w = quat[4]

    quat[1] = (x-y-z+w)/2
    quat[2] = -(x+y-z-w)/2
    quat[3] = -(-x-y-z-w)/2
    quat[4] = (x-y+z-w)/2

end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
    
    hoop_pos = {0,0,0}

    --local s = {0.104,-0.102,-0.116}
    --local c = {0.037,-0.013,2.84} --2.105
    --local s = {0.102,-0.099,-0.069}
    --local c = {0.016,0.005,2.243} --2.105
    --local s = {0.103,-0.098,-0.068}
    --local c = {-0.01,0.011,1.861} --2.105
    --local s = {0.109,-0.098,-0.116}
    --local c = {0.003,0.011,2.724} --2.105
    local s = {0.111,-0.103,-0.123}
    local c = {-0.036,-0.044,2.925} --2.105

    hoop_pos[1] = s[1]*(msg.poses[1].position.x)+c[1]
    hoop_pos[2] = s[2]*(msg.poses[1].position.y)+c[2]
    hoop_pos[3] = s[3]*(msg.poses[1].position.z)+c[3]

    local drone_pos = hoop_pos
    sim.setObjectPosition(drone_handle,-1,drone_pos)
end

--[[
Function Name: key_callback
Input: Int16
Output: none
Logic: Sets the position and orientation of all the trees one by one. First, only one aruco and whycon markers are kept in the scene,
       and the '/' key is pressed. This positions the Sal tree. Then, the markers are moved to the second tree and '/' is pressed and 
       so on, until all the trees are correctly positioned.
]]
function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    if (msg.data == 65) then
    	current_tree = hoop3
    	sim.setObjectPosition(current_tree[1],-1,hoop_pos)
    	sim.setObjectQuaternion(current_tree[2],current_tree[1],quat)
    elseif (msg.data == 75) then
    	current_tree = hoop2
    	sim.setObjectPosition(current_tree[1],-1,hoop_pos)
    	sim.setObjectQuaternion(current_tree[2],current_tree[1],quat)
    elseif (msg.data == 85) then
    	current_tree = hoop1
    	sim.setObjectPosition(current_tree[1],-1,hoop_pos)
    	sim.setObjectQuaternion(current_tree[2],current_tree[1],quat)

    elseif (msg.data == 105) then
    	sim.setObjectPosition(obstacle1,-1,hoop_pos)
    elseif (msg.data == 115) then
    	sim.setObjectPosition(obstacle2,-1,hoop_pos)
    
    elseif (msg.data == 95) then
    	home_tree = current_tree
        print("Home tree set")

    elseif (msg.data == 25) then
    	current_tree[6] = 0
    elseif (msg.data == 35) then
    	current_tree[6] = 1
    elseif (msg.data == 45) then
    	current_tree[6] = 2
	elseif (msg.data == 55) then
    	current_tree[6] = 3

    elseif (msg.data == 500) then
        sim.setObjectPosition(start_handle,-1,{hoop_pos[1],hoop_pos[2],0.51})
        plan_path()
    end
end