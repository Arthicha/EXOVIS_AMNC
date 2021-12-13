function setExoJointPositions_cb(msg)
    -- joint target position
    data = msg.data
    for i = 1,3,1 do
        exo_joint_position[i] = data[i]
    end
    simSetGraphUserData(graph_handle,"exo_lhip",data[1])
    simSetGraphUserData(graph_handle,"exo_lknee",data[2])
end

function setManJointPositions_cb(msg)
    -- joint target position
    data = msg.data
    for i = 1,3,1 do
        man_joint_position[i] = data[i]
    end
    simSetGraphUserData(graph_handle,"man_lhip",data[1])
    simSetGraphUserData(graph_handle,"man_lknee",data[2])
end

function debugMessage_cb(msg)
    -- joint target position
    data = msg.data
    for i = 1,4,1 do
        simSetGraphUserData(graph_handle,"debug"..tostring(i),data[i])
    end
end


function sysCall_init()

    dt = sim.getSimulationTimeStep()
    stepCounter = 0 -- couunting step

    -- initialize object handle
    graph_handle = sim.getObjectHandle("Graph")
    exo_joint_handle = {sim.getObjectHandle("exo_left_hip"),sim.getObjectHandle("exo_left_knee"),sim.getObjectHandle("exo_left_ankle")}
    man_joint_handle = {sim.getObjectHandle("man_left_hip"),sim.getObjectHandle("man_left_knee"),sim.getObjectHandle("exo_left_ankle")}

    exo_joint_position = {0.0,0.0,0.0}
    man_joint_position = {0.0,0.0,0.0}
    exo_joint_position_fb = {0.0,0.0,0.0}
    man_joint_position_fb = {0.0,0.0,0.0}
    man_joint_torque = {0.0,0.0,0.0}
    tracking_error = {0.0,0.0,0.0}


    if simROS then
        print("<font color='#0F0'>ROS interface was found.</font>@html")

        subscribers = {}
        advertisers = {}

        table.insert(subscribers, simROS.subscribe('/exvis/con/jointAngle','std_msgs/Int32MultiArray','setExoJointPositions_cb'))
        table.insert(subscribers, simROS.subscribe('/manJointPosition','std_msgs/Float32MultiArray','setManJointPositions_cb'))
        table.insert(subscribers, simROS.subscribe('/debugMessage','std_msgs/Float32MultiArray','debugMessage_cb'))

        table.insert(advertisers, simROS.advertise('/exvis/jointAngle','std_msgs/Int32MultiArray'))
        table.insert(advertisers, simROS.advertise('/manJointFeedback','std_msgs/Float32MultiArray'))
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end


--______________________________________________________________________--
--______________________________________________________________________--

function sysCall_actuation()

    --t = sim.getSimulationTime()
    print(man_joint_position[1],exo_joint_position[1])
    sim.setJointTargetPosition(man_joint_handle[1],-1*man_joint_position[1]/57.2958)
    sim.setJointTargetPosition(exo_joint_handle[1],-1*exo_joint_position[1]/57.2958)
    sim.setJointTargetPosition(man_joint_handle[2],1*man_joint_position[2]/57.2958)
    sim.setJointTargetPosition(exo_joint_handle[2],1*exo_joint_position[2]/57.2958)
    sim.setJointTargetPosition(man_joint_handle[3],0*man_joint_position[3]/57.2958)
    sim.setJointTargetPosition(exo_joint_handle[3],0*exo_joint_position[3]/57.2958)
end

--______________________________________________________________________--

function sysCall_sensing()


    for i = 1,3,1 do
        exo_joint_position_fb[i] = math.floor(sim.getJointPosition(exo_joint_handle[i])*180/3.14)
        man_joint_position_fb[i] = sim.getJointPosition(man_joint_handle[i])
        man_joint_torque[i] = 0.995*man_joint_torque[i]+0.005*math.abs(sim.getJointForce(man_joint_handle[i]))
        tracking_error[i] = 0.995*tracking_error[i]+0.005*math.abs(exo_joint_position[i]-exo_joint_position_fb[i])
    end
    exo_joint_position_fb[1] = -1*exo_joint_position_fb[1]
    man_joint_position_fb[1] = -1*man_joint_position_fb[1]
    simROS.publish(advertisers[1],{data=exo_joint_position_fb}) 
    simROS.publish(advertisers[2],{data=man_joint_position_fb}) 


end


--______________________________________________________________________--
function sysCall_cleanup()
    -- do some clean-up here
    -- Terminate Controller
    -- Wait for the signal to reach the node
    waitTimer=0
    simROS.publish(advertisers[1],{data=true})  
    print('terminate')
    while( waitTimer < 1000 ) do
        waitTimer = waitTimer+1
        simROS.publish(advertisers[1],{data=true})
    end

    for i=1,table.getn(subscribers),1 do
        simROS.shutdownSubscriber(subscribers[i])
    end 

    for i=1,table.getn(advertisers),1 do
        simROS.shutdownPublisher(advertisers[i])
    end
end

-- See the user manual or the available code snippets for additional callback functions and details

