sim = require('sim')

-- =========================================================
-- ROS2_All_Robot_Bridge
--
-- ???
-- 1. ???? R1~R5 ????? ROS2 ??
-- 2. ? ROS2 ???? CoppeliaSim ?? signal
-- 3. ????????????????
-- 4. ????????????? ROS2 ??
-- =========================================================

local ENABLE_ROS2 = true

-- ??????
local STATUS_TOPIC = '/compact_cell/status'

-- ?????
local MAIN_CMD_TOPIC = '/compact_cell/main_cmd'

-- ???????
local R1_CMD_TOPIC = '/compact_cell/r1_cmd'
local R2_CMD_TOPIC = '/compact_cell/r2_cmd'
local R3_CMD_TOPIC = '/compact_cell/r3_cmd'
local R4_CMD_TOPIC = '/compact_cell/r4_cmd'
local R5_CMD_TOPIC = '/compact_cell/r5_cmd'

-- ????????????
local GLOBAL_CMD_TOPIC = '/compact_cell/cmd'

local simROS2 = nil
local ros2Ready = false

local statusPub = nil

local subMain = nil
local subR1 = nil
local subR2 = nil
local subR3 = nil
local subR4 = nil
local subR5 = nil
local subGlobal = nil


-- =========================================================
-- ????
-- =========================================================

local function publishStatus(text)
    print('[ROS2 BRIDGE STATUS] ' .. text)

    if ros2Ready and statusPub then
        simROS2.publish(statusPub, {data = text})
    end
end

local function setSignal(name, value)
    sim.setStringSignal(name, value)
    print('[SIGNAL] ' .. name .. ' = ' .. value)
end

local function startsWith(str, prefix)
    return string.sub(str, 1, string.len(prefix)) == prefix
end


-- =========================================================
-- ???????
-- =========================================================

local function handleMainCommand(cmd)
    print('[MAIN CMD] ' .. cmd)

    if cmd == 'RESET_CELL' then
        setSignal('cell_product_state', 'reset')
        publishStatus('DONE:RESET_CELL')

    elseif cmd == 'SHOW_ASSEMBLY_SHELL' then
        setSignal('cell_product_state', 'assembly_shell')
        publishStatus('DONE:SHOW_ASSEMBLY_SHELL')

    elseif cmd == 'SHOW_ASSEMBLY_PCB' then
        setSignal('cell_product_state', 'assembly_pcb')
        publishStatus('DONE:SHOW_ASSEMBLY_PCB')

    elseif cmd == 'SHOW_ASSEMBLY_MODULE' then
        setSignal('cell_product_state', 'assembly_module')
        publishStatus('DONE:SHOW_ASSEMBLY_MODULE')

    elseif cmd == 'SHOW_ASSEMBLY_FULL' then
        setSignal('cell_product_state', 'assembly_full')
        publishStatus('DONE:SHOW_ASSEMBLY_FULL')

    elseif cmd == 'SHOW_INSPECTION_FULL' then
        setSignal('cell_product_state', 'inspection_full')
        publishStatus('DONE:SHOW_INSPECTION_FULL')

    elseif cmd == 'CAMERA_GOOD' then
        setSignal('cell_product_state', 'camera_good')
        publishStatus('DONE:CAMERA_GOOD')

    elseif cmd == 'CAMERA_DEFECT' then
        setSignal('cell_product_state', 'camera_defect')
        publishStatus('DONE:CAMERA_DEFECT')

    elseif cmd == 'CONVEYOR_GOOD' then
        setSignal('cell_conveyor_state', 'good')
        publishStatus('DONE:CONVEYOR_GOOD')

    elseif cmd == 'CONVEYOR_DEFECT' then
        setSignal('cell_conveyor_state', 'defect')
        publishStatus('DONE:CONVEYOR_DEFECT')

    else
        publishStatus('ERROR:UNKNOWN_MAIN_COMMAND:' .. cmd)
    end
end


-- =========================================================
-- R1 ????
-- =========================================================
-- R1????? + ?????
-- =========================================================

local function handleR1Command(cmd)
    print('[R1 CMD] ' .. cmd)

    -- ??? R1 ???????????
    setSignal('r1_ros_cmd', cmd)

    if cmd == 'R1_BOX_PLACED' then
        -- R1 ??????????
        setSignal('cell_product_state', 'assembly_shell')
        publishStatus('DONE:R1_BOX_PLACED')

    elseif cmd == 'R1_TERMINAL_PLACED' then
        -- R1 ??????????
        setSignal('cell_product_state', 'assembly_full')
        publishStatus('DONE:R1_TERMINAL_PLACED')

    elseif cmd == 'R1_READY' then
        publishStatus('DONE:R1_READY')

    else
        -- ?? R1 ?????????????
        publishStatus('FORWARD:R1:' .. cmd)
    end
end


-- =========================================================
-- R2 ????
-- =========================================================
-- R2?PCB ??
-- =========================================================

local function handleR2Command(cmd)
    print('[R2 CMD] ' .. cmd)

    setSignal('r2_ros_cmd', cmd)

    if cmd == 'R2_PCB_PLACED' then
        -- R2 ??? PCB ??????
        setSignal('cell_product_state', 'assembly_pcb')
        publishStatus('DONE:R2_PCB_PLACED')

    elseif cmd == 'R2_READY' then
        publishStatus('DONE:R2_READY')

    else
        publishStatus('FORWARD:R2:' .. cmd)
    end
end


-- =========================================================
-- R3 ????
-- =========================================================
-- R3??????? + ?????????
-- =========================================================

local function handleR3Command(cmd)
    print('[R3 CMD] ' .. cmd)

    setSignal('r3_ros_cmd', cmd)

    if cmd == 'R3_MODULE_PLACED' then
        -- R3 ???????????
        setSignal('cell_product_state', 'assembly_module')
        publishStatus('DONE:R3_MODULE_PLACED')

    elseif cmd == 'R3_PRODUCT_TO_INSPECTION' then
        -- R3 ????????????
        setSignal('cell_product_state', 'inspection_full')
        publishStatus('DONE:R3_PRODUCT_TO_INSPECTION')

    elseif cmd == 'R3_READY' then
        publishStatus('DONE:R3_READY')

    else
        publishStatus('FORWARD:R3:' .. cmd)
    end
end


-- =========================================================
-- R4 ????
-- =========================================================
-- R4???????
-- =========================================================

local function handleR4Command(cmd)
    print('[R4 CMD] ' .. cmd)

    setSignal('r4_ros_cmd', cmd)

    if cmd == 'R4_SCREW_DONE' then
        -- R4 ????????
        setSignal('cell_screw_state', 'done')
        publishStatus('DONE:R4_SCREW_DONE')

    elseif cmd == 'R4_READY' then
        publishStatus('DONE:R4_READY')

    else
        publishStatus('FORWARD:R4:' .. cmd)
    end
end


-- =========================================================
-- R5 ????
-- =========================================================
-- R5???? / ?????
-- =========================================================

local function handleR5Command(cmd)
    print('[R5 CMD] ' .. cmd)

    setSignal('r5_ros_cmd', cmd)

    if cmd == 'R5_SORT_GOOD_DONE' then
        -- R5 ????????????
        setSignal('cell_conveyor_state', 'good')
        publishStatus('DONE:R5_SORT_GOOD_DONE')

    elseif cmd == 'R5_SORT_DEFECT_DONE' then
        -- R5 ????????????
        setSignal('cell_conveyor_state', 'defect')
        publishStatus('DONE:R5_SORT_DEFECT_DONE')

    elseif cmd == 'R5_READY' then
        publishStatus('DONE:R5_READY')

    else
        publishStatus('FORWARD:R5:' .. cmd)
    end
end


-- =========================================================
-- ??????
-- =========================================================
-- ?????? /compact_cell/cmd
-- ????????? R1~R5 ????
-- =========================================================

local function handleGlobalCommand(cmd)
    print('[GLOBAL CMD] ' .. cmd)

    if startsWith(cmd, 'R1_') then
        handleR1Command(cmd)

    elseif startsWith(cmd, 'R2_') then
        handleR2Command(cmd)

    elseif startsWith(cmd, 'R3_') then
        handleR3Command(cmd)

    elseif startsWith(cmd, 'R4_') then
        handleR4Command(cmd)

    elseif startsWith(cmd, 'R5_') then
        handleR5Command(cmd)

    else
        handleMainCommand(cmd)
    end
end


-- =========================================================
-- ROS2 ????
-- =========================================================

function mainCmdCallback(msg)
    local cmd = msg.data
    handleMainCommand(cmd)
end

function r1CmdCallback(msg)
    local cmd = msg.data
    handleR1Command(cmd)
end

function r2CmdCallback(msg)
    local cmd = msg.data
    handleR2Command(cmd)
end

function r3CmdCallback(msg)
    local cmd = msg.data
    handleR3Command(cmd)
end

function r4CmdCallback(msg)
    local cmd = msg.data
    handleR4Command(cmd)
end

function r5CmdCallback(msg)
    local cmd = msg.data
    handleR5Command(cmd)
end

function globalCmdCallback(msg)
    local cmd = msg.data
    handleGlobalCommand(cmd)
end


-- =========================================================
-- CoppeliaSim ??
-- =========================================================

function sysCall_init()
    if not ENABLE_ROS2 then
        print('[ROS2 BRIDGE] disabled.')
        return
    end

    local ok, plugin = pcall(require, 'simROS2')

    if not ok then
        print('[ROS2 BRIDGE ERROR] simROS2 failed to load.')
        print(plugin)
        return
    end

    simROS2 = plugin

    statusPub = simROS2.createPublisher(
        STATUS_TOPIC,
        'std_msgs/msg/String'
    )

    subMain = simROS2.createSubscription(
        MAIN_CMD_TOPIC,
        'std_msgs/msg/String',
        'mainCmdCallback'
    )

    subR1 = simROS2.createSubscription(
        R1_CMD_TOPIC,
        'std_msgs/msg/String',
        'r1CmdCallback'
    )

    subR2 = simROS2.createSubscription(
        R2_CMD_TOPIC,
        'std_msgs/msg/String',
        'r2CmdCallback'
    )

    subR3 = simROS2.createSubscription(
        R3_CMD_TOPIC,
        'std_msgs/msg/String',
        'r3CmdCallback'
    )

    subR4 = simROS2.createSubscription(
        R4_CMD_TOPIC,
        'std_msgs/msg/String',
        'r4CmdCallback'
    )

    subR5 = simROS2.createSubscription(
        R5_CMD_TOPIC,
        'std_msgs/msg/String',
        'r5CmdCallback'
    )

    -- ???????
    subGlobal = simROS2.createSubscription(
        GLOBAL_CMD_TOPIC,
        'std_msgs/msg/String',
        'globalCmdCallback'
    )

    ros2Ready = true

    print('[ROS2 BRIDGE] ready.')
    print('[ROS2 BRIDGE] subscribe: ' .. MAIN_CMD_TOPIC)
    print('[ROS2 BRIDGE] subscribe: ' .. R1_CMD_TOPIC)
    print('[ROS2 BRIDGE] subscribe: ' .. R2_CMD_TOPIC)
    print('[ROS2 BRIDGE] subscribe: ' .. R3_CMD_TOPIC)
    print('[ROS2 BRIDGE] subscribe: ' .. R4_CMD_TOPIC)
    print('[ROS2 BRIDGE] subscribe: ' .. R5_CMD_TOPIC)
    print('[ROS2 BRIDGE] subscribe: ' .. GLOBAL_CMD_TOPIC)
    print('[ROS2 BRIDGE] publish:   ' .. STATUS_TOPIC)

    publishStatus('READY:ROS2_ALL_ROBOT_BRIDGE')
end

function sysCall_cleanup()
    if ros2Ready then
        if statusPub then
            simROS2.shutdownPublisher(statusPub)
        end

        if subMain then
            simROS2.shutdownSubscription(subMain)
        end

        if subR1 then
            simROS2.shutdownSubscription(subR1)
        end

        if subR2 then
            simROS2.shutdownSubscription(subR2)
        end

        if subR3 then
            simROS2.shutdownSubscription(subR3)
        end

        if subR4 then
            simROS2.shutdownSubscription(subR4)
        end

        if subR5 then
            simROS2.shutdownSubscription(subR5)
        end

        if subGlobal then
            simROS2.shutdownSubscription(subGlobal)
        end
    end
end
