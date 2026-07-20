sim = require('sim')

-- =========================================================
-- ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua
--
-- 修复点：
-- 上一版 createSubscription 用的是 local callback 函数。
-- 在 CoppeliaSim simROS2 中，订阅回调通常需要是脚本可见的全局函数名。
-- 如果回调是 local，publisher 能创建，所以能看到 /compact_cell/status；
-- 但 subscriber 可能创建失败，所以看不到 /compact_cell/cmd 等命令 topic。
--
-- 使用：
-- 1. 禁用/删除旧 ROS2_CompactCell_Bridge 脚本；
-- 2. 新建或替换 Dummy 脚本为本脚本；
-- 3. 从 ROS2 sourced 终端启动 CoppeliaSim；
-- 4. 开始仿真；
-- 5. ros2 topic list 应看到 /compact_cell/cmd、/compact_cell/tool_cmd 等。
-- =========================================================

local simROS2 = nil
local ros_ok = false
local subs = {}
local pub_status = nil

local mainMap = {
    RESET_CELL = {signal='cell_product_state', value='reset'},

    SHOW_ASSEMBLY_SHELL = {signal='cell_product_state', value='assembly_shell'},
    SHOW_ASSEMBLY_PCB = {signal='cell_product_state', value='assembly_pcb'},
    SHOW_ASSEMBLY_MODULE = {signal='cell_product_state', value='assembly_module'},
    SHOW_ASSEMBLY_FULL = {signal='cell_product_state', value='assembly_full'},
    SHOW_INSPECTION_FULL = {signal='cell_product_state', value='inspection_full'},

    CAMERA_GOOD = {signal='cell_product_state', value='camera_good'},
    CAMERA_DEFECT = {signal='cell_product_state', value='camera_defect'},

    CONVEYOR_GOOD = {signal='cell_conveyor_state', value='good'},
    CONVEYOR_DEFECT = {signal='cell_conveyor_state', value='defect'},

    R1_BOX_PLACED = {signal='cell_product_state', value='assembly_shell'},
    R2_PCB_PLACED = {signal='cell_product_state', value='assembly_pcb'},
    R3_MODULE_PLACED = {signal='cell_product_state', value='assembly_module'},
    R1_TERMINAL_PLACED = {signal='cell_product_state', value='assembly_full'},
    R3_PRODUCT_TO_INSPECTION = {signal='cell_product_state', value='inspection_full'},

    R4_SCREW_DONE = {signal='cell_screw_state', value='done'},

    R5_SORT_GOOD_DONE = {signal='cell_conveyor_state', value='good'},
    R5_SORT_DEFECT_DONE = {signal='cell_conveyor_state', value='defect'},
}

local toolCmdSet = {
    R1_GRIPPER_OPEN = true,
    R1_GRIPPER_CLOSE = true,
    R1_GRIPPER_CLOSE_BOX = true,
    R1_GRIPPER_CLOSE_TERMINAL = true,
    R1_GRIPPER_CLOSE_MODULE = true,
    R1_ATTACH_BOX = true,
    R1_RELEASE_BOX_ASSEMBLY = true,
    R1_ATTACH_MODULE = true,
    R1_RELEASE_MODULE_ASSEMBLY = true,
    R1_ATTACH_TERMINAL = true,
    R1_RELEASE_TERMINAL_ASSEMBLY = true,

    R2_VACUUM_ON = true,
    R2_VACUUM_OFF = true,
    R2_ATTACH_PCB = true,
    R2_RELEASE_PCB_ASSEMBLY = true,

    R3_GRIPPER_OPEN = true,
    R3_GRIPPER_CLOSE = true,
    R3_ATTACH_BOX = true,
    R3_RELEASE_BOX_ASSEMBLY = true,
    R3_ATTACH_MODULE = true,
    R3_RELEASE_MODULE_ASSEMBLY = true,
    R3_ATTACH_ASSEMBLY_PRODUCT = true,
    R3_RELEASE_PRODUCT_INSPECTION = true,

    R4_SCREW_START = true,
    R4_SCREW_STOP = true,

    R5_GRIPPER_OPEN = true,
    R5_GRIPPER_CLOSE = true,
    R5_ATTACH_INSPECTION_PRODUCT = true,
    R5_RELEASE_GOOD = true,
    R5_RELEASE_DEFECT = true,

    ALL_GRIPPERS_OPEN = true,
    ALL_GRIPPERS_CLOSE = true,
}

local function log(msg)
    print('[ROS2 BRIDGE V2] '..msg)
end

local function trim(s)
    if not s then return '' end
    s = string.gsub(s, '^%s+', '')
    s = string.gsub(s, '%s+$', '')
    return s
end

local function publishStatus(text)
    if ros_ok and pub_status then
        local ok,err = pcall(simROS2.publish, pub_status, {data=text})
        if not ok then
            print('[ROS2 BRIDGE V2 PUB ERROR] '..tostring(err))
        end
    end
    log(text)
end

local function sendMainCommand(cmd)
    local m = mainMap[cmd]
    if m then
        sim.setStringSignal(m.signal, m.value)
        publishStatus('MAIN_OK:'..cmd..' -> '..m.signal..'='..m.value)
        return true
    end
    return false
end

local function sendToolCommand(cmd)
    if toolCmdSet[cmd] then
        sim.setStringSignal('tool_cmd', cmd)
        publishStatus('TOOL_OK:'..cmd)
        return true
    end
    return false
end

function handleCompactCellCommand(cmd, source)
    cmd = trim(cmd)
    if cmd == '' then return end

    if sendToolCommand(cmd) then return end
    if sendMainCommand(cmd) then return end

    if cmd == 'R1_READY' or cmd == 'R2_READY' or cmd == 'R3_READY' or cmd == 'R4_READY' or cmd == 'R5_READY' then
        publishStatus('READY:'..cmd)
        return
    end

    publishStatus('UNKNOWN_FROM_'..source..':'..cmd)
end

-- 注意：这些回调函数必须是全局 function，不要写成 local function
function compactCell_cmd_cb(msg)
    handleCompactCellCommand(msg.data, 'cmd')
end

function compactCell_main_cb(msg)
    local cmd = trim(msg.data)
    if not sendMainCommand(cmd) then
        handleCompactCellCommand(cmd, 'main_cmd')
    end
end

function compactCell_tool_cb(msg)
    local cmd = trim(msg.data)
    if not sendToolCommand(cmd) then
        handleCompactCellCommand(cmd, 'tool_cmd')
    end
end

function compactCell_r1_cb(msg) handleCompactCellCommand(msg.data, 'r1_cmd') end
function compactCell_r2_cb(msg) handleCompactCellCommand(msg.data, 'r2_cmd') end
function compactCell_r3_cb(msg) handleCompactCellCommand(msg.data, 'r3_cmd') end
function compactCell_r4_cb(msg) handleCompactCellCommand(msg.data, 'r4_cmd') end
function compactCell_r5_cb(msg) handleCompactCellCommand(msg.data, 'r5_cmd') end

local function addSub(topic, msgType, cbName)
    local ok, subOrErr = pcall(simROS2.createSubscription, topic, msgType, cbName)

    if ok and subOrErr then
        table.insert(subs, subOrErr)
        log('SUB_OK: '..topic..' -> '..cbName)
    else
        log('SUB_FAILED: '..topic..' -> '..cbName..' err='..tostring(subOrErr))
    end
end

function sysCall_init()
    print('===== ROS2 Compact Cell Bridge V2 Global Callbacks =====')

    local ok, modOrErr = pcall(require, 'simROS2')
    if not ok then
        print('[ROS2 BRIDGE V2 ERROR] require("simROS2") failed: '..tostring(modOrErr))
        print('[ROS2 BRIDGE V2 ERROR] 必须从 source 过 ROS2 的终端启动 CoppeliaSim。')
        ros_ok = false
        return
    end

    simROS2 = modOrErr
    ros_ok = true

    local okPub, pubOrErr = pcall(simROS2.createPublisher, '/compact_cell/status', 'std_msgs/msg/String')
    if okPub and pubOrErr then
        pub_status = pubOrErr
        log('PUB_OK: /compact_cell/status')
    else
        log('PUB_FAILED: /compact_cell/status err='..tostring(pubOrErr))
    end

    addSub('/compact_cell/cmd', 'std_msgs/msg/String', 'compactCell_cmd_cb')
    addSub('/compact_cell/main_cmd', 'std_msgs/msg/String', 'compactCell_main_cb')
    addSub('/compact_cell/tool_cmd', 'std_msgs/msg/String', 'compactCell_tool_cb')
    addSub('/compact_cell/r1_cmd', 'std_msgs/msg/String', 'compactCell_r1_cb')
    addSub('/compact_cell/r2_cmd', 'std_msgs/msg/String', 'compactCell_r2_cb')
    addSub('/compact_cell/r3_cmd', 'std_msgs/msg/String', 'compactCell_r3_cb')
    addSub('/compact_cell/r4_cmd', 'std_msgs/msg/String', 'compactCell_r4_cb')
    addSub('/compact_cell/r5_cmd', 'std_msgs/msg/String', 'compactCell_r5_cb')

    publishStatus('ROS2_BRIDGE_V2_READY')
end

function sysCall_cleanup()
    if ros_ok and simROS2 then
        for _,s in ipairs(subs) do
            pcall(simROS2.shutdownSubscription, s)
        end
        if pub_status then
            pcall(simROS2.shutdownPublisher, pub_status)
        end
    end
end
