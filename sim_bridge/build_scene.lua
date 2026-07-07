sim = require('sim')

-- =========================================================
-- Compact Multi-CR5 Collaborative Cell with ROS2 interface
--
-- 功能：
-- 1. 生成双圆形减震工作台紧凑协作场景
-- 2. 黑色传送带，成品/缺陷品运行时循环运动
-- 3. 支持 R1 R2 R3 R4 四台外层对象自动移动到基座
-- 4. 支持 ROS2 发送 String 指令触发对应动作
-- 5. R3 相机为末端相机占位，不再是固定支架相机
--
-- ROS2 Topic:
--   Subscribe: /compact_cell/cmd      std_msgs/msg/String
--   Publish:   /compact_cell/status   std_msgs/msg/String
--
-- 可发送指令：
--   START
--   STOP
--   R1_FEED
--   R2_ASSEMBLE
--   R3_INSPECT
--   R3_SCREW
--   R4_SORT_GOOD
--   R4_SORT_DEFECT
--   AUTO_CYCLE
-- =========================================================

-- =========================================================
-- 配置区
-- =========================================================

-- 第一次生成场景时设为 true。
-- 你已经导入并摆好 R1-R4 后，建议改成 false，避免重建场景。
local REBUILD_SCENE_ON_START = true

-- 是否自动把外层对象 /R1 /R2 /R3 /R4 移到对应基座
local AUTO_PLACE_EXISTING_ROBOTS = true

-- 基座中心 z = 0.17，高度 0.10，上表面大约 z = 0.22
-- zOffset = 0.05 表示机器人根节点放到 0.22
-- 如果机械臂悬空或陷入基座，调这个值
local ROBOT_Z_OFFSET = 0.05

-- 是否尝试把 R3 末端相机自动挂到 R3 的 tip/flange 下
local AUTO_ATTACH_R3_CAMERA = true

-- 是否启用 ROS2 接口
local ENABLE_ROS2 = true

-- =========================================================
-- 全局变量
-- =========================================================

local root
local groups = {}

local goodProduct
local defectProduct

local goodStart = {1.35, -1.05, 0.36}
local goodEnd   = {1.35, -3.25, 0.36}

local defectStart = {2.15, -0.55, 0.36}
local defectEnd   = {3.75, -0.55, 0.36}

local conveyorSpeed = 0.28
local conveyorEnabled = true

-- 螺丝托盘位置：远离 R3_Base
local screwTrayX = 2.05
local screwTrayY = 0.70

-- ROS2
local simROS2 = nil
local ros2Available = false
local cmdSubscriber = nil
local statusPublisher = nil

-- 任务状态
local currentCommand = 'IDLE'
local activeTask = nil
local commandQueue = {}

-- =========================================================
-- 基础工具函数
-- =========================================================

local function safeGet(path)
    local ok, h = pcall(sim.getObject, path)
    if ok then
        return h
    else
        return -1
    end
end

local function setColor(obj, color)
    sim.setShapeColor(obj, nil, sim.colorcomponent_ambient_diffuse, color)
end

local function makeGroup(name, parent)
    local h = sim.createDummy(0.03)
    sim.setObjectAlias(h, name)
    if parent then
        sim.setObjectParent(h, parent, true)
    end
    return h
end

local function cuboid(name, pos, size, color, parent)
    local obj = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(obj, name)
    sim.setObjectPosition(obj, pos)
    setColor(obj, color)
    if parent then
        sim.setObjectParent(obj, parent, true)
    end
    return obj
end

local function cylinder(name, pos, radius, height, color, parent)
    local obj = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {radius * 2, radius * 2, height}, 0)
    sim.setObjectAlias(obj, name)
    sim.setObjectPosition(obj, pos)
    setColor(obj, color)
    if parent then
        sim.setObjectParent(obj, parent, true)
    end
    return obj
end

local function dummy(name, pos, parent)
    local obj = sim.createDummy(0.06)
    sim.setObjectAlias(obj, name)
    sim.setObjectPosition(obj, pos)
    if parent then
        sim.setObjectParent(obj, parent, true)
    end
    return obj
end

local function publishStatus(text)
    print('[STATUS] ' .. text)

    if ros2Available and statusPublisher then
        local ok = pcall(function()
            simROS2.publish(statusPublisher, {data = text})
        end)

        if not ok then
            print('Failed to publish ROS2 status: ' .. text)
        end
    end
end

local function removeTree(h)
    if h == -1 then
        return
    end

    local ok, objs = pcall(sim.getObjectsInTree, h, sim.handle_all, 0)

    if ok and objs and #objs > 0 then
        sim.removeObjects(objs)
    else
        sim.removeObjects({h})
    end
end

local function getPos(path)
    local h = safeGet(path)
    if h == -1 then
        print('Cannot find target point: ' .. path)
        return {0, 0, 0}
    end
    return sim.getObjectPosition(h)
end

-- =========================================================
-- 场景物体创建函数
-- =========================================================

local function makeOpenBox(prefix, x, y, z, parent, color)
    local boxRoot = makeGroup(prefix, parent)
    sim.setObjectPosition(boxRoot, {x, y, z})

    cuboid(prefix .. '_Bottom', {x, y, z + 0.02}, {0.45, 0.30, 0.04}, color, boxRoot)
    cuboid(prefix .. '_Left_Wall',  {x - 0.225, y, z + 0.13}, {0.04, 0.30, 0.22}, color, boxRoot)
    cuboid(prefix .. '_Right_Wall', {x + 0.225, y, z + 0.13}, {0.04, 0.30, 0.22}, color, boxRoot)
    cuboid(prefix .. '_Front_Wall', {x, y - 0.150, z + 0.13}, {0.45, 0.04, 0.22}, color, boxRoot)
    cuboid(prefix .. '_Back_Wall',  {x, y + 0.150, z + 0.13}, {0.45, 0.04, 0.22}, color, boxRoot)

    return boxRoot
end

local function makePCB(prefix, x, y, z, parent)
    local pcbColor = {0.00, 0.45, 0.18}
    local chipColor = {0.02, 0.02, 0.02}
    local blueColor = {0.00, 0.20, 0.80}
    local whiteColor = {0.90, 0.90, 0.85}

    local pcbRoot = makeGroup(prefix, parent)
    sim.setObjectPosition(pcbRoot, {x, y, z})

    cuboid(prefix .. '_Board', {x, y, z}, {0.32, 0.20, 0.015}, pcbColor, pcbRoot)
    cuboid(prefix .. '_Chip_1', {x - 0.06, y, z + 0.025}, {0.07, 0.07, 0.025}, chipColor, pcbRoot)
    cuboid(prefix .. '_Chip_2', {x + 0.07, y + 0.04, z + 0.025}, {0.05, 0.04, 0.02}, chipColor, pcbRoot)
    cuboid(prefix .. '_Connector', {x, y - 0.085, z + 0.025}, {0.14, 0.03, 0.03}, whiteColor, pcbRoot)

    cylinder(prefix .. '_Capacitor_1', {x - 0.12, y + 0.06, z + 0.035}, 0.012, 0.04, blueColor, pcbRoot)
    cylinder(prefix .. '_Capacitor_2', {x + 0.12, y + 0.06, z + 0.035}, 0.012, 0.04, blueColor, pcbRoot)

    return pcbRoot
end

local function makeConveyor(prefix, center, length, width, direction, parent)
    local metalColor = {0.45, 0.45, 0.45}
    local blackColor = {0.01, 0.01, 0.01}
    local rollerColor = {0.25, 0.25, 0.25}

    local convRoot = makeGroup(prefix, parent)
    sim.setObjectPosition(convRoot, center)

    local x = center[1]
    local y = center[2]
    local z = center[3]

    if direction == 'Y' then
        cuboid(prefix .. '_Frame', {x, y, z}, {width + 0.12, length, 0.16}, metalColor, convRoot)
        cuboid(prefix .. '_Belt_Black', {x, y, z + 0.10}, {width, length - 0.10, 0.035}, blackColor, convRoot)

        local r1 = cylinder(prefix .. '_Roller_1', {x, y - length / 2 + 0.10, z + 0.12}, 0.06, width, rollerColor, convRoot)
        sim.setObjectOrientation(r1, {0, math.pi / 2, 0})

        local r2 = cylinder(prefix .. '_Roller_2', {x, y + length / 2 - 0.10, z + 0.12}, 0.06, width, rollerColor, convRoot)
        sim.setObjectOrientation(r2, {0, math.pi / 2, 0})

        cuboid(prefix .. '_SideRail_Left', {x - width / 2 - 0.05, y, z + 0.18}, {0.04, length, 0.08}, metalColor, convRoot)
        cuboid(prefix .. '_SideRail_Right', {x + width / 2 + 0.05, y, z + 0.18}, {0.04, length, 0.08}, metalColor, convRoot)

    elseif direction == 'X' then
        cuboid(prefix .. '_Frame', {x, y, z}, {length, width + 0.12, 0.16}, metalColor, convRoot)
        cuboid(prefix .. '_Belt_Black', {x, y, z + 0.10}, {length - 0.10, width, 0.035}, blackColor, convRoot)

        local r1 = cylinder(prefix .. '_Roller_1', {x - length / 2 + 0.10, y, z + 0.12}, 0.06, width, rollerColor, convRoot)
        sim.setObjectOrientation(r1, {math.pi / 2, 0, 0})

        local r2 = cylinder(prefix .. '_Roller_2', {x + length / 2 - 0.10, y, z + 0.12}, 0.06, width, rollerColor, convRoot)
        sim.setObjectOrientation(r2, {math.pi / 2, 0, 0})

        cuboid(prefix .. '_SideRail_Left', {x, y - width / 2 - 0.05, z + 0.18}, {length, 0.04, 0.08}, metalColor, convRoot)
        cuboid(prefix .. '_SideRail_Right', {x, y + width / 2 + 0.05, z + 0.18}, {length, 0.04, 0.08}, metalColor, convRoot)
    end

    return convRoot
end

local function createRubberPads(tablePrefix, cx, cy, radius, parent)
    local rubberColor = {0.02, 0.02, 0.02}

    for i = 1, 8 do
        local angle = (i - 1) * 2 * math.pi / 8
        local x = cx + radius * math.cos(angle)
        local y = cy + radius * math.sin(angle)
        cylinder(tablePrefix .. '_RubberPad_' .. i, {x, y, 0.03}, 0.07, 0.06, rubberColor, parent)
    end
end

local function makeEndEffectorCamera(parent)
    local darkColor = {0.02, 0.02, 0.02}
    local lensColor = {0.01, 0.01, 0.01}
    local metalColor = {0.45, 0.45, 0.45}

    local camRoot = makeGroup('R3_End_Effector_Camera', parent)

    -- 如果没有找到 R3/tip，它会先显示在检测区上方
    sim.setObjectPosition(camRoot, {0.45, 0.10, 0.82})

    cuboid('R3_EE_Camera_Mount', {0.45, 0.10, 0.86}, {0.12, 0.08, 0.025}, metalColor, camRoot)
    cuboid('R3_EE_Camera_Body', {0.45, 0.10, 0.82}, {0.14, 0.10, 0.07}, darkColor, camRoot)

    local lens = cylinder('R3_EE_Camera_Lens', {0.45, 0.10, 0.76}, 0.032, 0.06, lensColor, camRoot)
    sim.setObjectOrientation(lens, {math.pi / 2, 0, 0})

    return camRoot
end

-- =========================================================
-- R1-R4 机械臂放置函数
-- =========================================================

local function moveRobotToBase(robotPath, basePath, yawDeg)
    local robot = safeGet(robotPath)
    local base = safeGet(basePath)

    if robot == -1 then
        print('Robot not found: ' .. robotPath)
        return
    end

    if base == -1 then
        print('Base not found: ' .. basePath)
        return
    end

    local basePos = sim.getObjectPosition(base)
    local targetPos = {
        basePos[1],
        basePos[2],
        basePos[3] + ROBOT_Z_OFFSET
    }

    sim.setObjectPosition(robot, targetPos)
    sim.setObjectOrientation(robot, {0, 0, math.rad(yawDeg)})

    print(robotPath .. ' moved to ' .. basePath)
end

local function placeExistingRobotsOnBases()
    if not AUTO_PLACE_EXISTING_ROBOTS then
        return
    end

    moveRobotToBase('/R1', '/CompactCell/RobotBases/R1_Base', -50)
    moveRobotToBase('/R2', '/CompactCell/RobotBases/R2_Base', 0)
    moveRobotToBase('/R3', '/CompactCell/RobotBases/R3_Base', 210)
    moveRobotToBase('/R4', '/CompactCell/RobotBases/R4_Base', 140)
end

-- =========================================================
-- R3 末端相机挂载
-- =========================================================

local function tryAttachCameraToR3Tip()
    if not AUTO_ATTACH_R3_CAMERA then
        return
    end

    local cam = safeGet('/CompactCell/Sensors/R3_End_Effector_Camera')
    if cam == -1 then
        print('R3_End_Effector_Camera not found. Skip attaching.')
        return
    end

    local candidateTips = {
        '/R3/tip',
        '/R3/Tip',
        '/R3/flange',
        '/R3/Flange',
        '/R3/TCP',
        '/R3/tcp',
        '/CR5_R3/tip',
        '/CR5_R3/Tip',
        '/CR5_R3/flange',
        '/CR5_R3/Flange',
        '/CompactCell/Robots/R3/tip',
        '/CompactCell/Robots/R3/Tip'
    }

    local tip = -1
    local tipPath = ''

    for i = 1, #candidateTips do
        local h = safeGet(candidateTips[i])
        if h ~= -1 then
            tip = h
            tipPath = candidateTips[i]
            break
        end
    end

    if tip == -1 then
        print('R3 tip/flange not found.')
        print('R3_End_Effector_Camera remains above inspection area as placeholder.')
        print('You can rename R3 end-effector dummy to /R3/tip.')
        return
    end

    sim.setObjectParent(cam, tip, false)
    sim.setObjectPosition(cam, {0.00, 0.00, 0.08}, tip)
    sim.setObjectOrientation(cam, {0.00, 0.00, 0.00}, tip)

    print('R3 end-effector camera attached to: ' .. tipPath)
end

-- =========================================================
-- 场景创建
-- =========================================================

local function createScene()
    groups = {}

    local groundColor = {0.78, 0.78, 0.78}
    local tableColor = {0.62, 0.73, 0.36}
    local areaColor = {0.86, 0.86, 0.82}
    local metalColor = {0.55, 0.55, 0.55}
    local darkColor = {0.02, 0.02, 0.02}
    local boxColor = {0.65, 0.45, 0.25}
    local goodColor = {0.30, 0.65, 0.30}
    local defectColor = {0.85, 0.20, 0.10}
    local yellowColor = {0.90, 0.75, 0.20}
    local blueColor = {0.10, 0.30, 0.85}

    root = makeGroup('CompactCell', nil)

    groups.Ground = makeGroup('Ground_Group', root)
    groups.Tables = makeGroup('Tables', root)
    groups.Areas = makeGroup('Areas', root)
    groups.RobotBases = makeGroup('RobotBases', root)
    groups.Robots = makeGroup('Robots', root)
    groups.Parts = makeGroup('Parts', root)
    groups.Conveyors = makeGroup('Conveyors', root)
    groups.Targets = makeGroup('Targets', root)
    groups.Sensors = makeGroup('Sensors', root)

    -- 地面
    cuboid('Ground', {0.2, -0.35, -0.01}, {7.5, 5.2, 0.02}, groundColor, groups.Ground)

    -- 双圆形减震工作台
    -- 半径 1.30，避免两个工作台重合
    local tableRadius = 1.30
    local rubberPadRadius = 1.05

    cylinder('Damping_Table_Left', {-1.45, 0.45, 0.06}, tableRadius, 0.12, tableColor, groups.Tables)
    createRubberPads('LeftTable', -1.45, 0.45, rubberPadRadius, groups.Tables)

    cylinder('Damping_Table_Right', {1.20, -0.05, 0.06}, tableRadius, 0.12, tableColor, groups.Tables)
    createRubberPads('RightTable', 1.20, -0.05, rubberPadRadius, groups.Tables)

    -- R1-R4 基座
    cylinder('R1_Base', {-1.75, 0.95, 0.17}, 0.22, 0.10, metalColor, groups.RobotBases)
    cylinder('R2_Base', {-1.75, -0.30, 0.17}, 0.22, 0.10, metalColor, groups.RobotBases)
    cylinder('R3_Base', {1.05, 0.40, 0.17}, 0.22, 0.10, metalColor, groups.RobotBases)
    cylinder('R4_Base', {1.30, -0.65, 0.17}, 0.22, 0.10, metalColor, groups.RobotBases)

    -- 左侧功能区
    cuboid('Box_Area', {-2.55, 0.55, 0.15}, {0.55, 0.40, 0.04}, areaColor, groups.Areas)
    cuboid('Electronics_Area', {-1.10, 0.65, 0.15}, {0.60, 0.45, 0.04}, areaColor, groups.Areas)
    cuboid('Assembly_Area', {-0.75, -0.35, 0.15}, {0.70, 0.50, 0.04}, areaColor, groups.Areas)
    cuboid('Assembly_Fixture', {-0.75, -0.35, 0.20}, {0.52, 0.34, 0.06}, metalColor, groups.Areas)

    makeOpenBox('Box_Blank_1', -2.65, 0.55, 0.17, groups.Parts, boxColor)
    makeOpenBox('Box_Blank_2', -2.35, 0.55, 0.17, groups.Parts, boxColor)
    makeOpenBox('Assembly_Box', -0.75, -0.35, 0.24, groups.Parts, boxColor)

    makePCB('PCB_1', -1.25, 0.65, 0.20, groups.Parts)
    makePCB('PCB_2', -0.90, 0.65, 0.20, groups.Parts)

    cuboid('Component_Block_1', {-1.28, 0.90, 0.20}, {0.12, 0.08, 0.04}, blueColor, groups.Parts)
    cuboid('Component_Block_2', {-1.05, 0.90, 0.20}, {0.12, 0.08, 0.04}, blueColor, groups.Parts)
    cuboid('Component_Block_3', {-0.82, 0.90, 0.20}, {0.12, 0.08, 0.04}, blueColor, groups.Parts)

    -- 右侧检测与锁付区
    cuboid('Inspection_Screw_Area', {0.45, 0.10, 0.15}, {0.75, 0.55, 0.04}, areaColor, groups.Areas)
    cuboid('Inspection_Platform', {0.45, 0.10, 0.20}, {0.52, 0.36, 0.06}, metalColor, groups.Areas)

    -- 修复后的检测区产品
    makeOpenBox('Inspection_Box', 0.45, 0.10, 0.24, groups.Parts, boxColor)

    cylinder('Screw_Hole_1', {0.27, -0.02, 0.49}, 0.018, 0.012, darkColor, groups.Parts)
    cylinder('Screw_Hole_2', {0.63, -0.02, 0.49}, 0.018, 0.012, darkColor, groups.Parts)
    cylinder('Screw_Hole_3', {0.27, 0.22, 0.49}, 0.018, 0.012, darkColor, groups.Parts)
    cylinder('Screw_Hole_4', {0.63, 0.22, 0.49}, 0.018, 0.012, darkColor, groups.Parts)

    -- R3 末端相机占位
    makeEndEffectorCamera(groups.Sensors)
    dummy('R3_Camera_Inspect_Target', {0.45, 0.10, 0.60}, groups.Targets)

    -- 螺丝托盘，远离 R3 基座
    cuboid('Screw_Tray', {screwTrayX, screwTrayY, 0.15}, {0.42, 0.25, 0.04}, metalColor, groups.Areas)

    local idx = 1
    for i = 0, 3 do
        for j = 0, 1 do
            local sx = screwTrayX - 0.15 + i * 0.10
            local sy = screwTrayY - 0.06 + j * 0.12
            cylinder('Screw_' .. idx, {sx, sy, 0.21}, 0.012, 0.035, metalColor, groups.Parts)
            idx = idx + 1
        end
    end

    local screwdriver = cylinder('Screwdriver_Tool', {screwTrayX + 0.32, screwTrayY, 0.23}, 0.022, 0.35, darkColor, groups.Parts)
    sim.setObjectOrientation(screwdriver, {0, math.pi / 2, 0})

    -- 黑色传送带
    makeConveyor('Conveyor_Good', {1.35, -2.15, 0.18}, 2.50, 0.55, 'Y', groups.Conveyors)
    makeConveyor('Conveyor_Defect', {2.95, -0.55, 0.18}, 1.90, 0.55, 'X', groups.Conveyors)

    goodProduct = makeOpenBox('Good_Product_Moving', goodStart[1], goodStart[2], goodStart[3], groups.Parts, goodColor)
    defectProduct = makeOpenBox('Defect_Product_Moving', defectStart[1], defectStart[2], defectStart[3], groups.Parts, defectColor)

    cuboid('Good_Label_Block', {goodStart[1], goodStart[2], goodStart[3] + 0.30}, {0.18, 0.08, 0.025}, yellowColor, goodProduct)
    cuboid('Defect_Label_Block', {defectStart[1], defectStart[2], defectStart[3] + 0.30}, {0.18, 0.08, 0.025}, defectColor, defectProduct)

    -- 中间交接区
    cuboid('Transfer_Area_Between_Tables', {-0.10, -0.12, 0.14}, {0.45, 0.35, 0.035}, {0.90, 0.82, 0.55}, groups.Areas)
    dummy('Transfer_Target', {-0.10, -0.12, 0.55}, groups.Targets)

    -- =====================================================
    -- 目标点 Dummy
    -- =====================================================

    dummy('R1_Home', {-1.75, 0.95, 0.90}, groups.Targets)
    dummy('R1_Pick_Box_Approach', {-2.55, 0.55, 0.75}, groups.Targets)
    dummy('R1_Pick_Box_Target', {-2.55, 0.55, 0.45}, groups.Targets)
    dummy('R1_Place_Box_Approach', {-0.75, -0.35, 0.80}, groups.Targets)
    dummy('R1_Place_Box_Target', {-0.75, -0.35, 0.50}, groups.Targets)

    dummy('R2_Home', {-1.75, -0.30, 0.90}, groups.Targets)
    dummy('R2_Pick_PCB_Approach', {-1.25, 0.65, 0.75}, groups.Targets)
    dummy('R2_Pick_PCB_Target', {-1.25, 0.65, 0.42}, groups.Targets)
    dummy('R2_Install_PCB_Approach', {-0.75, -0.35, 0.80}, groups.Targets)
    dummy('R2_Install_PCB_Target', {-0.75, -0.35, 0.52}, groups.Targets)

    dummy('R3_Home', {1.05, 0.40, 0.90}, groups.Targets)
    dummy('R3_Inspect_Approach', {0.45, 0.10, 0.85}, groups.Targets)
    dummy('R3_Inspect_Target', {0.45, 0.10, 0.60}, groups.Targets)
    dummy('R3_Pick_Screw_Approach', {screwTrayX, screwTrayY, 0.70}, groups.Targets)
    dummy('R3_Pick_Screw_Target', {screwTrayX - 0.15, screwTrayY - 0.06, 0.28}, groups.Targets)
    dummy('R3_Screw_Point_1', {0.27, -0.02, 0.62}, groups.Targets)
    dummy('R3_Screw_Point_2', {0.63, -0.02, 0.62}, groups.Targets)
    dummy('R3_Screw_Point_3', {0.27, 0.22, 0.62}, groups.Targets)
    dummy('R3_Screw_Point_4', {0.63, 0.22, 0.62}, groups.Targets)

    dummy('R4_Home', {1.30, -0.65, 0.90}, groups.Targets)
    dummy('R4_Pick_Product_Approach', {0.45, 0.10, 0.85}, groups.Targets)
    dummy('R4_Pick_Product_Target', {0.45, 0.10, 0.52}, groups.Targets)
    dummy('R4_Place_Good_Approach', {1.35, -1.05, 0.75}, groups.Targets)
    dummy('R4_Place_Good_Target', {1.35, -1.05, 0.48}, groups.Targets)
    dummy('R4_Place_Defect_Approach', {2.15, -0.55, 0.75}, groups.Targets)
    dummy('R4_Place_Defect_Target', {2.15, -0.55, 0.48}, groups.Targets)

    -- 控制目标 fallback。
    -- 如果你的 CR5 已经有 IK target，脚本会优先移动真实 target；
    -- 如果没有，就移动这些可视化 target。
    dummy('R1_Motion_Target', {-1.75, 0.95, 0.90}, groups.Targets)
    dummy('R2_Motion_Target', {-1.75, -0.30, 0.90}, groups.Targets)
    dummy('R3_Motion_Target', {1.05, 0.40, 0.90}, groups.Targets)
    dummy('R4_Motion_Target', {1.30, -0.65, 0.90}, groups.Targets)

    -- 标准点位 P_*，供其他脚本查找
    dummy('P_FEED_01', {-2.55, 0.55, 0.45}, groups.Targets)
    dummy('P_ASSEMBLY_01', {-1.25, 0.65, 0.42}, groups.Targets)
    dummy('P_ASSEMBLY_02', {-0.75, -0.35, 0.52}, groups.Targets)
    dummy('P_SCREW_01', {screwTrayX - 0.15, screwTrayY - 0.06, 0.28}, groups.Targets)
    dummy('P_SCREW_02', {0.45, 0.10, 0.62}, groups.Targets)
    dummy('P_INSPECT_01', {0.45, 0.10, 0.60}, groups.Targets)
    dummy('P_UNLOAD_01', {0.45, 0.10, 0.52}, groups.Targets)
    dummy('P_GOOD_01', {1.35, -1.05, 0.48}, groups.Targets)
    dummy('P_DEFECT_01', {2.15, -0.55, 0.48}, groups.Targets)
    dummy('P_REWORK_01', {3.75, -0.55, 0.48}, groups.Targets)
    dummy('P_CONVEYOR_START', {1.35, -1.05, 0.36}, groups.Targets)
    dummy('P_CONVEYOR_END', {1.35, -3.25, 0.36}, groups.Targets)
    dummy('P_SHARED_TRANSFER', {-0.10, -0.12, 0.55}, groups.Targets)

    print('CompactCell created successfully.')
    print('Black conveyors created.')
    print('Screw_Tray moved away from R3_Base.')
    print('R3 end-effector camera placeholder created.')
end

local function loadExistingHandles()
    root = safeGet('/CompactCell')

    groups.Parts = safeGet('/CompactCell/Parts')
    groups.Targets = safeGet('/CompactCell/Targets')
    groups.Sensors = safeGet('/CompactCell/Sensors')
    groups.RobotBases = safeGet('/CompactCell/RobotBases')

    goodProduct = safeGet('/CompactCell/Parts/Good_Product_Moving')
    defectProduct = safeGet('/CompactCell/Parts/Defect_Product_Moving')
end

-- =========================================================
-- 传送带产品运动
-- =========================================================

local function moveAlongLine(obj, p0, p1, speed, phase)
    if obj == nil or obj == -1 then
        return
    end

    local t = sim.getSimulationTime() + phase

    local dx = p1[1] - p0[1]
    local dy = p1[2] - p0[2]
    local dz = p1[3] - p0[3]

    local dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    local moveTime = dist / speed

    if moveTime <= 0.0001 then
        return
    end

    local pauseTime = 0.8
    local cycleTime = moveTime + pauseTime

    local tau = t % cycleTime
    local a = tau / moveTime

    if a > 1 then
        a = 1
    end

    local x = p0[1] + dx * a
    local y = p0[2] + dy * a
    local z = p0[3] + dz * a

    sim.setObjectPosition(obj, {x, y, z})
end

-- =========================================================
-- 机器人 target 查找与任务运动
-- =========================================================

local function getRobotControlTarget(robotName)
    local candidates = {
        '/' .. robotName .. '/target',
        '/' .. robotName .. '/Target',
        '/' .. robotName .. '/ikTarget',
        '/' .. robotName .. '/IK_Target',
        '/' .. robotName .. '/tipTarget',
        '/' .. robotName .. '/Tip_Target',
        '/CompactCell/Targets/' .. robotName .. '_Motion_Target'
    }

    for i = 1, #candidates do
        local h = safeGet(candidates[i])
        if h ~= -1 then
            return h, candidates[i]
        end
    end

    return -1, ''
end

local function buildTask(taskName, robotName, waypointNames, partPath, attachIndex, detachIndex)
    local points = {}

    for i = 1, #waypointNames do
        local path = '/CompactCell/Targets/' .. waypointNames[i]
        points[#points + 1] = getPos(path)
    end

    local target, targetPath = getRobotControlTarget(robotName)

    if target == -1 then
        print('No control target found for ' .. robotName)
        return nil
    end

    print(taskName .. ' will move target: ' .. targetPath)

    local part = -1
    if partPath then
        part = safeGet(partPath)
        if part == -1 then
            print('Task part not found: ' .. partPath)
        end
    end

    local segmentTime = 0.9

    return {
        name = taskName,
        robotName = robotName,
        target = target,
        points = points,
        startTime = sim.getSimulationTime(),
        segmentTime = segmentTime,
        part = part,
        attachTime = attachIndex and ((attachIndex - 1) * segmentTime) or nil,
        detachTime = detachIndex and ((detachIndex - 1) * segmentTime) or nil,
        attached = false,
        detached = false
    }
end

local function startTask(taskName)
    local task = nil

    if taskName == 'R1_FEED' then
        task = buildTask(
            taskName,
            'R1',
            {
                'R1_Home',
                'R1_Pick_Box_Approach',
                'R1_Pick_Box_Target',
                'R1_Pick_Box_Approach',
                'R1_Place_Box_Approach',
                'R1_Place_Box_Target',
                'R1_Place_Box_Approach',
                'R1_Home'
            },
            '/CompactCell/Parts/Box_Blank_1',
            3,
            6
        )

    elseif taskName == 'R2_ASSEMBLE' then
        task = buildTask(
            taskName,
            'R2',
            {
                'R2_Home',
                'R2_Pick_PCB_Approach',
                'R2_Pick_PCB_Target',
                'R2_Pick_PCB_Approach',
                'R2_Install_PCB_Approach',
                'R2_Install_PCB_Target',
                'R2_Install_PCB_Approach',
                'R2_Home'
            },
            '/CompactCell/Parts/PCB_1',
            3,
            6
        )

    elseif taskName == 'R3_INSPECT' then
        task = buildTask(
            taskName,
            'R3',
            {
                'R3_Home',
                'R3_Inspect_Approach',
                'R3_Inspect_Target',
                'R3_Inspect_Approach',
                'R3_Home'
            },
            nil,
            nil,
            nil
        )

    elseif taskName == 'R3_SCREW' then
        task = buildTask(
            taskName,
            'R3',
            {
                'R3_Home',
                'R3_Pick_Screw_Approach',
                'R3_Pick_Screw_Target',
                'R3_Pick_Screw_Approach',
                'R3_Screw_Point_1',
                'R3_Screw_Point_2',
                'R3_Screw_Point_3',
                'R3_Screw_Point_4',
                'R3_Inspect_Approach',
                'R3_Home'
            },
            nil,
            nil,
            nil
        )

    elseif taskName == 'R4_SORT_GOOD' then
        task = buildTask(
            taskName,
            'R4',
            {
                'R4_Home',
                'R4_Pick_Product_Approach',
                'R4_Pick_Product_Target',
                'R4_Pick_Product_Approach',
                'R4_Place_Good_Approach',
                'R4_Place_Good_Target',
                'R4_Place_Good_Approach',
                'R4_Home'
            },
            '/CompactCell/Parts/Inspection_Box',
            3,
            6
        )

    elseif taskName == 'R4_SORT_DEFECT' then
        task = buildTask(
            taskName,
            'R4',
            {
                'R4_Home',
                'R4_Pick_Product_Approach',
                'R4_Pick_Product_Target',
                'R4_Pick_Product_Approach',
                'R4_Place_Defect_Approach',
                'R4_Place_Defect_Target',
                'R4_Place_Defect_Approach',
                'R4_Home'
            },
            '/CompactCell/Parts/Inspection_Box',
            3,
            6
        )
    end

    if task then
        activeTask = task
        publishStatus(taskName .. '_STARTED')
    else
        print('Unknown task or task build failed: ' .. taskName)
        publishStatus(taskName .. '_FAILED')
    end
end

local function updateActiveTask()
    if not activeTask then
        return
    end

    local t = sim.getSimulationTime() - activeTask.startTime
    local n = #activeTask.points

    if n < 2 then
        publishStatus(activeTask.name .. '_DONE')
        activeTask = nil
        return
    end

    local totalTime = (n - 1) * activeTask.segmentTime

    -- 取件：把物体挂到运动 target 下
    if activeTask.part ~= -1 and activeTask.attachTime and not activeTask.attached and t >= activeTask.attachTime then
        sim.setObjectParent(activeTask.part, activeTask.target, true)
        activeTask.attached = true
        print(activeTask.name .. ': part attached.')
    end

    -- 放件：把物体放回 Parts 分组
    if activeTask.part ~= -1 and activeTask.detachTime and not activeTask.detached and t >= activeTask.detachTime then
        local partsParent = safeGet('/CompactCell/Parts')
        if partsParent ~= -1 then
            sim.setObjectParent(activeTask.part, partsParent, true)
        else
            sim.setObjectParent(activeTask.part, -1, true)
        end
        activeTask.detached = true
        print(activeTask.name .. ': part detached.')
    end

    if t >= totalTime then
        local finalPos = activeTask.points[n]
        sim.setObjectPosition(activeTask.target, finalPos)

        local doneName = activeTask.name .. '_DONE'
        publishStatus(doneName)

        activeTask = nil

        if #commandQueue > 0 then
            local nextCmd = table.remove(commandQueue, 1)
            startTask(nextCmd)
        end

        return
    end

    local seg = math.floor(t / activeTask.segmentTime) + 1
    if seg < 1 then seg = 1 end
    if seg > n - 1 then seg = n - 1 end

    local localT = (t - (seg - 1) * activeTask.segmentTime) / activeTask.segmentTime

    local p0 = activeTask.points[seg]
    local p1 = activeTask.points[seg + 1]

    local x = p0[1] + (p1[1] - p0[1]) * localT
    local y = p0[2] + (p1[2] - p0[2]) * localT
    local z = p0[3] + (p1[3] - p0[3]) * localT

    sim.setObjectPosition(activeTask.target, {x, y, z})
end

-- =========================================================
-- ROS2 指令处理
-- =========================================================

local function handleCommand(cmd)
    currentCommand = cmd

    print('[CMD] ' .. cmd)

    if cmd == 'START' then
        conveyorEnabled = true
        publishStatus('CONVEYOR_STARTED')

    elseif cmd == 'STOP' then
        conveyorEnabled = false
        activeTask = nil
        commandQueue = {}
        publishStatus('STOPPED')

    elseif cmd == 'AUTO_CYCLE' then
        commandQueue = {
            'R2_ASSEMBLE',
            'R3_INSPECT',
            'R3_SCREW',
            'R4_SORT_GOOD'
        }
        startTask('R1_FEED')

    elseif cmd == 'R1_FEED'
        or cmd == 'R2_ASSEMBLE'
        or cmd == 'R3_INSPECT'
        or cmd == 'R3_SCREW'
        or cmd == 'R4_SORT_GOOD'
        or cmd == 'R4_SORT_DEFECT' then

        if activeTask then
            print('Task is running. New command queued: ' .. cmd)
            commandQueue[#commandQueue + 1] = cmd
            publishStatus(cmd .. '_QUEUED')
        else
            startTask(cmd)
        end

    else
        print('Unknown command: ' .. cmd)
        publishStatus('UNKNOWN_COMMAND_' .. cmd)
    end
end

function compactCellCommandCallback(msg)
    if msg and msg.data then
        handleCommand(msg.data)
    end
end

local function setupROS2()
    if not ENABLE_ROS2 then
        print('ROS2 disabled by configuration.')
        return
    end

    local ok, ros2 = pcall(require, 'simROS2')

    if not ok then
        ros2Available = false
        print('simROS2 not available.')
        print('Please source ROS2 environment before starting CoppeliaSim.')
        return
    end

    simROS2 = ros2
    ros2Available = true

    cmdSubscriber = simROS2.createSubscription(
        '/compact_cell/cmd',
        'std_msgs/msg/String',
        'compactCellCommandCallback'
    )

    statusPublisher = simROS2.createPublisher(
        '/compact_cell/status',
        'std_msgs/msg/String'
    )

    print('ROS2 interface loaded.')
    print('Subscribe: /compact_cell/cmd')
    print('Publish:   /compact_cell/status')
end

-- =========================================================
-- CoppeliaSim 回调函数
-- =========================================================

function sysCall_init()
    local existing = safeGet('/CompactCell')

    if existing ~= -1 and REBUILD_SCENE_ON_START then
        print('Old /CompactCell found. Removing and rebuilding...')
        removeTree(existing)
        existing = -1
    end

    if existing == -1 then
        createScene()
    else
        print('/CompactCell already exists. Scene creation skipped.')
        loadExistingHandles()
    end

    loadExistingHandles()
    placeExistingRobotsOnBases()
    tryAttachCameraToR3Tip()
    setupROS2()

    publishStatus('COMPACT_CELL_READY')
end

function sysCall_actuation()
    if conveyorEnabled then
        moveAlongLine(goodProduct, goodStart, goodEnd, conveyorSpeed, 0.0)
        moveAlongLine(defectProduct, defectStart, defectEnd, conveyorSpeed, 1.2)
    end

    updateActiveTask()
end

function sysCall_cleanup()
    if ros2Available then
        if cmdSubscriber then
            pcall(function()
                simROS2.shutdownSubscription(cmdSubscriber)
            end)
        end

        if statusPublisher then
            pcall(function()
                simROS2.shutdownPublisher(statusPublisher)
            end)
        end
    end
end
