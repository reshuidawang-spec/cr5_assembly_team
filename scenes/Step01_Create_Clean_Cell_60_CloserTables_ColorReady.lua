sim = require('sim')

-- =========================================================
-- Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua
--
-- 第 1 步：创建干净场景 + 60% 工件 + 工作台完全分离 + 颜色循环准备版
--
-- 这个脚本只做：
-- 1. 创建地面、两个圆形工作台、供料区、装配区、检测区、传送带、固定相机；
-- 2. 创建 60% 尺寸的电控箱、PCB、控制模块、端子排；
-- 3. 自动把已有 R1/R2/R3/R4/R5 放到对应基座；
-- 4. 冻结机械臂动力学，避免机械臂乱飞。
--
-- 这个脚本不做：
-- 1. 不创建夹爪；
-- 2. 不创建目标点；
-- 3. 不创建 TCP 点；
-- 4. 不接 ROS2；
-- 5. 不控制夹爪。
--
-- 使用：
-- 1. 新建空场景，先导入并命名 5 台机械臂：R1/R2/R3/R4/R5；
-- 2. 新建 Dummy：Step01_Create_Clean_Cell_60；
-- 3. 添加 Non-threaded child script；
-- 4. 粘贴本脚本；
-- 5. 运行一次；
-- 6. 成功后把本脚本禁用或删除。
-- =========================================================

-- 第一次运行 true。如果你后面已经调好了场景，改 false，避免重复删除重建。
local REBUILD_CELL_ON_START = true

-- 自动摆放 R1~R5
local AUTO_PLACE_ROBOTS = true

-- 冻结机械臂动力学，防止仿真运行时乱飞
local FREEZE_ROBOTS = true

-- 工件缩放比例
local WORKPIECE_SCALE = 0.60

-- 机器人基座 z 微调
local ROBOT_Z_OFFSET = 0.02

-- =========================================================
-- 基础高度
-- =========================================================

local TABLE_TOP_Z = 0.12

local AREA_H = 0.035
local AREA_Z = TABLE_TOP_Z + AREA_H / 2
local AREA_TOP_Z = TABLE_TOP_Z + AREA_H

local FIXTURE_H = 0.060
local FIXTURE_Z = AREA_TOP_Z + FIXTURE_H / 2
local FIXTURE_TOP_Z = AREA_TOP_Z + FIXTURE_H

local SUPPLY_BOTTOM_Z = AREA_TOP_Z + 0.001
local PRODUCT_BOTTOM_Z = FIXTURE_TOP_Z + 0.001

local PRODUCT_ON_BELT_Z = 0.270

-- =========================================================
-- 60% 工件尺寸
-- =========================================================

local BOX_L = 0.35 * WORKPIECE_SCALE
local BOX_W = 0.25 * WORKPIECE_SCALE
local BOX_H = 0.12 * WORKPIECE_SCALE
local BOX_T = 0.012 * WORKPIECE_SCALE

local PCB_L = 0.24 * WORKPIECE_SCALE
local PCB_W = 0.16 * WORKPIECE_SCALE
local PCB_T = 0.008 * WORKPIECE_SCALE

local MOD_L = 0.09 * WORKPIECE_SCALE
local MOD_W = 0.065 * WORKPIECE_SCALE
local MOD_H = 0.035 * WORKPIECE_SCALE

local TER_L = 0.16 * WORKPIECE_SCALE
local TER_W = 0.035 * WORKPIECE_SCALE
local TER_H = 0.035 * WORKPIECE_SCALE

-- 原来相对高度也同步缩小
local PCB_Z_OFFSET = 0.064 * WORKPIECE_SCALE
local MODULE_Z_OFFSET = (0.064 + 0.004 + 0.0175) * WORKPIECE_SCALE
local TERMINAL_Z_OFFSET = MODULE_Z_OFFSET

-- 原来内部偏移也同步缩小
local MODULE_X_OFFSET = 0.045 * WORKPIECE_SCALE
local MODULE_Y_OFFSET = -0.015 * WORKPIECE_SCALE

local TERMINAL_X_OFFSET = 0.060 * WORKPIECE_SCALE
local TERMINAL_Y_OFFSET = -0.070 * WORKPIECE_SCALE

-- =========================================================
-- 颜色
-- =========================================================

local COLOR_GROUND   = {0.78, 0.78, 0.78}
local COLOR_TABLE    = {0.62, 0.62, 0.62}
local COLOR_METAL    = {0.50, 0.50, 0.50}
local COLOR_DARK     = {0.02, 0.02, 0.02}
local COLOR_BLACK    = {0.005, 0.005, 0.005}
local COLOR_AREA     = {0.86, 0.82, 0.68}
local COLOR_BOX      = {0.62, 0.62, 0.62}
local COLOR_PCB      = {0.00, 0.45, 0.18}
local COLOR_MODULE   = {0.08, 0.12, 0.18}
local COLOR_TERMINAL = {0.92, 0.82, 0.35}
local COLOR_CAMERA_VIEW = {0.75, 0.92, 1.00}
local COLOR_ROBOT_BODY = {0.92, 0.94, 0.96}
local COLOR_ROBOT_JOINT = {0.72, 0.80, 0.90}
local COLOR_ROBOT_ACCENT = {0.28, 0.55, 0.95}

-- =========================================================
-- 布局坐标
-- =========================================================

local robotBases = {
    -- 修正版：R1/R2/R3 全部收回左侧圆形工作台内
    R1 = {-1.60,  0.65, 0.17},
    R2 = {-1.56, -0.22, 0.17},
    R3 = {-0.62,  0.40, 0.17},
    R4 = { 0.78,  0.25, 0.17},
    R5 = { 0.35, -0.50, 0.17}
}

local robotYaw = {
    R1 = -35,
    R2 =  20,
    R3 =  15,
    R4 = 210,
    R5 = 110
}

local leftTableCenter  = {-1.20,  0.25, 0.06}
local rightTableCenter = { 0.75, -0.10, 0.06}

-- 工位坐标
local P = {
    -- 修正版：供料/装配区重新排布，避开 R1/R2/R3 底座
    boxSupply      = {-1.86,  0.22},
    terminalSupply = {-1.82, -0.02},
    pcbSupply      = {-1.22, -0.42},
    moduleSupply   = {-0.78, -0.20},

    assembly       = {-1.08,  0.12},
    inspection     = { 0.35,  0.05},

    cameraColumn   = { 0.10,  0.55},

    goodInlet      = { 0.85, -1.10},
    defectInlet    = {-0.15, -1.12}
}

local root = -1
local groups = {}

-- =========================================================
-- 通用函数
-- =========================================================

local function safeGet(path)
    local ok, h = pcall(sim.getObject, path)
    if ok then return h end
    return -1
end

local function setWorldPosition(h, p)
    if h == -1 then return end
    pcall(sim.setObjectPosition, h, -1, p)
end

local function setWorldOrientation(h, e)
    if h == -1 then return end
    pcall(sim.setObjectOrientation, h, -1, e)
end

local function setColor(obj, color)
    pcall(sim.setShapeColor, obj, nil, sim.colorcomponent_ambient_diffuse, color)
end

local function parentTo(obj, parent)
    if parent and parent ~= -1 then
        sim.setObjectParent(obj, parent, true)
    end
end

local function removeTree(h)
    if h == -1 then return end
    local ok, objs = pcall(sim.getObjectsInTree, h, sim.handle_all, 0)
    if ok and objs then
        sim.removeObjects(objs)
    else
        sim.removeObjects({h})
    end
end

local function makeGroup(name, parent, pos)
    local h = sim.createDummy(0.01)
    sim.setObjectAlias(h, name)

    -- 隐藏分组 Dummy 自身
    pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 0)

    parentTo(h, parent)
    if pos then setWorldPosition(h, pos) end
    return h
end

local function cuboid(name, pos, size, color, parent)
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, name)
    setWorldPosition(h, pos)
    setWorldOrientation(h, {0,0,0})
    setColor(h, color)
    parentTo(h, parent)

    pcall(sim.setObjectInt32Param, h, sim.shapeintparam_static, 1)
    pcall(sim.setObjectInt32Param, h, sim.shapeintparam_respondable, 0)
    return h
end

local function cylinder(name, pos, radius, height, color, parent)
    local h = sim.createPrimitiveShape(
        sim.primitiveshape_cylinder,
        {radius * 2, radius * 2, height},
        0
    )
    sim.setObjectAlias(h, name)
    setWorldPosition(h, pos)
    setWorldOrientation(h, {0,0,0})
    setColor(h, color)
    parentTo(h, parent)

    pcall(sim.setObjectInt32Param, h, sim.shapeintparam_static, 1)
    pcall(sim.setObjectInt32Param, h, sim.shapeintparam_respondable, 0)
    return h
end

local function getAllAliases(h)
    local names = {}

    local ok1, n1 = pcall(sim.getObjectAlias, h, 0)
    if ok1 and n1 then table.insert(names, n1) end

    local ok2, n2 = pcall(sim.getObjectAlias, h, 1)
    if ok2 and n2 then table.insert(names, n2) end

    return names
end

local function findInTreeByName(rootHandle, targetName)
    local ok, objs = pcall(sim.getObjectsInTree, rootHandle, sim.handle_all, 0)
    if not ok or not objs then return -1 end

    for i = 1, #objs do
        local h = objs[i]
        local names = getAllAliases(h)

        for k = 1, #names do
            local n = names[k]
            if n == targetName then return h end
            if string.find(n, targetName, 1, true) then return h end
        end
    end

    return -1
end

-- =========================================================
-- 工件建模
-- =========================================================

local function makeControlBoxShell(prefix, x, y, bottomZ, parent, color)
    local g = makeGroup(prefix, parent, {x, y, bottomZ})

    cuboid(prefix .. '_Bottom', {x, y, bottomZ + BOX_T / 2}, {BOX_L, BOX_W, BOX_T}, color, g)

    cuboid(prefix .. '_Left_Wall',
        {x - BOX_L / 2 + BOX_T / 2, y, bottomZ + BOX_H / 2},
        {BOX_T, BOX_W, BOX_H},
        color,
        g
    )

    cuboid(prefix .. '_Right_Wall',
        {x + BOX_L / 2 - BOX_T / 2, y, bottomZ + BOX_H / 2},
        {BOX_T, BOX_W, BOX_H},
        color,
        g
    )

    cuboid(prefix .. '_Front_Wall',
        {x, y - BOX_W / 2 + BOX_T / 2, bottomZ + BOX_H / 2},
        {BOX_L, BOX_T, BOX_H},
        color,
        g
    )

    cuboid(prefix .. '_Back_Wall',
        {x, y + BOX_W / 2 - BOX_T / 2, bottomZ + BOX_H / 2},
        {BOX_L, BOX_T, BOX_H},
        color,
        g
    )

    -- 四个小立柱
    local postX = BOX_L * 0.36
    local postY = BOX_W * 0.32
    local postR = 0.010 * WORKPIECE_SCALE
    local postH = 0.050 * WORKPIECE_SCALE
    cylinder(prefix .. '_Post_1', {x - postX, y - postY, bottomZ + postH / 2 + BOX_T}, postR, postH, COLOR_METAL, g)
    cylinder(prefix .. '_Post_2', {x + postX, y - postY, bottomZ + postH / 2 + BOX_T}, postR, postH, COLOR_METAL, g)
    cylinder(prefix .. '_Post_3', {x - postX, y + postY, bottomZ + postH / 2 + BOX_T}, postR, postH, COLOR_METAL, g)
    cylinder(prefix .. '_Post_4', {x + postX, y + postY, bottomZ + postH / 2 + BOX_T}, postR, postH, COLOR_METAL, g)

    return g
end

local function makePCB(prefix, x, y, z, parent)
    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Board', {x, y, z}, {PCB_L, PCB_W, PCB_T}, COLOR_PCB, g)

    local gold = {0.95, 0.70, 0.15}

    local hx = PCB_L * 0.43
    local hy = PCB_W * 0.40
    cylinder(prefix .. '_Hole_1', {x - hx, y - hy, z + PCB_T}, 0.007 * WORKPIECE_SCALE, 0.004 * WORKPIECE_SCALE, gold, g)
    cylinder(prefix .. '_Hole_2', {x + hx, y - hy, z + PCB_T}, 0.007 * WORKPIECE_SCALE, 0.004 * WORKPIECE_SCALE, gold, g)
    cylinder(prefix .. '_Hole_3', {x - hx, y + hy, z + PCB_T}, 0.007 * WORKPIECE_SCALE, 0.004 * WORKPIECE_SCALE, gold, g)
    cylinder(prefix .. '_Hole_4', {x + hx, y + hy, z + PCB_T}, 0.007 * WORKPIECE_SCALE, 0.004 * WORKPIECE_SCALE, gold, g)

    cuboid(prefix .. '_Main_Chip', {x - 0.030 * WORKPIECE_SCALE, y, z + 0.014 * WORKPIECE_SCALE}, {0.045 * WORKPIECE_SCALE, 0.045 * WORKPIECE_SCALE, 0.012 * WORKPIECE_SCALE}, COLOR_DARK, g)
    cuboid(prefix .. '_Small_Chip', {x + 0.055 * WORKPIECE_SCALE, y + 0.040 * WORKPIECE_SCALE, z + 0.013 * WORKPIECE_SCALE}, {0.030 * WORKPIECE_SCALE, 0.025 * WORKPIECE_SCALE, 0.010 * WORKPIECE_SCALE}, COLOR_DARK, g)

    cuboid(prefix .. '_Connector_1', {x - 0.070 * WORKPIECE_SCALE, y - 0.065 * WORKPIECE_SCALE, z + 0.016 * WORKPIECE_SCALE}, {0.060 * WORKPIECE_SCALE, 0.020 * WORKPIECE_SCALE, 0.020 * WORKPIECE_SCALE}, COLOR_TERMINAL, g)
    cuboid(prefix .. '_Connector_2', {x + 0.070 * WORKPIECE_SCALE, y - 0.065 * WORKPIECE_SCALE, z + 0.016 * WORKPIECE_SCALE}, {0.060 * WORKPIECE_SCALE, 0.020 * WORKPIECE_SCALE, 0.020 * WORKPIECE_SCALE}, COLOR_TERMINAL, g)

    return g
end

local function makeControlModule(prefix, x, y, z, parent)
    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Body', {x, y, z}, {MOD_L, MOD_W, MOD_H}, COLOR_MODULE, g)
    cuboid(prefix .. '_Label', {x + MOD_L / 2 + 0.001, y, z + 0.001}, {0.002, MOD_W * 0.55, MOD_H * 0.55}, {0.90, 0.90, 0.82}, g)

    return g
end

local function makeTerminalBlock(prefix, x, y, z, parent)
    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Body', {x, y, z}, {TER_L, TER_W, TER_H}, COLOR_TERMINAL, g)

    for i = 1, 4 do
        local sx = x - TER_L * 0.375 + (i - 1) * TER_L * 0.25
        cuboid(prefix .. '_Slot_' .. i, {sx, y - TER_W / 2 - 0.001, z - TER_H * 0.05}, {TER_L * 0.12, 0.004, TER_H * 0.40}, COLOR_DARK, g)
    end

    cylinder(prefix .. '_Main_ScrewHead', {x, y, z + TER_H / 2 + 0.004 * WORKPIECE_SCALE}, 0.008 * WORKPIECE_SCALE, 0.005 * WORKPIECE_SCALE, COLOR_METAL, g)

    return g
end

local function makeAssembledControlBox(prefix, x, y, shellZ, parent, boxColor)
    local productGroup = makeGroup(prefix, parent, {x, y, shellZ})

    makeControlBoxShell(prefix .. '_Shell', x, y, shellZ, productGroup, boxColor)

    local pcbZ = shellZ + PCB_Z_OFFSET
    local moduleZ = shellZ + MODULE_Z_OFFSET
    local terminalZ = shellZ + TERMINAL_Z_OFFSET

    makePCB(prefix .. '_PCB', x, y, pcbZ, productGroup)

    makeControlModule(
        prefix .. '_Control_Module',
        x + MODULE_X_OFFSET,
        y + MODULE_Y_OFFSET,
        moduleZ,
        productGroup
    )

    makeTerminalBlock(
        prefix .. '_Terminal_Block',
        x + TERMINAL_X_OFFSET,
        y + TERMINAL_Y_OFFSET,
        terminalZ,
        productGroup
    )

    return productGroup
end

-- =========================================================
-- 工作台、相机、传送带
-- =========================================================

local function makeConveyor(prefix, center, length, width, direction, parent)
    local g = makeGroup(prefix, parent, center)

    local x = center[1]
    local y = center[2]
    local z = center[3]

    local frameH = 0.12
    local legW = 0.035
    local legH = z - frameH / 2
    if legH < 0.02 then legH = 0.02 end

    local legZ = legH / 2

    if direction == 'Y' then
        cuboid(prefix .. '_Frame', {x, y, z}, {width + 0.10, length, frameH}, COLOR_METAL, g)
        cuboid(prefix .. '_Belt_Black', {x, y, z + 0.075}, {width, length - 0.08, 0.030}, COLOR_BLACK, g)

        cuboid(prefix .. '_Leg_1', {x - width / 2, y - length / 2 + 0.12, legZ}, {legW, legW, legH}, COLOR_METAL, g)
        cuboid(prefix .. '_Leg_2', {x + width / 2, y - length / 2 + 0.12, legZ}, {legW, legW, legH}, COLOR_METAL, g)
        cuboid(prefix .. '_Leg_3', {x - width / 2, y + length / 2 - 0.12, legZ}, {legW, legW, legH}, COLOR_METAL, g)
        cuboid(prefix .. '_Leg_4', {x + width / 2, y + length / 2 - 0.12, legZ}, {legW, legW, legH}, COLOR_METAL, g)
    end

    if direction == 'X' then
        cuboid(prefix .. '_Frame', {x, y, z}, {length, width + 0.10, frameH}, COLOR_METAL, g)
        cuboid(prefix .. '_Belt_Black', {x, y, z + 0.075}, {length - 0.08, width, 0.030}, COLOR_BLACK, g)

        cuboid(prefix .. '_Leg_1', {x - length / 2 + 0.12, y - width / 2, legZ}, {legW, legW, legH}, COLOR_METAL, g)
        cuboid(prefix .. '_Leg_2', {x - length / 2 + 0.12, y + width / 2, legZ}, {legW, legW, legH}, COLOR_METAL, g)
        cuboid(prefix .. '_Leg_3', {x + length / 2 - 0.12, y - width / 2, legZ}, {legW, legW, legH}, COLOR_METAL, g)
        cuboid(prefix .. '_Leg_4', {x + length / 2 - 0.12, y + width / 2, legZ}, {legW, legW, legH}, COLOR_METAL, g)
    end

    return g
end

local function makeFixedCamera(parent)
    local g = makeGroup('Fixed_Vision_Camera_Station', parent)

    local x = P.cameraColumn[1]
    local y = P.cameraColumn[2]

    cylinder('Camera_Column_Base', {x, y, 0.08}, 0.055, 0.08, COLOR_METAL, g)
    cylinder('Camera_Column', {x, y, 0.48}, 0.022, 0.80, COLOR_METAL, g)

    cuboid('Camera_Bracket_X', {P.cameraColumn[1] + 0.13, P.cameraColumn[2], 0.86}, {0.30, 0.035, 0.035}, COLOR_METAL, g)
    cuboid('Camera_Bracket_Y', {P.inspection[1], (P.cameraColumn[2] + P.inspection[2]) / 2, 0.86}, {0.035, math.abs(P.cameraColumn[2] - P.inspection[2]), 0.035}, COLOR_METAL, g)

    cuboid('Fixed_Camera_Body', {P.inspection[1], P.inspection[2] + 0.10, 0.82}, {0.08, 0.06, 0.06}, COLOR_DARK, g)
    cuboid('Fixed_Camera_Lens', {P.inspection[1], P.inspection[2] + 0.10, 0.775}, {0.035, 0.035, 0.035}, COLOR_BLACK, g)

    cuboid(
        'Camera_View_Area',
        {P.inspection[1], P.inspection[2], FIXTURE_TOP_Z + 0.003},
        {0.30, 0.22, 0.005},
        COLOR_CAMERA_VIEW,
        g
    )

    return g
end

-- =========================================================
-- 创建场景主体
-- =========================================================

local function createScene()
    root = makeGroup('FiveCR5A_Cell', -1)

    groups.Ground     = makeGroup('Ground_Group', root)
    groups.Tables     = makeGroup('Tables', root)
    groups.RobotBases = makeGroup('RobotBases', root)
    groups.Areas      = makeGroup('Areas', root)
    groups.Parts      = makeGroup('Parts', root)
    groups.Conveyors  = makeGroup('Conveyors', root)
    groups.Sensors    = makeGroup('Sensors', root)

    cuboid('Ground', {-0.55, -0.30, -0.01}, {5.6, 4.3, 0.02}, COLOR_GROUND, groups.Ground)

    local tableRadius = 0.95
    cylinder('Damping_Table_Left', leftTableCenter, tableRadius, 0.12, COLOR_TABLE, groups.Tables)
    cylinder('Damping_Table_Right', rightTableCenter, tableRadius, 0.12, COLOR_TABLE, groups.Tables)

    local padRadius = 0.76
    for i = 1, 8 do
        local a = (i - 1) * 2 * math.pi / 8

        cylinder(
            'Left_RubberPad_' .. i,
            {leftTableCenter[1] + padRadius * math.cos(a), leftTableCenter[2] + padRadius * math.sin(a), 0.03},
            0.045,
            0.06,
            COLOR_DARK,
            groups.Tables
        )

        cylinder(
            'Right_RubberPad_' .. i,
            {rightTableCenter[1] + padRadius * math.cos(a), rightTableCenter[2] + padRadius * math.sin(a), 0.03},
            0.045,
            0.06,
            COLOR_DARK,
            groups.Tables
        )
    end

    cylinder('R1_Base', robotBases.R1, 0.18, 0.10, COLOR_METAL, groups.RobotBases)
    cylinder('R2_Base', robotBases.R2, 0.18, 0.10, COLOR_METAL, groups.RobotBases)
    cylinder('R3_Base', robotBases.R3, 0.18, 0.10, COLOR_METAL, groups.RobotBases)
    cylinder('R4_Base', robotBases.R4, 0.18, 0.10, COLOR_METAL, groups.RobotBases)
    cylinder('R5_Base', robotBases.R5, 0.18, 0.10, COLOR_METAL, groups.RobotBases)

    -- 供料区
    cuboid('Box_Supply_Area', {P.boxSupply[1], P.boxSupply[2], AREA_Z}, {0.24, 0.16, AREA_H}, COLOR_AREA, groups.Areas)
    cuboid('Terminal_Supply_Area', {P.terminalSupply[1], P.terminalSupply[2], AREA_Z}, {0.18, 0.12, AREA_H}, COLOR_AREA, groups.Areas)
    cuboid('PCB_Supply_Area', {P.pcbSupply[1], P.pcbSupply[2], AREA_Z}, {0.22, 0.15, AREA_H}, COLOR_AREA, groups.Areas)
    cuboid('Module_Supply_Area', {P.moduleSupply[1], P.moduleSupply[2], AREA_Z}, {0.18, 0.12, AREA_H}, COLOR_AREA, groups.Areas)

    -- 装配区
    cuboid('Assembly_Area', {P.assembly[1], P.assembly[2], AREA_Z}, {0.38, 0.26, AREA_H}, {0.90, 0.82, 0.55}, groups.Areas)
    cuboid('Assembly_Fixture', {P.assembly[1], P.assembly[2], FIXTURE_Z}, {0.30, 0.20, FIXTURE_H}, COLOR_METAL, groups.Areas)

    -- 检测/锁付区
    cuboid('Inspection_Screw_Area', {P.inspection[1], P.inspection[2], AREA_Z}, {0.40, 0.28, AREA_H}, {0.90, 0.82, 0.55}, groups.Areas)
    cuboid('Inspection_Platform', {P.inspection[1], P.inspection[2], FIXTURE_Z}, {0.30, 0.20, FIXTURE_H}, COLOR_METAL, groups.Areas)

    makeFixedCamera(groups.Sensors)

    -- 60% 供料件
    makeControlBoxShell('Box_Blank', P.boxSupply[1], P.boxSupply[2], SUPPLY_BOTTOM_Z, groups.Parts, COLOR_BOX)
    makePCB('PCB_Supply', P.pcbSupply[1], P.pcbSupply[2], SUPPLY_BOTTOM_Z + PCB_T / 2, groups.Parts)
    makeControlModule('Control_Module_Supply', P.moduleSupply[1], P.moduleSupply[2], SUPPLY_BOTTOM_Z + MOD_H / 2, groups.Parts)
    makeTerminalBlock('Terminal_Block_Supply', P.terminalSupply[1], P.terminalSupply[2], SUPPLY_BOTTOM_Z + TER_H / 2, groups.Parts)

    -- 60% 装配区和检测区产品。后面控制流程里再决定显隐。
    makeAssembledControlBox('Assembly_ControlBox_Product', P.assembly[1], P.assembly[2], PRODUCT_BOTTOM_Z, groups.Parts, COLOR_BOX)
    makeAssembledControlBox('Inspection_ControlBox_Product', P.inspection[1], P.inspection[2], PRODUCT_BOTTOM_Z, groups.Parts, COLOR_BOX)

    -- 传送带
    makeConveyor('Good_Conveyor', {0.85, -1.72, 0.18}, 1.25, 0.36, 'Y', groups.Conveyors)
    makeConveyor('Defect_Conveyor', {-0.75, -1.12, 0.18}, 1.20, 0.36, 'X', groups.Conveyors)

    print('[OK] Separated-table 60% cell created. No targets. No grippers.')
end

-- =========================================================
-- 机械臂摆放和冻结
-- =========================================================

local function findRobotRoot(name)
    local h = safeGet('/' .. name)
    if h ~= -1 then return h end
    return -1
end

local function moveRobot(name)
    local robot = findRobotRoot(name)

    if robot == -1 then
        print('[ROBOT WARN] Cannot find /' .. name .. '. Please import and name robot as ' .. name)
        return
    end

    local b = robotBases[name]
    local targetPos = {b[1], b[2], b[3] + ROBOT_Z_OFFSET}

    setWorldPosition(robot, targetPos)
    setWorldOrientation(robot, {0, 0, math.rad(robotYaw[name])})

    print('[ROBOT OK] ' .. name .. ' moved to base.')
end

local function placeAllRobots()
    if not AUTO_PLACE_ROBOTS then return end

    moveRobot('R1')
    moveRobot('R2')
    moveRobot('R3')
    moveRobot('R4')
    moveRobot('R5')
end

local function freezeRobot(robotName)
    local robot = safeGet('/' .. robotName)

    if robot == -1 then
        print('[KIN WARN] Cannot find /' .. robotName)
        return
    end

    local ok, objs = pcall(sim.getObjectsInTree, robot, sim.handle_all, 0)

    if not ok or not objs then
        print('[KIN WARN] Cannot get tree for /' .. robotName)
        return
    end

    for i = 1, #objs do
        local h = objs[i]
        local okType, objType = pcall(sim.getObjectType, h)

        if okType then
            if objType == sim.object_shape_type then
                pcall(sim.setObjectInt32Param, h, sim.shapeintparam_static, 1)
                pcall(sim.setObjectInt32Param, h, sim.shapeintparam_respondable, 0)
            end

            if objType == sim.object_joint_type then
                pcall(sim.setJointMode, h, sim.jointmode_kinematic, 0)

                local okQ, q = pcall(sim.getJointPosition, h)
                if okQ then
                    pcall(sim.setJointTargetPosition, h, q)
                end

                pcall(
                    sim.setObjectInt32Param,
                    h,
                    sim.jointintparam_dynctrlmode,
                    sim.jointdynctrl_position
                )
            end
        end
    end

    print('[KIN OK] Frozen dynamics for /' .. robotName)
end

local function freezeAllRobots()
    if not FREEZE_ROBOTS then return end

    freezeRobot('R1')
    freezeRobot('R2')
    freezeRobot('R3')
    freezeRobot('R4')
    freezeRobot('R5')
end


-- =========================================================
-- 机械臂配色：白灰主体 + 蓝色关节点缀
-- =========================================================

local function getAliasesNoDeprecated(h)
    local result = {}
    local ok1, n1 = pcall(sim.getObjectAlias, h, 0)
    if ok1 and n1 then table.insert(result, n1) end
    local ok2, n2 = pcall(sim.getObjectAlias, h, 1)
    if ok2 and n2 then table.insert(result, n2) end
    return result
end

local function aliasHas(h, key)
    local ns = getAliasesNoDeprecated(h)
    for i=1,#ns do
        if string.find(string.lower(ns[i]), string.lower(key), 1, true) then
            return true
        end
    end
    return false
end

local function colorRobot(robotName)
    local robot = safeGet('/' .. robotName)
    if robot == -1 then
        print('[COLOR WARN] Cannot find /' .. robotName)
        return
    end

    local ok, objs = pcall(sim.getObjectsInTree, robot, sim.handle_all, 0)
    if not ok or not objs then return end

    local count = 0
    for i=1,#objs do
        local h = objs[i]
        local okType, t = pcall(sim.getObjectType, h)
        if okType and t == sim.object_shape_type then
            count = count + 1

            -- 默认白灰色，接近参考图
            local col = COLOR_ROBOT_BODY

            -- 对含 base/joint/link6/flange 等名字的 shape 做轻微蓝灰点缀
            if aliasHas(h, 'base') or aliasHas(h, 'joint') or aliasHas(h, 'flange') then
                col = COLOR_ROBOT_JOINT
            end

            if aliasHas(h, 'link6') or aliasHas(h, 'tool') then
                col = COLOR_ROBOT_ACCENT
            end

            pcall(sim.setShapeColor, h, nil, sim.colorcomponent_ambient_diffuse, col)
            pcall(sim.setShapeColor, h, nil, sim.colorcomponent_specular, {0.25,0.25,0.25})
            pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 1)
        end
    end

    print('[COLOR OK] ' .. robotName .. ' shapes colored: ' .. count)
end

local function colorAllRobots()
    colorRobot('R1')
    colorRobot('R2')
    colorRobot('R3')
    colorRobot('R4')
    colorRobot('R5')
end

-- =========================================================
-- CoppeliaSim 回调
-- =========================================================

function sysCall_init()
    print('===== Step01: Create Clean 60% Cell, Separated Tables, Color Ready =====')

    local oldRoot = safeGet('/FiveCR5A_Cell')

    if oldRoot ~= -1 and REBUILD_CELL_ON_START then
        print('[INFO] Removing old /FiveCR5A_Cell ...')
        removeTree(oldRoot)
        oldRoot = -1
    end

    if oldRoot == -1 then
        createScene()
    else
        root = oldRoot
        print('[INFO] /FiveCR5A_Cell already exists, skip scene creation.')
    end

    placeAllRobots()
    colorAllRobots()
    freezeAllRobots()

    print('===== Step01 Done =====')
    print('[NEXT] Disable this script, then go to Step02: create and mount end effectors.')
end

function sysCall_cleanup()
end
