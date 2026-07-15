sim = require('sim')

-- =========================================================
-- Five CR5A Electrical Control Box Assembly Cell
-- ??? + Tip + Target + Kinematic setup ?????
-- =========================================================

-- ?????????? true
local REBUILD_SCENE_ON_START = false

-- ????????? R1~R5 ? tip ?
-- ?????? tip ???? false
local RESET_TIPS_ON_START = false

-- ????????????
-- ????????????? false
local RESET_TARGETS_ON_START = false

-- ???????? R1~R5 ?????
local AUTO_PLACE_ROBOTS = true

-- ???? R1~R5 ???????????
local FREEZE_ROBOT_DYNAMICS_ON_START = true

-- ?????????
local SHOW_TARGET_DUMMIES = true

-- ???????????
local CONVEYOR_ENABLED = true

-- ?????
local ENABLE_REACH_CHECK = true

-- ???????
local ROBOT_Z_OFFSET = 0.02


-- =========================================================
-- ????
-- =========================================================

local TABLE_TOP_Z = 0.12

local AREA_H = 0.035
local AREA_Z = TABLE_TOP_Z + AREA_H / 2
local AREA_TOP_Z = TABLE_TOP_Z + AREA_H

local FIXTURE_H = 0.060
local FIXTURE_Z = AREA_TOP_Z + FIXTURE_H / 2
local FIXTURE_TOP_Z = AREA_TOP_Z + FIXTURE_H

local ASSEMBLY_BOX_Z = FIXTURE_TOP_Z + 0.001
local INSPECTION_BOX_Z = FIXTURE_TOP_Z + 0.001

local SUPPLY_TOP_Z = AREA_TOP_Z + 0.001

local PCB_Z_OFFSET = 0.064
local MODULE_Z_OFFSET = PCB_Z_OFFSET + 0.004 + 0.0175
local TERMINAL_Z_OFFSET = PCB_Z_OFFSET + 0.004 + 0.0175

local PRODUCT_ON_BELT_Z = 0.270


-- =========================================================
-- ??
-- =========================================================

local COLOR_GROUND   = {0.78, 0.78, 0.78}
local COLOR_TABLE    = {0.60, 0.72, 0.38}
local COLOR_METAL    = {0.55, 0.55, 0.55}
local COLOR_DARK     = {0.02, 0.02, 0.02}
local COLOR_BLACK    = {0.005, 0.005, 0.005}
local COLOR_AREA     = {0.88, 0.84, 0.68}
local COLOR_BOX      = {0.62, 0.62, 0.62}
local COLOR_PCB      = {0.00, 0.45, 0.18}
local COLOR_MODULE   = {0.08, 0.12, 0.18}
local COLOR_TERMINAL = {0.92, 0.92, 0.88}
local COLOR_CAMERA_VIEW = {0.75, 0.92, 1.00}
local COLOR_TARGET = {1.00, 0.72, 0.20}


-- =========================================================
-- ????
-- =========================================================

local robotBases = {
    R1 = {-1.47,  0.67, 0.17},
    R2 = {-1.55, -0.15, 0.17},
    R3 = {-0.55,  0.28, 0.17},
    R4 = { 0.58,  0.25, 0.17},
    R5 = { 0.15, -0.50, 0.17}
}

local robotYaw = {
    R1 = -35,
    R2 =  20,
    R3 =  15,
    R4 = 210,
    R5 = 110
}

local leftTableCenter  = {-1.20,  0.25, 0.06}
local rightTableCenter = { 0.55, -0.10, 0.06}

-- ????????
local P = {
    -- ???????? R1????????
    boxSupply      = {-1.80,  0.35},
    terminalSupply = {-1.90, 0.10},
    pcbSupply      = {-1.28, -0.28},
    moduleSupply   = {-0.85, -0.05},

    assembly       = {-1.15,  0.20},
    inspection     = { 0.15,  0.05},

    cameraColumn   = {-0.10,  0.55},

    goodInlet      = { 0.65, -1.10},
    defectInlet    = {-0.35, -1.12}
}

local goodStart   = { 0.65, -1.10, PRODUCT_ON_BELT_Z}
local goodEnd     = { 0.65, -2.20, PRODUCT_ON_BELT_Z}
local defectStart = {-0.35, -1.12, PRODUCT_ON_BELT_Z}
local defectEnd   = {-1.45, -1.12, PRODUCT_ON_BELT_Z}

local conveyorSpeed = 0.18
local conveyorStartTime = 0.0

local root = -1
local groups = {}

local activeProduct = -1
local activeConveyor = nil


-- =========================================================
-- ??????
-- =========================================================

local function safeGet(path)
    local ok, h = pcall(sim.getObject, path)
    if ok then return h end
    return -1
end

local function setWorldPosition(h, p)
    if h == -1 then return end
    local ok = pcall(sim.setObjectPosition, h, -1, p)
    if not ok then pcall(sim.setObjectPosition, h, p) end
end

local function getWorldPosition(h)
    local ok, p = pcall(sim.getObjectPosition, h, -1)
    if ok then return p end
    local ok2, p2 = pcall(sim.getObjectPosition, h)
    if ok2 then return p2 end
    return nil
end

local function setWorldOrientation(h, e)
    if h == -1 then return end
    local ok = pcall(sim.setObjectOrientation, h, -1, e)
    if not ok then pcall(sim.setObjectOrientation, h, e) end
end

local function setColor(obj, color)
    pcall(sim.setShapeColor, obj, nil, sim.colorcomponent_ambient_diffuse, color)
end

local function parentTo(obj, parent)
    if parent and parent ~= -1 then
        sim.setObjectParent(obj, parent, true)
    end
end

local function makeGroup(name, parent, pos)
    local h = sim.createDummy(0.01)
    sim.setObjectAlias(h, name)

    -- ???? Dummy ???????????
    pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 0)

    parentTo(h, parent)
    if pos then setWorldPosition(h, pos) end
    return h
end

local function cuboid(name, pos, size, color, parent)
    local h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, name)
    setWorldPosition(h, pos)
    setColor(h, color)
    parentTo(h, parent)
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
    setColor(h, color)
    parentTo(h, parent)
    return h
end

local function targetDummy(name, pos, ori, parent)
    local h = sim.createDummy(0.025)
    sim.setObjectAlias(h, name)
    setWorldPosition(h, pos)
    setWorldOrientation(h, ori or {0, 0, 0})
    parentTo(h, parent)

    if SHOW_TARGET_DUMMIES then
        pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 1)
    else
        pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 0)
    end

    return h
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

local function getAllNames(h)
    local names = {}

    local ok1, n1 = pcall(sim.getObjectAlias, h, 0)
    if ok1 and n1 then table.insert(names, n1) end

    local ok2, n2 = pcall(sim.getObjectAlias, h, 1)
    if ok2 and n2 then table.insert(names, n2) end

    local ok3, n3 = pcall(sim.getObjectName, h)
    if ok3 and n3 then table.insert(names, n3) end

    return names
end

local function findInTreeByName(rootHandle, targetName)
    local ok, objs = pcall(sim.getObjectsInTree, rootHandle, sim.handle_all, 0)

    if not ok or not objs then return -1 end

    for i = 1, #objs do
        local h = objs[i]
        local names = getAllNames(h)

        for k = 1, #names do
            local n = names[k]
            if n == targetName then return h end
            if string.find(n, targetName, 1, true) then return h end
        end
    end

    return -1
end


-- =========================================================
-- ?? / ?? / ??
-- ????? shape????? Dummy ???????
-- =========================================================

local function getTreeObjects(path)
    local h = safeGet(path)
    if h == -1 then return {} end

    local result = {h}
    local ok, objs = pcall(sim.getObjectsInTree, h, sim.handle_all, 0)

    if ok and objs then
        for i = 1, #objs do
            table.insert(result, objs[i])
        end
    end

    return result
end

local function setTreeVisible(path, visible)
    local objs = getTreeObjects(path)
    local layer = visible and 1 or 0

    for i = 1, #objs do
        local h = objs[i]
        local okType, objType = pcall(sim.getObjectType, h)

        if okType and objType == sim.object_shape_type then
            pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, layer)
        end
    end
end

local function setTreeColor(path, color)
    local objs = getTreeObjects(path)

    for i = 1, #objs do
        local h = objs[i]
        local okType, objType = pcall(sim.getObjectType, h)

        if okType and objType == sim.object_shape_type then
            pcall(sim.setShapeColor, h, nil, sim.colorcomponent_ambient_diffuse, color)
        end
    end
end


-- =========================================================
-- ??????
-- =========================================================

local function setProductStage(productName, stage)
    local productRoot = '/FiveCR5A_Cell/Parts/' .. productName
    local base = productRoot .. '/' .. productName

    if stage <= 0 then
        setTreeVisible(productRoot, false)
        return
    end

    setTreeVisible(base .. '_Shell', stage >= 1)
    setTreeVisible(base .. '_PCB', stage >= 2)
    setTreeVisible(base .. '_Control_Module', stage >= 3)
    setTreeVisible(base .. '_Terminal_Block', stage >= 4)
end

local function setSupplyVisible(boxVisible, pcbVisible, moduleVisible, terminalVisible)
    setTreeVisible('/FiveCR5A_Cell/Parts/Box_Blank', boxVisible)
    setTreeVisible('/FiveCR5A_Cell/Parts/PCB_Supply', pcbVisible)
    setTreeVisible('/FiveCR5A_Cell/Parts/Control_Module_Supply', moduleVisible)
    setTreeVisible('/FiveCR5A_Cell/Parts/Terminal_Block_Supply', terminalVisible)
end

local function resetRuntimeState()
    -- ??????????
    setSupplyVisible(true, true, true, true)

    -- ?????????
    setProductStage('Assembly_ControlBox_Product', 0)
    setProductStage('Inspection_ControlBox_Product', 0)

    setTreeColor(
        '/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area',
        COLOR_CAMERA_VIEW
    )

    activeProduct = -1
    activeConveyor = nil
    conveyorStartTime = sim.getSimulationTime()

    sim.clearStringSignal('cell_product_state')
    sim.clearStringSignal('cell_conveyor_state')

    print('[OK] Runtime reset: supply visible, assembly/inspection/conveyors empty.')
end


-- =========================================================
-- ???????
-- =========================================================

local function makeControlBoxShell(prefix, x, y, z, parent, color)
    local L = 0.35
    local W = 0.25
    local H = 0.12
    local T = 0.012

    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Bottom', {x, y, z + T / 2}, {L, W, T}, color, g)

    cuboid(prefix .. '_Left_Wall',
        {x - L / 2 + T / 2, y, z + H / 2},
        {T, W, H},
        color,
        g
    )

    cuboid(prefix .. '_Right_Wall',
        {x + L / 2 - T / 2, y, z + H / 2},
        {T, W, H},
        color,
        g
    )

    cuboid(prefix .. '_Front_Wall',
        {x, y - W / 2 + T / 2, z + H / 2},
        {L, T, H},
        color,
        g
    )

    cuboid(prefix .. '_Back_Wall',
        {x, y + W / 2 - T / 2, z + H / 2},
        {L, T, H},
        color,
        g
    )

    cylinder(prefix .. '_Post_1', {x - 0.125, y - 0.080, z + 0.035}, 0.010, 0.050, COLOR_METAL, g)
    cylinder(prefix .. '_Post_2', {x + 0.125, y - 0.080, z + 0.035}, 0.010, 0.050, COLOR_METAL, g)
    cylinder(prefix .. '_Post_3', {x - 0.125, y + 0.080, z + 0.035}, 0.010, 0.050, COLOR_METAL, g)
    cylinder(prefix .. '_Post_4', {x + 0.125, y + 0.080, z + 0.035}, 0.010, 0.050, COLOR_METAL, g)

    local g1 = cylinder(
        prefix .. '_CableGland_1',
        {x - 0.08, y - W / 2 - 0.012, z + 0.055},
        0.018,
        0.035,
        COLOR_DARK,
        g
    )
    setWorldOrientation(g1, {math.pi / 2, 0, 0})

    local g2 = cylinder(
        prefix .. '_CableGland_2',
        {x + 0.08, y - W / 2 - 0.012, z + 0.055},
        0.018,
        0.035,
        COLOR_DARK,
        g
    )
    setWorldOrientation(g2, {math.pi / 2, 0, 0})

    return g
end

local function makePCB(prefix, x, y, z, parent)
    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Board', {x, y, z}, {0.24, 0.16, 0.008}, COLOR_PCB, g)

    local gold = {0.95, 0.70, 0.15}

    cylinder(prefix .. '_Hole_1', {x - 0.105, y - 0.065, z + 0.006}, 0.007, 0.004, gold, g)
    cylinder(prefix .. '_Hole_2', {x + 0.105, y - 0.065, z + 0.006}, 0.007, 0.004, gold, g)
    cylinder(prefix .. '_Hole_3', {x - 0.105, y + 0.065, z + 0.006}, 0.007, 0.004, gold, g)
    cylinder(prefix .. '_Hole_4', {x + 0.105, y + 0.065, z + 0.006}, 0.007, 0.004, gold, g)

    cuboid(prefix .. '_Main_Chip', {x - 0.03, y, z + 0.014}, {0.045, 0.045, 0.012}, COLOR_DARK, g)
    cuboid(prefix .. '_Small_Chip', {x + 0.055, y + 0.040, z + 0.013}, {0.030, 0.025, 0.010}, COLOR_DARK, g)

    cuboid(prefix .. '_Connector_1', {x - 0.070, y - 0.065, z + 0.016}, {0.060, 0.020, 0.020}, COLOR_TERMINAL, g)
    cuboid(prefix .. '_Connector_2', {x + 0.070, y - 0.065, z + 0.016}, {0.060, 0.020, 0.020}, COLOR_TERMINAL, g)

    cylinder(prefix .. '_Capacitor_1', {x - 0.090, y + 0.040, z + 0.022}, 0.008, 0.025, {0.05, 0.20, 0.80}, g)
    cylinder(prefix .. '_Capacitor_2', {x + 0.095, y + 0.035, z + 0.022}, 0.008, 0.025, {0.05, 0.20, 0.80}, g)

    return g
end

local function makeControlModule(prefix, x, y, z, parent)
    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Body', {x, y, z}, {0.09, 0.065, 0.035}, COLOR_MODULE, g)
    cuboid(prefix .. '_Label', {x + 0.046, y, z + 0.003}, {0.002, 0.035, 0.020}, {0.90, 0.90, 0.82}, g)

    return g
end

local function makeTerminalBlock(prefix, x, y, z, parent)
    local g = makeGroup(prefix, parent, {x, y, z})

    cuboid(prefix .. '_Body', {x, y, z}, {0.16, 0.035, 0.035}, COLOR_TERMINAL, g)

    for i = 1, 4 do
        local sx = x - 0.060 + (i - 1) * 0.040
        cuboid(prefix .. '_Slot_' .. i, {sx, y - 0.019, z - 0.002}, {0.020, 0.004, 0.014}, COLOR_DARK, g)
    end

    cylinder(prefix .. '_Main_ScrewHead', {x, y, z + 0.022}, 0.008, 0.005, COLOR_METAL, g)

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
        x + 0.030,
        y + 0.020,
        moduleZ,
        productGroup
    )

    makeTerminalBlock(
        prefix .. '_Terminal_Block',
        x + 0.060,
        y - 0.070,
        terminalZ,
        productGroup
    )

    return productGroup
end


-- =========================================================
-- ???
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


-- =========================================================
-- ????
-- =========================================================

local function makeFixedCamera(parent)
    local g = makeGroup('Fixed_Vision_Camera_Station', parent)

    local x = P.cameraColumn[1]
    local y = P.cameraColumn[2]

    cylinder('Camera_Column_Base', {x, y, 0.08}, 0.055, 0.08, COLOR_METAL, g)
    cylinder('Camera_Column', {x, y, 0.48}, 0.022, 0.80, COLOR_METAL, g)

    cuboid('Camera_Bracket_X', {0.03, 0.55, 0.86}, {0.30, 0.035, 0.035}, COLOR_METAL, g)
    cuboid('Camera_Bracket_Y', {0.15, 0.33, 0.86}, {0.035, 0.45, 0.035}, COLOR_METAL, g)

    cuboid('Fixed_Camera_Body', {0.15, 0.15, 0.82}, {0.08, 0.06, 0.06}, COLOR_DARK, g)
    cuboid('Fixed_Camera_Lens', {0.15, 0.15, 0.775}, {0.035, 0.035, 0.035}, COLOR_BLACK, g)

    cuboid(
        'Camera_View_Area',
        {P.inspection[1], P.inspection[2], FIXTURE_TOP_Z + 0.003},
        {0.42, 0.30, 0.005},
        COLOR_CAMERA_VIEW,
        g
    )

    return g
end


-- =========================================================
-- ?????
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
    groups.Targets    = makeGroup('Targets', root)

    cuboid('Ground', {-0.55, -0.30, -0.01}, {5.6, 4.3, 0.02}, COLOR_GROUND, groups.Ground)

    local tableRadius = 0.86
    cylinder('Damping_Table_Left', leftTableCenter, tableRadius, 0.12, COLOR_TABLE, groups.Tables)
    cylinder('Damping_Table_Right', rightTableCenter, tableRadius, 0.12, COLOR_TABLE, groups.Tables)

    local padRadius = 0.68

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

    -- ???
    cuboid('Box_Supply_Area', {P.boxSupply[1], P.boxSupply[2], AREA_Z}, {0.32, 0.22, AREA_H}, COLOR_AREA, groups.Areas)
  cuboid('Terminal_Supply_Area', {P.terminalSupply[1], P.terminalSupply[2], AREA_Z}, {0.24, 0.16, AREA_H}, COLOR_AREA, groups.Areas)
    cuboid('PCB_Supply_Area', {P.pcbSupply[1], P.pcbSupply[2], AREA_Z}, {0.36, 0.24, AREA_H}, COLOR_AREA, groups.Areas)
    cuboid('Module_Supply_Area', {P.moduleSupply[1], P.moduleSupply[2], AREA_Z}, {0.32, 0.20, AREA_H}, COLOR_AREA, groups.Areas)

    -- ???
    cuboid('Assembly_Area', {P.assembly[1], P.assembly[2], AREA_Z}, {0.50, 0.35, AREA_H}, {0.90, 0.82, 0.55}, groups.Areas)
    cuboid('Assembly_Fixture', {P.assembly[1], P.assembly[2], FIXTURE_Z}, {0.42, 0.28, FIXTURE_H}, COLOR_METAL, groups.Areas)

    -- ??/???
    cuboid('Inspection_Screw_Area', {P.inspection[1], P.inspection[2], AREA_Z}, {0.52, 0.34, AREA_H}, {0.90, 0.82, 0.55}, groups.Areas)
    cuboid('Inspection_Platform', {P.inspection[1], P.inspection[2], FIXTURE_Z}, {0.42, 0.28, FIXTURE_H}, COLOR_METAL, groups.Areas)

    makeFixedCamera(groups.Sensors)

    -- ??????????
    makeControlBoxShell('Box_Blank', P.boxSupply[1], P.boxSupply[2], SUPPLY_TOP_Z, groups.Parts, COLOR_BOX)
    makePCB('PCB_Supply', P.pcbSupply[1], P.pcbSupply[2], SUPPLY_TOP_Z + 0.004, groups.Parts)
    makeControlModule('Control_Module_Supply', P.moduleSupply[1], P.moduleSupply[2], SUPPLY_TOP_Z + 0.0175, groups.Parts)
    makeTerminalBlock('Terminal_Block_Supply', P.terminalSupply[1], P.terminalSupply[2], SUPPLY_TOP_Z + 0.0175, groups.Parts)

    -- ???????????
    makeAssembledControlBox('Assembly_ControlBox_Product', P.assembly[1], P.assembly[2], ASSEMBLY_BOX_Z, groups.Parts, COLOR_BOX)
    makeAssembledControlBox('Inspection_ControlBox_Product', P.inspection[1], P.inspection[2], INSPECTION_BOX_Z, groups.Parts, COLOR_BOX)

    -- ???
    makeConveyor('Good_Conveyor', {0.65, -1.72, 0.18}, 1.25, 0.36, 'Y', groups.Conveyors)
    makeConveyor('Defect_Conveyor', {-0.95, -1.12, 0.18}, 1.20, 0.36, 'X', groups.Conveyors)

    print('[OK] Main scene created.')
end


-- =========================================================
-- R1~R5 tip ???
-- =========================================================

local tipConfigs = {
    {robotName='R1', tipName='R1_gripper_tip', localPos={0,0,0}, localOri={0,0,0}},
    {robotName='R2', tipName='R2_gripper_tip', localPos={0,0,0}, localOri={0,0,0}},
    {robotName='R3', tipName='R3_gripper_tip', localPos={0,0,0}, localOri={0,0,0}},
    {robotName='R4', tipName='R4_tool_tip',    localPos={0,0,0}, localOri={0,0,0}},
    {robotName='R5', tipName='R5_gripper_tip', localPos={0,0,0}, localOri={0,0,0}},
}

local function createOrResetTip(robotName, tipName, localPos, localOri)
    local robot = safeGet('/' .. robotName)

    if robot == -1 then
        print('[TIP WARN] Cannot find /' .. robotName)
        return
    end

    local link6 = findInTreeByName(robot, 'Link6_visual')

    if link6 == -1 then
        link6 = findInTreeByName(robot, 'Link6')
    end

    if link6 == -1 then
        print('[TIP WARN] Cannot find Link6_visual under /' .. robotName)
        return
    end

    local tip = findInTreeByName(robot, tipName)

    if tip == -1 then
        tip = sim.createDummy(0.025)
        sim.setObjectAlias(tip, tipName)
        print('[TIP OK] Created ' .. tipName)
    elseif not RESET_TIPS_ON_START then
        print('[TIP KEEP] Existing ' .. tipName .. ' kept.')
        return
    else
        print('[TIP OK] Existing ' .. tipName .. ' reset.')
    end

    sim.setObjectParent(tip, link6, false)
    sim.setObjectPosition(tip, link6, localPos)
    sim.setObjectOrientation(tip, link6, localOri)

    local pWorld = sim.getObjectPosition(tip, -1)
    local pLocal = sim.getObjectPosition(tip, link6)

    print(string.format(
        '[TIP] %-16s world={%.3f, %.3f, %.3f} local={%.3f, %.3f, %.3f}',
        tipName,
        pWorld[1], pWorld[2], pWorld[3],
        pLocal[1], pLocal[2], pLocal[3]
    ))
end

local function createAllTips()
    print('========== Creating R1~R5 tips ==========')

    for i = 1, #tipConfigs do
        local cfg = tipConfigs[i]
        createOrResetTip(cfg.robotName, cfg.tipName, cfg.localPos, cfg.localOri)
    end

    print('========== Tip creation finished ==========')
end


-- =========================================================
-- ?????
-- APP = ???
-- TCP = ????/??/???
-- =========================================================

local targetConfigs = {
    -- R1
    {group='R1_Targets', name='R1_HOME_REF',           pos={-1.47,  0.67, 0.80}, ori={0,0,0}},
    {group='R1_Targets', name='R1_BOX_PICK_APP',       pos={-1.80,  0.35, 0.55}, ori={0,0,0}},
    {group='R1_Targets', name='R1_BOX_PICK_TCP',       pos={-1.80,  0.35, 0.30}, ori={0,0,0}},
    {group='R1_Targets', name='R1_BOX_PLACE_APP',      pos={-1.15,  0.20, 0.55}, ori={0,0,0}},
    {group='R1_Targets', name='R1_BOX_PLACE_TCP',      pos={-1.15,  0.20, 0.30}, ori={0,0,0}},
    {group='R1_Targets', name='R1_TERMINAL_PICK_APP',  pos={-1.90, 0.10, 0.45}, ori={0,0,0}},
    {group='R1_Targets', name='R1_TERMINAL_PICK_TCP',  pos={-1.90, 0.10, 0.24}, ori={0,0,0}},
    {group='R1_Targets', name='R1_TERMINAL_PLACE_APP', pos={-1.09,  0.13, 0.50}, ori={0,0,0}},
    {group='R1_Targets', name='R1_TERMINAL_PLACE_TCP', pos={-1.09,  0.13, 0.34}, ori={0,0,0}},

    -- R2
    {group='R2_Targets', name='R2_HOME_REF',           pos={-1.55, -0.15, 0.80}, ori={0,0,0}},
    {group='R2_Targets', name='R2_PCB_PICK_APP',       pos={-1.28, -0.28, 0.45}, ori={0,0,0}},
    {group='R2_Targets', name='R2_PCB_PICK_TCP',       pos={-1.28, -0.28, 0.22}, ori={0,0,0}},
    {group='R2_Targets', name='R2_PCB_PLACE_APP',      pos={-1.15,  0.20, 0.50}, ori={0,0,0}},
    {group='R2_Targets', name='R2_PCB_PLACE_TCP',      pos={-1.15,  0.20, 0.29}, ori={0,0,0}},

    -- R3
    {group='R3_Targets', name='R3_HOME_REF',           pos={-0.55,  0.28, 0.80}, ori={0,0,0}},
    {group='R3_Targets', name='R3_MODULE_PICK_APP',    pos={-0.85, -0.05, 0.45}, ori={0,0,0}},
    {group='R3_Targets', name='R3_MODULE_PICK_TCP',    pos={-0.85, -0.05, 0.24}, ori={0,0,0}},
    {group='R3_Targets', name='R3_MODULE_PLACE_APP',   pos={-1.12,  0.22, 0.50}, ori={0,0,0}},
    {group='R3_Targets', name='R3_MODULE_PLACE_TCP',   pos={-1.12,  0.22, 0.34}, ori={0,0,0}},
    {group='R3_Targets', name='R3_PRODUCT_PICK_APP',   pos={-1.15,  0.20, 0.60}, ori={0,0,0}},
    {group='R3_Targets', name='R3_PRODUCT_PICK_TCP',   pos={-1.15,  0.20, 0.34}, ori={0,0,0}},
    {group='R3_Targets', name='R3_PRODUCT_PLACE_INSPECTION_APP', pos={0.15, 0.05, 0.60}, ori={0,0,0}},
    {group='R3_Targets', name='R3_PRODUCT_PLACE_INSPECTION_TCP', pos={0.15, 0.05, 0.34}, ori={0,0,0}},

    -- Camera
    {group='Sensor_Targets', name='CAMERA_INSPECTION_CENTER', pos={0.15, 0.05, 0.55}, ori={0,0,0}},

    -- R4
    {group='R4_Targets', name='R4_HOME_REF',           pos={0.58,  0.25, 0.80}, ori={0,0,0}},
    {group='R4_Targets', name='R4_SCREW_APP',          pos={0.21, -0.02, 0.55}, ori={0,0,0}},
    {group='R4_Targets', name='R4_SCREW_TCP',          pos={0.21, -0.02, 0.36}, ori={0,0,0}},
    {group='R4_Targets', name='R4_SCREW_PRESS',        pos={0.21, -0.02, 0.33}, ori={0,0,0}},

    -- R5
    {group='R5_Targets', name='R5_HOME_REF',           pos={0.15, -0.50, 0.80}, ori={0,0,0}},
    {group='R5_Targets', name='R5_PRODUCT_PICK_APP',   pos={0.15,  0.05, 0.60}, ori={0,0,0}},
    {group='R5_Targets', name='R5_PRODUCT_PICK_TCP',   pos={0.15,  0.05, 0.34}, ori={0,0,0}},
    {group='R5_Targets', name='R5_GOOD_PLACE_APP',     pos={0.65, -1.10, 0.62}, ori={0,0,0}},
    {group='R5_Targets', name='R5_GOOD_PLACE_TCP',     pos={0.65, -1.10, 0.42}, ori={0,0,0}},
    {group='R5_Targets', name='R5_DEFECT_PLACE_APP',   pos={-0.35, -1.12, 0.62}, ori={0,0,0}},
    {group='R5_Targets', name='R5_DEFECT_PLACE_TCP',   pos={-0.35, -1.12, 0.42}, ori={0,0,0}},
}

local function getOrCreateTargetGroup(groupName)
    local path = '/FiveCR5A_Cell/Targets/' .. groupName
    local g = safeGet(path)

    if g ~= -1 then return g end

    local targetRoot = safeGet('/FiveCR5A_Cell/Targets')

    if targetRoot == -1 then
        targetRoot = makeGroup('Targets', root)
    end

    g = makeGroup(groupName, targetRoot, {0,0,0})
    return g
end

local function createOrResetTarget(cfg)
    local group = getOrCreateTargetGroup(cfg.group)
    local path = '/FiveCR5A_Cell/Targets/' .. cfg.group .. '/' .. cfg.name

    local h = safeGet(path)

    if h == -1 then
        h = targetDummy(cfg.name, cfg.pos, cfg.ori, group)
        print('[TARGET OK] Created ' .. cfg.name)
    elseif RESET_TARGETS_ON_START then
        setWorldPosition(h, cfg.pos)
        setWorldOrientation(h, cfg.ori)
        print('[TARGET OK] Reset ' .. cfg.name)
    else
        print('[TARGET KEEP] Existing ' .. cfg.name .. ' kept.')
    end

    if SHOW_TARGET_DUMMIES then
        pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 1)
    else
        pcall(sim.setObjectInt32Param, h, sim.objintparam_visibility_layer, 0)
    end
end

local function createAllTargets()
    print('========== Creating process targets ==========')

    for i = 1, #targetConfigs do
        createOrResetTarget(targetConfigs[i])
    end

    print('========== Target creation finished ==========')
end


-- =========================================================
-- ???????????
-- =========================================================

local function findRobotRoot(name)
    local h = safeGet('/' .. name)
    if h ~= -1 then return h end
    return -1
end

local function moveRobot(name)
    local robot = findRobotRoot(name)

    if robot == -1 then
        print('[ROBOT WARN] Cannot find /' .. name)
        return
    end

    local base = safeGet('/FiveCR5A_Cell/RobotBases/' .. name .. '_Base')
    local targetPos = nil

    if base ~= -1 then
        local bp = getWorldPosition(base)
        targetPos = {bp[1], bp[2], bp[3] + ROBOT_Z_OFFSET}
    else
        local b = robotBases[name]
        targetPos = {b[1], b[2], b[3] + ROBOT_Z_OFFSET}
    end

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
    if not FREEZE_ROBOT_DYNAMICS_ON_START then return end

    freezeRobot('R1')
    freezeRobot('R2')
    freezeRobot('R3')
    freezeRobot('R4')
    freezeRobot('R5')
end


-- =========================================================
-- ?????
-- =========================================================

local function checkReach(baseName, targetPath)
    if not ENABLE_REACH_CHECK then return end

    local b = safeGet('/FiveCR5A_Cell/RobotBases/' .. baseName .. '_Base')
    local t = safeGet(targetPath)

    if b == -1 or t == -1 then return end

    local bp = getWorldPosition(b)
    local tp = getWorldPosition(t)

    local dx = tp[1] - bp[1]
    local dy = tp[2] - bp[2]
    local d = math.sqrt(dx * dx + dy * dy)

    local msg = baseName .. ' -> ' .. targetPath .. ' = ' .. string.format('%.3f', d) .. ' m'

    if d < 0.25 then
        print('[TOO CLOSE] ' .. msg)
    elseif d > 0.85 then
        print('[TOO FAR] ' .. msg)
    else
        print('[REACH OK] ' .. msg)
    end
end

local function checkAllReach()
    if not ENABLE_REACH_CHECK then return end

    print('========== Reachability Check ==========')

    checkReach('R1', '/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_TCP')
    checkReach('R1', '/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PICK_TCP')
    checkReach('R1', '/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PLACE_TCP')

    checkReach('R2', '/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PICK_TCP')
    checkReach('R2', '/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PLACE_TCP')

    checkReach('R3', '/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PICK_TCP')
    checkReach('R3', '/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PLACE_INSPECTION_TCP')

    checkReach('R4', '/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_TCP')

    checkReach('R5', '/FiveCR5A_Cell/Targets/R5_Targets/R5_PRODUCT_PICK_TCP')
    checkReach('R5', '/FiveCR5A_Cell/Targets/R5_Targets/R5_GOOD_PLACE_TCP')
    checkReach('R5', '/FiveCR5A_Cell/Targets/R5_Targets/R5_DEFECT_PLACE_TCP')

    print('========================================')
end


-- =========================================================
-- ??????????????
-- =========================================================

local function processProductStateSignal()
    local state = sim.getStringSignal('cell_product_state')

    if state == nil then return end

    if state == 'reset' then
        resetRuntimeState()

    elseif state == 'assembly_shell' then
        setTreeVisible('/FiveCR5A_Cell/Parts/Box_Blank', false)
        setProductStage('Assembly_ControlBox_Product', 1)

    elseif state == 'assembly_pcb' then
        setTreeVisible('/FiveCR5A_Cell/Parts/PCB_Supply', false)
        setProductStage('Assembly_ControlBox_Product', 2)

    elseif state == 'assembly_module' then
        setTreeVisible('/FiveCR5A_Cell/Parts/Control_Module_Supply', false)
        setProductStage('Assembly_ControlBox_Product', 3)

    elseif state == 'assembly_full' then
        setTreeVisible('/FiveCR5A_Cell/Parts/Terminal_Block_Supply', false)
        setProductStage('Assembly_ControlBox_Product', 4)

    elseif state == 'inspection_full' then
        setProductStage('Assembly_ControlBox_Product', 0)
        setProductStage('Inspection_ControlBox_Product', 4)

        local product = safeGet('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product')
        setWorldPosition(product, {P.inspection[1], P.inspection[2], INSPECTION_BOX_Z})

    elseif state == 'camera_good' then
        setTreeColor('/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area', {0.20, 0.90, 0.25})

    elseif state == 'camera_defect' then
        setTreeColor('/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area', {0.95, 0.15, 0.10})
    end

    sim.clearStringSignal('cell_product_state')
end

local function processConveyorSignal()
    local state = sim.getStringSignal('cell_conveyor_state')

    if state == nil then return end

    local product = safeGet('/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product')

    if state == 'good' then
        activeProduct = product
        activeConveyor = 'good'
        conveyorStartTime = sim.getSimulationTime()

        setProductStage('Inspection_ControlBox_Product', 4)
        setWorldPosition(activeProduct, goodStart)

        print('[CONVEYOR] Good product started.')

    elseif state == 'defect' then
        activeProduct = product
        activeConveyor = 'defect'
        conveyorStartTime = sim.getSimulationTime()

        setProductStage('Inspection_ControlBox_Product', 4)
        setWorldPosition(activeProduct, defectStart)

        print('[CONVEYOR] Defect product started.')
    end

    sim.clearStringSignal('cell_conveyor_state')
end


-- =========================================================
-- ?????
-- =========================================================

local function moveAlongLine(obj, p0, p1, speed)
    if obj == -1 then return end

    local t = sim.getSimulationTime() - conveyorStartTime

    local dx = p1[1] - p0[1]
    local dy = p1[2] - p0[2]
    local dz = p1[3] - p0[3]

    local dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 0.0001 then return end

    local moveTime = dist / speed
    local a = t / moveTime
    if a > 1 then a = 1 end

    local p = {
        p0[1] + dx * a,
        p0[2] + dy * a,
        p0[3] + dz * a
    }

    setWorldPosition(obj, p)
end


-- =========================================================
-- CoppeliaSim ??
-- =========================================================

function sysCall_init()
    local oldRoot = safeGet('/FiveCR5A_Cell')

    if oldRoot ~= -1 and REBUILD_SCENE_ON_START then
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
    createAllTips()
    createAllTargets()
    freezeAllRobots()
    resetRuntimeState()
    checkAllReach()

    print('[OK] Main cell initialization finished.')
    print('[NEXT] After first successful run, set REBUILD_SCENE_ON_START=false, RESET_TIPS_ON_START=false, RESET_TARGETS_ON_START=false.')
end

function sysCall_actuation()
    processProductStateSignal()
    processConveyorSignal()

    if CONVEYOR_ENABLED and activeProduct ~= -1 then
        if activeConveyor == 'good' then
            moveAlongLine(activeProduct, goodStart, goodEnd, conveyorSpeed)
        elseif activeConveyor == 'defect' then
            moveAlongLine(activeProduct, defectStart, defectEnd, conveyorSpeed)
        end
    end
end

function sysCall_cleanup()
end
